#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

class LidarCameraOverlayNode : public rclcpp::Node
{
public:
  LidarCameraOverlayNode()
  : Node("lidar_camera_overlay"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    image_topic_ = declare_parameter<std::string>(
      "image_topic", "/sensing/camera/camera0/image_rect_color");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/sensing/camera/camera0/camera_info");
    pointcloud_topic_ = declare_parameter<std::string>(
      "pointcloud_topic", "/sensing/lidar/concatenated/pointcloud");
    output_topic_ = declare_parameter<std::string>("output_topic", "/debug/lidar_camera_overlay");
    camera_frame_ = declare_parameter<std::string>("camera_frame", "camera0_color_optical_frame");
    use_distortion_ = declare_parameter<bool>("use_distortion", true);
    point_stride_ = std::max(1, static_cast<int>(declare_parameter<int>("point_stride", 2)));
    point_size_ = std::max(1, static_cast<int>(declare_parameter<int>("point_size", 2)));
    sync_queue_size_ =
      std::max(2, static_cast<int>(declare_parameter<int>("sync_queue_size", 20)));
    sync_max_interval_sec_ = declare_parameter<double>("sync_max_interval_sec", 0.10);
    min_distance_ = declare_parameter<double>("min_distance", 0.2);
    max_distance_ = declare_parameter<double>("max_distance", 80.0);

    output_pub_ = create_publisher<sensor_msgs::msg::Image>(output_topic_, rclcpp::QoS(1));

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::QoS(1).best_effort(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(camera_info_mutex_);
        camera_info_ = std::move(msg);
      });

    image_sub_.subscribe(this, image_topic_, rmw_qos_profile_sensor_data);
    pointcloud_sub_.subscribe(this, pointcloud_topic_, rmw_qos_profile_sensor_data);

    sync_ = std::make_shared<Synchronizer>(
      SyncPolicy(sync_queue_size_), image_sub_, pointcloud_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_max_interval_sec_));
    sync_->registerCallback(
      std::bind(
        &LidarCameraOverlayNode::on_image_and_pointcloud, this, std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "LiDAR-camera overlay started.");
    RCLCPP_INFO(get_logger(), "  image: %s", image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  camera info: %s", camera_info_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  pointcloud: %s", pointcloud_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  output: %s", output_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  camera frame: %s", camera_frame_.c_str());
  }

private:
  using Image = sensor_msgs::msg::Image;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, PointCloud2>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  static cv::Mat make_camera_matrix(const sensor_msgs::msg::CameraInfo & info)
  {
    return (cv::Mat_<double>(3, 3) << info.k[0], info.k[1], info.k[2], info.k[3], info.k[4],
      info.k[5], info.k[6], info.k[7], info.k[8]);
  }

  static cv::Mat make_distortion_matrix(const sensor_msgs::msg::CameraInfo & info)
  {
    if (info.d.empty()) {
      return cv::Mat::zeros(1, 5, CV_64F);
    }
    cv::Mat distortion(1, static_cast<int>(info.d.size()), CV_64F);
    std::memcpy(distortion.ptr<double>(), info.d.data(), info.d.size() * sizeof(double));
    return distortion;
  }

  cv::Scalar color_for_distance(const double distance) const
  {
    const double ratio = std::clamp(
      (distance - min_distance_) / std::max(0.001, max_distance_ - min_distance_), 0.0, 1.0);
    // BGR: near points are red, far points are blue.
    return cv::Scalar(255.0 * ratio, 40.0, 255.0 * (1.0 - ratio));
  }

  void on_image_and_pointcloud(
    Image::ConstSharedPtr image_msg, PointCloud2::ConstSharedPtr pointcloud_msg)
  {
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info;
    {
      std::lock_guard<std::mutex> lock(camera_info_mutex_);
      camera_info = camera_info_;
    }
    if (!camera_info) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for CameraInfo on %s",
        camera_info_topic_.c_str());
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      // The camera and LiDAR extrinsics are static. TimePointZero avoids rejecting a valid
      // static transform because the sensor messages have different capture timestamps.
      transform = tf_buffer_.lookupTransform(
        camera_frame_, pointcloud_msg->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "TF %s -> %s unavailable: %s",
        pointcloud_msg->header.frame_id.c_str(), camera_frame_.c_str(), ex.what());
      return;
    }

    sensor_msgs::msg::PointCloud2 cloud_in_camera;
    try {
      tf2::doTransform(*pointcloud_msg, cloud_in_camera, transform);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Pointcloud transform failed: %s", ex.what());
      return;
    }

    cv_bridge::CvImagePtr cv_image;
    try {
      cv_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Image conversion failed: %s", ex.what());
      return;
    }

    const auto camera_matrix = make_camera_matrix(*camera_info);
    const auto distortion = make_distortion_matrix(*camera_info);
    std::vector<cv::Point3f> camera_points;
    camera_points.reserve(cloud_in_camera.width * cloud_in_camera.height / point_stride_);

    std::size_t point_index = 0;
    for (
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_in_camera, "x"),
      iter_y(cloud_in_camera, "y"), iter_z(cloud_in_camera, "z");
      iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++point_index) {
      if (point_index % static_cast<std::size_t>(point_stride_) != 0) {
        continue;
      }
      const float x = *iter_x;
      const float y = *iter_y;
      const float z = *iter_z;
      const double distance = std::sqrt(static_cast<double>(x) * x + y * y + z * z);
      if (!std::isfinite(distance) || z <= 0.05F || distance < min_distance_ || distance > max_distance_) {
        continue;
      }
      camera_points.emplace_back(x, y, z);
    }

    std::vector<cv::Point2f> image_points;
    if (!camera_points.empty()) {
      const cv::Mat no_rotation = cv::Mat::zeros(3, 1, CV_64F);
      const cv::Mat no_translation = cv::Mat::zeros(3, 1, CV_64F);
      const cv::Mat projection_distortion = use_distortion_ ? distortion : cv::Mat::zeros(1, 5, CV_64F);
      cv::projectPoints(
        camera_points, no_rotation, no_translation, camera_matrix, projection_distortion,
        image_points);
    }

    std::size_t visible_points = 0;
    for (std::size_t i = 0; i < image_points.size(); ++i) {
      const auto & point = image_points[i];
      if (
        point.x < 0.0F || point.x >= static_cast<float>(cv_image->image.cols) || point.y < 0.0F ||
        point.y >= static_cast<float>(cv_image->image.rows)) {
        continue;
      }
      const auto & camera_point = camera_points[i];
      const double distance = std::sqrt(
        static_cast<double>(camera_point.x) * camera_point.x +
        static_cast<double>(camera_point.y) * camera_point.y +
        static_cast<double>(camera_point.z) * camera_point.z);
      cv::circle(
        cv_image->image, cv::Point(cvRound(point.x), cvRound(point.y)), point_size_,
        color_for_distance(distance), cv::FILLED, cv::LINE_AA);
      ++visible_points;
    }

    const double image_time =
      static_cast<double>(image_msg->header.stamp.sec) + image_msg->header.stamp.nanosec * 1e-9;
    const double cloud_time = static_cast<double>(pointcloud_msg->header.stamp.sec) +
                              pointcloud_msg->header.stamp.nanosec * 1e-9;
    const std::string status =
      "LiDAR overlay  points=" + std::to_string(visible_points) +
      "  sync_dt=" + std::to_string(image_time - cloud_time).substr(0, 7) + " s";
    cv::putText(
      cv_image->image, status, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
      cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

    cv_image->header = image_msg->header;
    output_pub_->publish(*cv_image->toImageMsg());
  }

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string pointcloud_topic_;
  std::string output_topic_;
  std::string camera_frame_;
  bool use_distortion_{true};
  int point_stride_{2};
  int point_size_{2};
  int sync_queue_size_{20};
  double sync_max_interval_sec_{0.10};
  double min_distance_{0.2};
  double max_distance_{80.0};

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr output_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
  std::mutex camera_info_mutex_;

  message_filters::Subscriber<Image> image_sub_;
  message_filters::Subscriber<PointCloud2> pointcloud_sub_;
  std::shared_ptr<Synchronizer> sync_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarCameraOverlayNode>());
  rclcpp::shutdown();
  return 0;
}
