#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/detected_object_kinematics.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tier4_perception_msgs/msg/detected_object_with_feature.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
#include <tf2/convert.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class ClassAgnosticObstacleDetector : public rclcpp::Node
{
public:
  using DetectedObjects = tier4_perception_msgs::msg::DetectedObjectsWithFeature;
  using DetectedObject = tier4_perception_msgs::msg::DetectedObjectWithFeature;

  ClassAgnosticObstacleDetector()
  : Node("class_agnostic_obstacle_detector"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    canny_low_ = declare_parameter<int>("canny_low", 45);
    canny_high_ = declare_parameter<int>("canny_high", 135);
    min_area_ratio_ = declare_parameter<double>("min_area_ratio", 0.003);
    max_area_ratio_ = declare_parameter<double>("max_area_ratio", 0.55);
    min_width_ = declare_parameter<int>("min_width", 28);
    min_height_ = declare_parameter<int>("min_height", 28);
    merge_gap_ = declare_parameter<int>("merge_gap", 18);
    bottom_roi_ratio_ = declare_parameter<double>("bottom_roi_ratio", 0.95);
    known_timeout_ = declare_parameter<double>("known_timeout", 0.25);
    depth_timeout_ = declare_parameter<double>("depth_timeout", 0.15);
    depth_scale_ = declare_parameter<double>("depth_scale", 0.001);
    depth_percentile_ = declare_parameter<double>("depth_percentile", 0.20);
    min_depth_m_ = declare_parameter<double>("min_depth_m", 0.20);
    max_depth_m_ = declare_parameter<double>("max_depth_m", 80.0);
    object_depth_m_ = declare_parameter<double>("object_depth_m", 0.60);
    target_frame_ = declare_parameter<std::string>("target_frame", "");
    tf_timeout_ = declare_parameter<double>("tf_timeout", 0.05);
    enable_round_contour_ = declare_parameter<bool>("enable_round_contour", true);
    round_min_radius_px_ = declare_parameter<int>("round_min_radius_px", 15);
    round_max_radius_px_ = declare_parameter<int>("round_max_radius_px", 110);
    round_circularity_min_ = declare_parameter<double>("round_circularity_min", 0.68);
    round_aspect_ratio_max_ = declare_parameter<double>("round_aspect_ratio_max", 1.55);
    round_min_area_ratio_ = declare_parameter<double>("round_min_area_ratio", 0.0005);
    round_max_area_ratio_ = declare_parameter<double>("round_max_area_ratio", 0.08);
    round_roi_top_ratio_ = declare_parameter<double>("round_roi_top_ratio", 0.15);
    round_min_fill_ratio_ = declare_parameter<double>("round_min_fill_ratio", 0.50);
    const auto qos = rclcpp::SensorDataQoS();
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "~/input/image", qos,
      std::bind(&ClassAgnosticObstacleDetector::onImage, this, std::placeholders::_1));
    known_sub_ = create_subscription<DetectedObjects>(
      "~/input/known_objects", qos,
      std::bind(&ClassAgnosticObstacleDetector::onKnownObjects, this, std::placeholders::_1));
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "~/input/depth", qos,
      std::bind(&ClassAgnosticObstacleDetector::onDepth, this, std::placeholders::_1));
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "~/input/camera_info", qos,
      std::bind(&ClassAgnosticObstacleDetector::onCameraInfo, this, std::placeholders::_1));
    objects_pub_ = create_publisher<DetectedObjects>("~/output/objects", 10);
    objects_3d_pub_ = create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
      "~/output/objects_3d", 10);
    autoware_objects_3d_pub_ =
      create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
      "~/output/autoware_objects_3d", 10);
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("~/output/debug_image", 10);
    nearest_distance_pub_ = create_publisher<std_msgs::msg::Float32>(
      "~/output/nearest_distance", 10);
  }

private:
  static bool intersectsOrNear(const cv::Rect & a, const cv::Rect & b, const int gap)
  {
    const cv::Rect expanded(a.x - gap, a.y - gap, a.width + 2 * gap, a.height + 2 * gap);
    return (expanded & b).area() > 0;
  }

  static double intersectionOverUnion(const cv::Rect & a, const cv::Rect & b)
  {
    const double intersection = static_cast<double>((a & b).area());
    const double union_area = static_cast<double>(a.area() + b.area()) - intersection;
    return union_area > 0.0 ? intersection / union_area : 0.0;
  }

  static std::vector<cv::Rect> mergeRects(std::vector<cv::Rect> rects, const int gap)
  {
    bool changed = true;
    while (changed) {
      changed = false;
      for (std::size_t i = 0; i < rects.size() && !changed; ++i) {
        for (std::size_t j = i + 1; j < rects.size(); ++j) {
          if (intersectsOrNear(rects[i], rects[j], gap)) {
            rects[i] |= rects[j];
            rects.erase(rects.begin() + static_cast<std::ptrdiff_t>(j));
            changed = true;
            break;
          }
        }
      }
    }
    return rects;
  }

  std::vector<cv::Rect> findUnknownObstacles(const cv::Mat & image) const
  {
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0.0);

    cv::Mat edges;
    cv::Canny(gray, edges, canny_low_, canny_high_);
    const int roi_height = std::clamp(
      static_cast<int>(std::lround(image.rows * bottom_roi_ratio_)), 1, image.rows);
    if (roi_height < image.rows) {
      edges.rowRange(0, image.rows - roi_height).setTo(0);
    }

    const auto close_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9));
    const auto dilate_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(edges, edges, cv::MORPH_CLOSE, close_kernel, cv::Point(-1, -1), 2);
    cv::dilate(edges, edges, dilate_kernel, cv::Point(-1, -1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    const double image_area = static_cast<double>(image.cols * image.rows);
    std::vector<cv::Rect> rects;
    for (const auto & contour : contours) {
      cv::Rect rect = cv::boundingRect(contour) & cv::Rect(0, 0, image.cols, image.rows);
      const double area_ratio = static_cast<double>(rect.area()) / image_area;
      if (rect.width < min_width_ || rect.height < min_height_) continue;
      if (area_ratio < min_area_ratio_ || area_ratio > max_area_ratio_) continue;
      if (rect.width > image.cols * 0.95 || rect.height > image.rows * 0.98) continue;
      rects.push_back(rect);
    }
    return mergeRects(std::move(rects), merge_gap_);
  }

  std::vector<cv::Rect> findRoundOrEllipseObstacles(const cv::Mat & image) const
  {
    std::vector<RoundOrEllipseProposal> candidates;
    if (!enable_round_contour_) return {};

    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    auto clahe = cv::createCLAHE(1.5, cv::Size(8, 8));
    clahe->apply(gray, gray);
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0.0);

    cv::Mat edges;
    cv::Canny(
      gray, edges, std::max(20.0, canny_low_ * 0.75),
      std::max(60.0, canny_high_ * 0.85));

    // Close only small contour gaps. Do not dilate the full image because
    // unrelated road edges could then become one large candidate.
    cv::morphologyEx(
      edges, edges, cv::MORPH_CLOSE,
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
    const int top = std::clamp(
      static_cast<int>(std::lround(image.rows * round_roi_top_ratio_)), 0, image.rows - 1);
    edges.rowRange(0, top).setTo(0);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    const double image_area = static_cast<double>(image.cols * image.rows);
    for (const auto & contour : contours) {
      const double area = cv::contourArea(contour);
      const double perimeter = cv::arcLength(contour, true);
      if (area <= 0.0 || perimeter <= 0.0) continue;

      const cv::Rect rect = cv::boundingRect(contour) & cv::Rect(0, 0, image.cols, image.rows);
      const int diameter = std::max(rect.width, rect.height);
      if (diameter < 2 * round_min_radius_px_ || diameter > 2 * round_max_radius_px_) continue;

      const double rect_area_ratio = static_cast<double>(rect.area()) / image_area;
      if (rect_area_ratio < round_min_area_ratio_ || rect_area_ratio > round_max_area_ratio_) continue;

      const double aspect = static_cast<double>(std::max(rect.width, rect.height)) /
        std::max(1, std::min(rect.width, rect.height));
      if (aspect > round_aspect_ratio_max_) continue;

      // Fit an ellipse as a second shape check. Both the bounding box and the
      // fitted ellipse must stay within the aspect-ratio limit so a long road
      // edge cannot pass as an ellipse merely because its contour is closed.
      if (contour.size() < 5) continue;
      double fitted_aspect = 0.0;
      try {
        const auto fitted = cv::fitEllipse(contour);
        const double major = std::max(fitted.size.width, fitted.size.height);
        const double minor = std::min(fitted.size.width, fitted.size.height);
        if (minor <= 0.0) continue;
        fitted_aspect = major / minor;
      } catch (const cv::Exception &) {
        continue;
      }
      if (fitted_aspect > round_aspect_ratio_max_) continue;

      const double circularity =
        4.0 * 3.14159265358979323846 * area / (perimeter * perimeter);
      if (circularity < round_circularity_min_) continue;

      // A circular contour should occupy a meaningful part of its box.
      const double fill_ratio = area / static_cast<double>(rect.area());
      if (fill_ratio < round_min_fill_ratio_) continue;
      candidates.push_back({rect, circularity * fill_ratio});
    }

    // Suppress overlapping circle candidates only. Do not merge them into a
    // larger rectangle and do not alter the original unknown-obstacle boxes.
    std::sort(
      candidates.begin(), candidates.end(),
      [](const RoundOrEllipseProposal & a, const RoundOrEllipseProposal & b) {
        return a.score > b.score;
      });
    std::vector<cv::Rect> result;
    for (const auto & candidate : candidates) {
      const bool duplicate = std::any_of(
        result.begin(), result.end(),
        [&candidate](const cv::Rect & kept) {
          const double intersection = static_cast<double>((candidate.rect & kept).area());
          const double smaller = static_cast<double>(std::min(candidate.rect.area(), kept.area()));
          return intersectionOverUnion(candidate.rect, kept) > 0.35 ||
            (smaller > 0.0 && intersection / smaller > 0.50);
        });
      if (!duplicate) result.push_back(candidate.rect);
    }
    return result;
  }

  void onKnownObjects(const DetectedObjects::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(known_mutex_);
    latest_known_ = msg;
  }

  void onDepth(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    latest_depth_ = msg;
  }

  void onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    latest_camera_info_ = msg;
  }

  struct DepthMeasurement
  {
    geometry_msgs::msg::Point position;
    double range_m{std::numeric_limits<double>::quiet_NaN()};
    double forward_m{std::numeric_limits<double>::quiet_NaN()};
    double width_m{0.0};
    double height_m{0.0};
    // The image overlay only needs a valid depth sample.  Keep this separate
    // from the optional camera-to-vehicle transform used by the 3D output.
    bool in_target_frame{false};
  };

  struct RoundOrEllipseProposal
  {
    cv::Rect rect;
    double score{0.0};
  };

  std::optional<DepthMeasurement> estimatePosition(
    const cv::Rect & roi, const sensor_msgs::msg::Image & image_msg)
  {
    sensor_msgs::msg::Image::ConstSharedPtr depth_msg;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info;
    {
      std::lock_guard<std::mutex> lock(sensor_mutex_);
      depth_msg = latest_depth_;
      camera_info = latest_camera_info_;
    }
    if (!depth_msg || !camera_info || depth_msg->header.frame_id.empty()) {
      return std::nullopt;
    }

    const auto image_time = rclcpp::Time(image_msg.header.stamp);
    const auto depth_time = rclcpp::Time(depth_msg->header.stamp);
    if (std::abs((image_time - depth_time).seconds()) > depth_timeout_) {
      return std::nullopt;
    }

    cv_bridge::CvImageConstPtr depth_cv;
    try {
      depth_cv = cv_bridge::toCvShare(depth_msg);
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "depth cv_bridge: %s", error.what());
      return std::nullopt;
    }

    const cv::Mat & depth = depth_cv->image;
    if (depth.empty() || (depth.type() != CV_16UC1 && depth.type() != CV_32FC1)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Unsupported depth encoding: %s", depth_msg->encoding.c_str());
      return std::nullopt;
    }

    const auto clipped = roi & cv::Rect(0, 0, static_cast<int>(image_msg.width),
      static_cast<int>(image_msg.height));
    if (clipped.width < 2 || clipped.height < 2) return std::nullopt;

    // Ignore the box border, which often contains background or neighboring objects.
    const int left = clipped.x + clipped.width / 5;
    const int right = clipped.x + clipped.width * 4 / 5;
    const int top = clipped.y + clipped.height / 5;
    const int bottom = clipped.y + clipped.height * 9 / 10;
    if (right <= left || bottom <= top) return std::nullopt;

    const double image_width = camera_info->width > 0 ? camera_info->width : image_msg.width;
    const double image_height = camera_info->height > 0 ? camera_info->height : image_msg.height;
    if (camera_info->k[0] <= 0.0 || camera_info->k[4] <= 0.0 || image_width <= 0.0 || image_height <= 0.0) {
      return std::nullopt;
    }

    const double depth_scale_x = static_cast<double>(depth.cols) / image_width;
    const double depth_scale_y = static_cast<double>(depth.rows) / image_height;
    const int depth_left = std::clamp(static_cast<int>(std::floor(left * depth_scale_x)), 0, depth.cols - 1);
    const int depth_right = std::clamp(static_cast<int>(std::ceil(right * depth_scale_x)), 1, depth.cols);
    const int depth_top = std::clamp(static_cast<int>(std::floor(top * depth_scale_y)), 0, depth.rows - 1);
    const int depth_bottom = std::clamp(static_cast<int>(std::ceil(bottom * depth_scale_y)), 1, depth.rows);
    if (depth_right <= depth_left || depth_bottom <= depth_top) return std::nullopt;

    std::vector<double> depths;
    const int step_x = std::max(1, (depth_right - depth_left) / 32);
    const int step_y = std::max(1, (depth_bottom - depth_top) / 24);
    depths.reserve(32 * 24);
    for (int v = depth_top; v < depth_bottom; v += step_y) {
      for (int u = depth_left; u < depth_right; u += step_x) {
        double z = 0.0;
        if (depth.type() == CV_16UC1) {
          // RealSense 16UC1 depth is normally expressed in millimetres.
          z = static_cast<double>(depth.at<uint16_t>(v, u)) * depth_scale_;
        } else {
          // ROS 32FC1 depth images are expressed in metres.
          z = static_cast<double>(depth.at<float>(v, u));
        }
        if (std::isfinite(z) && z >= min_depth_m_ && z <= max_depth_m_) depths.push_back(z);
      }
    }
    if (depths.size() < 8) return std::nullopt;

    const double percentile = std::clamp(depth_percentile_, 0.0, 1.0);
    const auto index = static_cast<std::size_t>((depths.size() - 1) * percentile);
    std::nth_element(depths.begin(), depths.begin() + static_cast<std::ptrdiff_t>(index), depths.end());
    const double z = depths[index];

    const double u = ((static_cast<double>(clipped.x) + clipped.width * 0.5) * depth_scale_x);
    const double v = ((static_cast<double>(clipped.y) + clipped.height * 0.5) * depth_scale_y);
    const double fx = camera_info->k[0] * depth_scale_x;
    const double fy = camera_info->k[4] * depth_scale_y;
    const double cx = camera_info->k[2] * depth_scale_x;
    const double cy = camera_info->k[5] * depth_scale_y;

    geometry_msgs::msg::PointStamped point_camera;
    point_camera.header = depth_msg->header;
    point_camera.point.z = z;
    point_camera.point.x = (u - cx) * z / fx;
    point_camera.point.y = (v - cy) * z / fy;

    geometry_msgs::msg::Point point = point_camera.point;
    bool in_target_frame = target_frame_.empty() || target_frame_ == point_camera.header.frame_id;
    if (!target_frame_.empty() && target_frame_ != point_camera.header.frame_id) {
      try {
        point = tf_buffer_.transform(
          point_camera, target_frame_, tf2::durationFromSec(tf_timeout_)).point;
        in_target_frame = true;
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000, "Cannot transform depth point %s -> %s: %s",
          point_camera.header.frame_id.c_str(), target_frame_.c_str(), error.what());
        // A missing TF must not hide the valid distance in the debug image.
        // Keep the point in the camera frame and suppress only the 3D output.
        point = point_camera.point;
      }
    }

    DepthMeasurement measurement;
    measurement.position = point;
    const auto & camera_point = point_camera.point;
    measurement.range_m = std::sqrt(
      camera_point.x * camera_point.x + camera_point.y * camera_point.y +
      camera_point.z * camera_point.z);
    // In an optical frame, +Z points forward. In a vehicle frame such as
    // base_link, +X points forward.
    const bool target_is_vehicle_frame = in_target_frame && !target_frame_.empty() &&
      target_frame_ != point_camera.header.frame_id &&
      target_frame_.find("optical_frame") == std::string::npos;
    measurement.forward_m = target_is_vehicle_frame ? point.x : camera_point.z;
    measurement.width_m = std::max(0.05, (right - left) * z / fx);
    measurement.height_m = std::max(0.05, (bottom - top) * z / fy);
    measurement.in_target_frame = in_target_frame;
    return measurement;
  }

  static std::string formatDistance(const DepthMeasurement & measurement)
  {
    std::ostringstream text;
    text << std::fixed << std::setprecision(2) << "DIST " << measurement.range_m << "m FWD " <<
      measurement.forward_m << "m";
    return text.str();
  }

  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge: %s", error.what());
      return;
    }

    cv::Mat debug_image = cv_ptr->image.clone();
    auto unknown_rects = findUnknownObstacles(debug_image);
    auto round_rects = findRoundOrEllipseObstacles(debug_image);
    DetectedObjects output;
    output.header = msg->header;
    autoware_perception_msgs::msg::DetectedObjects output_3d;
    output_3d.header = msg->header;
    if (!target_frame_.empty()) output_3d.header.frame_id = target_frame_;
    nearest_distance_m_ = std::numeric_limits<double>::infinity();

    std::vector<cv::Rect> known_rects;
    std::vector<bool> round_flags;
    DetectedObjects::ConstSharedPtr known;
    {
      std::lock_guard<std::mutex> lock(known_mutex_);
      known = latest_known_;
    }
    if (known) {
      const auto image_time = rclcpp::Time(msg->header.stamp);
      const auto known_time = rclcpp::Time(known->header.stamp);
      if (std::abs((image_time - known_time).seconds()) <= known_timeout_) {
        output.feature_objects = known->feature_objects;
        round_flags.assign(output.feature_objects.size(), false);
        for (const auto & object : known->feature_objects) {
          const auto & roi = object.feature.roi;
          known_rects.emplace_back(roi.x_offset, roi.y_offset, roi.width, roi.height);
        }
      }
    }

    for (const auto & rect : unknown_rects) {
      const bool covered_by_known = std::any_of(
        known_rects.begin(), known_rects.end(),
        [&rect](const cv::Rect & known_rect) { return intersectionOverUnion(rect, known_rect) > 0.35; });
      if (covered_by_known) continue;

      DetectedObject object;
      object.feature.roi.x_offset = static_cast<uint32_t>(rect.x);
      object.feature.roi.y_offset = static_cast<uint32_t>(rect.y);
      object.feature.roi.width = static_cast<uint32_t>(rect.width);
      object.feature.roi.height = static_cast<uint32_t>(rect.height);
      object.object.existence_probability = 0.5F;
      autoware_perception_msgs::msg::ObjectClassification classification;
      classification.label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
      classification.probability = 1.0F;
      object.object.classification.push_back(classification);
      output.feature_objects.push_back(object);
      round_flags.push_back(false);
    }

    // Add circular candidates after the original detector has completed. This
    // keeps the existing YOLOX/Canny chain unchanged and only suppresses a
    // circle when it duplicates an already existing box.
    for (const auto & rect : round_rects) {
      const auto overlaps = [&rect](const cv::Rect & other) {
        const double intersection = static_cast<double>((rect & other).area());
        const double smaller = static_cast<double>(std::min(rect.area(), other.area()));
        return intersectionOverUnion(rect, other) > 0.25 ||
          (smaller > 0.0 && intersection / smaller > 0.50);
      };
      if (std::any_of(known_rects.begin(), known_rects.end(), overlaps) ||
        std::any_of(unknown_rects.begin(), unknown_rects.end(), overlaps)) {
        continue;
      }

      DetectedObject object;
      object.feature.roi.x_offset = static_cast<uint32_t>(rect.x);
      object.feature.roi.y_offset = static_cast<uint32_t>(rect.y);
      object.feature.roi.width = static_cast<uint32_t>(rect.width);
      object.feature.roi.height = static_cast<uint32_t>(rect.height);
      object.object.existence_probability = 0.5F;
      autoware_perception_msgs::msg::ObjectClassification classification;
      classification.label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
      classification.probability = 1.0F;
      object.object.classification.push_back(classification);
      output.feature_objects.push_back(object);
      round_flags.push_back(true);
    }

    for (std::size_t i = 0; i < output.feature_objects.size(); ++i) {
      auto & object = output.feature_objects[i];
      const auto & roi = object.feature.roi;
      const cv::Rect rect(roi.x_offset, roi.y_offset, roi.width, roi.height);
      const bool is_unknown = object.object.classification.empty() ||
        object.object.classification.front().label ==
          autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
      const bool is_round = i < round_flags.size() && round_flags[i];
      const cv::Scalar color = is_round ? cv::Scalar(255, 0, 255) :
        (is_unknown ? cv::Scalar(0, 200, 255) : cv::Scalar(0, 255, 0));
      const auto measurement = estimatePosition(rect, *msg);
      std::string label = is_round ? "ROUND/ELLIPSE OBSTACLE" :
        (is_unknown ? "UNKNOWN OBSTACLE" : "KNOWN");
      if (measurement) {
        object.object.kinematics.pose_with_covariance.pose.position = measurement->position;
        object.object.kinematics.has_position_covariance = false;
        object.object.kinematics.orientation_availability =
          autoware_perception_msgs::msg::DetectedObjectKinematics::UNAVAILABLE;
        object.object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
        object.object.shape.dimensions.x = object_depth_m_;
        object.object.shape.dimensions.y = measurement->width_m;
        object.object.shape.dimensions.z = measurement->height_m;
        if (measurement->in_target_frame) {
          output_3d.objects.push_back(object.object);
        }
        if (measurement->forward_m > 0.0 && measurement->forward_m < nearest_distance_m_) {
          nearest_distance_m_ = measurement->forward_m;
        }
        label += " " + formatDistance(*measurement);
      } else {
        label += " DIST N/A";
      }
      cv::rectangle(debug_image, rect, color, 2, cv::LINE_AA);
      cv::putText(
        debug_image, label, cv::Point(rect.x, std::max(18, rect.y - 5)),
        cv::FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv::LINE_AA);
    }

    objects_pub_->publish(output);
    objects_3d_pub_->publish(output_3d);
    autoware_objects_3d_pub_->publish(output_3d);
    std_msgs::msg::Float32 nearest_distance_msg;
    nearest_distance_msg.data = std::isfinite(nearest_distance_m_) ?
      static_cast<float>(nearest_distance_m_) : -1.0F;
    nearest_distance_pub_->publish(nearest_distance_msg);
    auto debug_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, debug_image).toImageMsg();
    image_pub_->publish(*debug_msg);
  }

  int canny_low_;
  int canny_high_;
  double min_area_ratio_;
  double max_area_ratio_;
  int min_width_;
  int min_height_;
  int merge_gap_;
  double bottom_roi_ratio_;
  double known_timeout_;
  double depth_timeout_;
  double depth_scale_;
  double depth_percentile_;
  double min_depth_m_;
  double max_depth_m_;
  double object_depth_m_;
  std::string target_frame_;
  double tf_timeout_;
  bool enable_round_contour_;
  int round_min_radius_px_;
  int round_max_radius_px_;
  double round_circularity_min_;
  double round_aspect_ratio_max_;
  double round_min_area_ratio_;
  double round_max_area_ratio_;
  double round_roi_top_ratio_;
  double round_min_fill_ratio_;
  std::mutex known_mutex_;
  std::mutex sensor_mutex_;
  DetectedObjects::ConstSharedPtr latest_known_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_depth_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_camera_info_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<DetectedObjects>::SharedPtr known_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<DetectedObjects>::SharedPtr objects_pub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr objects_3d_pub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr
    autoware_objects_3d_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr nearest_distance_pub_;
  double nearest_distance_m_{std::numeric_limits<double>::infinity()};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClassAgnosticObstacleDetector>());
  rclcpp::shutdown();
  return 0;
}
