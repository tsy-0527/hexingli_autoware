#ifndef GNSS_INS_LOCALIZER__GNSS_INS_LOCALIZER_HPP_
#define GNSS_INS_LOCALIZER__GNSS_INS_LOCALIZER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <autoware_utils_diagnostics/diagnostics_interface.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/route_state.hpp>

#include <gins2000/msg/gins2000.hpp>

#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace gnss_ins_localizer
{

class GnssInsLocalizer : public rclcpp::Node
{
public:
  explicit GnssInsLocalizer(const rclcpp::NodeOptions & options);

private:
  using GnssIns = gins2000::msg::Gins2000;

  void onGnssIns(const GnssIns::ConstSharedPtr msg);
  void onVectorMap(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg);
  void onGoal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void onRouteState(const autoware_planning_msgs::msg::RouteState::ConstSharedPtr msg);
  void onKinematicState(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void publishGnssInsDiagnostics();

  void handleInitialize(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);
  void handleCheckMissionReady(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);

  bool loadMapOrigin(const std::string & map_projector_info_path);
  void llToMap(double lat, double lon, double & mx, double & my) const;
  tf2::Quaternion ginsAttitudeToMapQuat(double roll_deg, double pitch_deg, double yaw_deg) const;
  bool isPoseUsable(uint8_t status) const;
  bool isFullyAligned(uint8_t status) const;

  std::string map_path_;
  double origin_lat_{0.0};
  double origin_lon_{0.0};
  double origin_utm_x_{0.0};
  double origin_utm_y_{0.0};
  int origin_utm_zone_{0};
  bool origin_utm_northp_{true};
  bool origin_loaded_{false};

  std::string map_frame_;
  std::string base_frame_;
  std::string gins_frame_;
  bool override_gins_frame_id_{true};

  int roll_sign_{1};
  int pitch_sign_{-1};
  bool swap_roll_pitch_source_{true};

  double sigma_xy_full_{0.05};
  double sigma_z_full_{0.10};
  double sigma_yaw_full_deg_{0.3};
  double sigma_xy_degraded_{0.50};
  double sigma_z_degraded_{1.00};
  double sigma_yaw_degraded_deg_{3.0};

  double stale_timeout_sec_{0.2};
  std::string status_mode_{"positive"};
  int required_ins_status_mask_{15};
  bool publish_imu_{false};

  std::mutex mtx_;
  GnssIns::ConstSharedPtr latest_gins_;
  rclcpp::Time last_gins_recv_time_;

  bool vector_map_received_{false};
  bool initialize_called_{false};
  std::optional<geometry_msgs::msg::PoseStamped> last_goal_;
  std::optional<uint16_t> last_route_state_;

  std::deque<rclcpp::Time> kinematic_stamps_;

  rclcpp::Subscription<GnssIns>::SharedPtr sub_gins_;
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<autoware_planning_msgs::msg::RouteState>::SharedPtr sub_route_state_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_kinematic_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_gnss_pose_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_initialpose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_debug_pose_;

  std::unique_ptr<autoware_utils_diagnostics::DiagnosticsInterface> gnss_ins_diagnostics_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_initialize_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_check_mission_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace gnss_ins_localizer

#endif
