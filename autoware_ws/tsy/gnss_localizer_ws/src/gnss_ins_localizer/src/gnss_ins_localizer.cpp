#include "gnss_ins_localizer/gnss_ins_localizer.hpp"

#include <GeographicLib/UTMUPS.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>
#include <rclcpp_components/register_node_macro.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace gnss_ins_localizer
{

static constexpr uint8_t INS_BIT_POSITION = 0x01;
static constexpr uint8_t INS_BIT_VELOCITY = 0x02;
static constexpr uint8_t INS_BIT_ATTITUDE = 0x04;
static constexpr uint8_t INS_BIT_HEADING  = 0x08;
static constexpr uint8_t INS_ALL_ALIGNED  = 0x0F;

static inline double deg2rad(double d) { return d * M_PI / 180.0; }

using namespace std::chrono_literals;

GnssInsLocalizer::GnssInsLocalizer(const rclcpp::NodeOptions & options)
: rclcpp::Node("gnss_ins_localizer", options)
{
  map_path_    = declare_parameter<std::string>("map_path", "");
  map_frame_   = declare_parameter<std::string>("map_frame", "map");
  base_frame_  = declare_parameter<std::string>("base_frame", "base_link");
  gins_frame_  = declare_parameter<std::string>("gins_frame", "gnss_ins_link");
  override_gins_frame_id_ = declare_parameter<bool>("override_gins_frame_id", true);

  roll_sign_               = declare_parameter<int>("roll_sign", 1);
  pitch_sign_              = declare_parameter<int>("pitch_sign", -1);
  swap_roll_pitch_source_  = declare_parameter<bool>("swap_roll_pitch_source", true);

  sigma_xy_full_           = declare_parameter<double>("sigma_xy_full", 0.2);
  sigma_z_full_            = declare_parameter<double>("sigma_z_full", 0.10);
  sigma_yaw_full_deg_      = declare_parameter<double>("sigma_yaw_full_deg", 1.0);
  sigma_xy_degraded_       = declare_parameter<double>("sigma_xy_degraded", 0.5);
  sigma_z_degraded_        = declare_parameter<double>("sigma_z_degraded", 1.0);
  sigma_yaw_degraded_deg_  = declare_parameter<double>("sigma_yaw_degraded_deg", 3.0);
  stale_timeout_sec_       = declare_parameter<double>("stale_timeout_sec", 0.2);
  status_mode_             = declare_parameter<std::string>("status_mode", "positive");
  required_ins_status_mask_ = declare_parameter<int>("required_ins_status_mask", INS_ALL_ALIGNED);
  publish_imu_              = declare_parameter<bool>("publish_imu", false);

  if (status_mode_ != "positive" && status_mode_ != "bitmask") {
    RCLCPP_WARN(
      get_logger(), "Unknown status_mode='%s'; falling back to 'positive'.", status_mode_.c_str());
    status_mode_ = "positive";
  }
  if (required_ins_status_mask_ <= 0 || required_ins_status_mask_ > 255) {
    RCLCPP_WARN(
      get_logger(), "required_ins_status_mask=%d is invalid; using 0x%02X.",
      required_ins_status_mask_, INS_ALL_ALIGNED);
    required_ins_status_mask_ = INS_ALL_ALIGNED;
  }

  std::string projector_yaml = map_path_ + "/map_projector_info.yaml";
  if (!loadMapOrigin(projector_yaml)) {
    RCLCPP_FATAL(get_logger(), "Failed to load map origin from %s", projector_yaml.c_str());
    throw std::runtime_error("map origin load failed");
  }

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  rclcpp::QoS sensor_qos(20);
  sensor_qos.best_effort();
  rclcpp::QoS reliable_qos(10);
  reliable_qos.reliable();
  rclcpp::QoS map_qos(1);
  map_qos.transient_local().reliable();

  sub_gins_ = create_subscription<GnssIns>(
    "~/input/gins", sensor_qos,
    std::bind(&GnssInsLocalizer::onGnssIns, this, std::placeholders::_1));

  sub_map_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "~/input/vector_map", map_qos,
    std::bind(&GnssInsLocalizer::onVectorMap, this, std::placeholders::_1));

  sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/input/goal", reliable_qos,
    std::bind(&GnssInsLocalizer::onGoal, this, std::placeholders::_1));

  sub_route_state_ = create_subscription<autoware_planning_msgs::msg::RouteState>(
    "~/input/route_state", reliable_qos,
    std::bind(&GnssInsLocalizer::onRouteState, this, std::placeholders::_1));

  sub_kinematic_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/kinematic_state", reliable_qos,
    std::bind(&GnssInsLocalizer::onKinematicState, this, std::placeholders::_1));

  pub_imu_    = create_publisher<sensor_msgs::msg::Imu>("~/output/imu", sensor_qos);
  pub_pose_   = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                  "~/output/pose_with_covariance", reliable_qos);
  pub_gnss_pose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                  "~/output/gnss_pose_with_covariance", reliable_qos);
  pub_twist_  = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
                  "~/output/twist_with_covariance", reliable_qos);
  pub_initialpose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                  "~/output/initialpose3d", reliable_qos);
  pub_debug_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(
                  "~/output/debug_pose", reliable_qos);

  gnss_ins_diagnostics_ = std::make_unique<autoware_utils_diagnostics::DiagnosticsInterface>(
    this, "gnss_ins_status");
  diagnostics_timer_ = create_wall_timer(
    100ms, std::bind(&GnssInsLocalizer::publishGnssInsDiagnostics, this));

  srv_initialize_ = create_service<std_srvs::srv::Trigger>(
    "~/initialize",
    std::bind(&GnssInsLocalizer::handleInitialize, this,
              std::placeholders::_1, std::placeholders::_2));

  srv_check_mission_ = create_service<std_srvs::srv::Trigger>(
    "~/check_mission_ready",
    std::bind(&GnssInsLocalizer::handleCheckMissionReady, this,
              std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(),
    "gnss_ins_localizer ready. Origin lat=%.9f lon=%.9f UTM zone=%d %s",
    origin_lat_, origin_lon_, origin_utm_zone_, origin_utm_northp_ ? "N" : "S");
  RCLCPP_INFO(get_logger(),
    "status_mode=%s required_ins_status_mask=0x%02X publish_imu=%s",
    status_mode_.c_str(), required_ins_status_mask_, publish_imu_ ? "true" : "false");
  RCLCPP_INFO(get_logger(),
    "Call 'ros2 service call /gnss_ins_localizer/initialize std_srvs/srv/Trigger' to publish initialpose.");
}

bool GnssInsLocalizer::loadMapOrigin(const std::string & path)
{
  if (!std::filesystem::exists(path)) {
    RCLCPP_ERROR(get_logger(), "Map projector info not found: %s", path.c_str());
    return false;
  }
  try {
    YAML::Node root = YAML::LoadFile(path);
    std::string ptype = root["projector_type"].as<std::string>();
    if (ptype != "LocalCartesianUTM") {
      RCLCPP_WARN(get_logger(),
        "projector_type='%s', code assumes LocalCartesianUTM. Continuing anyway.",
        ptype.c_str());
    }
    origin_lat_ = root["map_origin"]["latitude"].as<double>();
    origin_lon_ = root["map_origin"]["longitude"].as<double>();

    GeographicLib::UTMUPS::Forward(
      origin_lat_, origin_lon_,
      origin_utm_zone_, origin_utm_northp_,
      origin_utm_x_, origin_utm_y_);
    origin_loaded_ = true;
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to parse %s: %s", path.c_str(), e.what());
    return false;
  }
}

void GnssInsLocalizer::llToMap(double lat, double lon, double & mx, double & my) const
{
  int zone; bool northp; double x, y;
  GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, x, y, origin_utm_zone_);
  mx = x - origin_utm_x_;
  my = y - origin_utm_y_;
}

tf2::Quaternion GnssInsLocalizer::ginsAttitudeToMapQuat(
  double roll_deg, double pitch_deg, double yaw_deg) const
{
  double r_in = deg2rad(roll_deg);
  double p_in = deg2rad(pitch_deg);
  double y_in = deg2rad(yaw_deg);
  double roll_flu, pitch_flu;
  if (swap_roll_pitch_source_) {
    roll_flu  = roll_sign_  * p_in;
    pitch_flu = pitch_sign_ * r_in;
  } else {
    roll_flu  = roll_sign_  * r_in;
    pitch_flu = pitch_sign_ * p_in;
  }

  double yaw_enu = M_PI / 2.0 + y_in;
  while (yaw_enu > M_PI) yaw_enu -= 2 * M_PI;
  while (yaw_enu < -M_PI) yaw_enu += 2 * M_PI;

  tf2::Quaternion q;
  q.setRPY(roll_flu, pitch_flu, yaw_enu);
  q.normalize();
  return q;
}

bool GnssInsLocalizer::isPoseUsable(const uint8_t status) const
{
  if (status_mode_ == "positive") {
    // The current GINS2000 driver and mock publisher use status=4 for
    // "combined navigation normal", rather than four independent bits.
    return status != 0;
  }

  constexpr uint8_t pose_mask = INS_BIT_POSITION | INS_BIT_HEADING;
  return (status & pose_mask) == pose_mask;
}

bool GnssInsLocalizer::isFullyAligned(const uint8_t status) const
{
  if (status_mode_ == "positive") {
    return status != 0;
  }

  const auto required_mask = static_cast<uint8_t>(required_ins_status_mask_);
  return (status & required_mask) == required_mask;
}

void GnssInsLocalizer::onGnssIns(const GnssIns::ConstSharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lk(mtx_);
    latest_gins_ = msg;
    last_gins_recv_time_ = now();
  }

  const rclcpp::Time stamp = msg->header.stamp;
  const bool fully_aligned = isFullyAligned(msg->ins_status);
  const bool pose_usable   = isPoseUsable(msg->ins_status);

  {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = override_gins_frame_id_ ? gins_frame_ : msg->header.frame_id;

    auto q = ginsAttitudeToMapQuat(msg->attitude.x, msg->attitude.y, msg->attitude.z);
    imu.orientation.x = q.x();
    imu.orientation.y = q.y();
    imu.orientation.z = q.z();
    imu.orientation.w = q.w();

    imu.angular_velocity.x    =  msg->angular_velocity.y;
    imu.angular_velocity.y    = -msg->angular_velocity.x;
    imu.angular_velocity.z    =  msg->angular_velocity.z;
    imu.linear_acceleration.x =  msg->linear_acceleration.y;
    imu.linear_acceleration.y = -msg->linear_acceleration.x;
    imu.linear_acceleration.z =  msg->linear_acceleration.z;

    const double sxy = fully_aligned ? sigma_xy_full_ : sigma_xy_degraded_;
    const double syaw = deg2rad(fully_aligned ? sigma_yaw_full_deg_ : sigma_yaw_degraded_deg_);
    imu.orientation_covariance[0] = sxy * sxy;
    imu.orientation_covariance[4] = sxy * sxy;
    imu.orientation_covariance[8] = syaw * syaw;
    imu.angular_velocity_covariance[0] = 1e-4;
    imu.angular_velocity_covariance[4] = 1e-4;
    imu.angular_velocity_covariance[8] = 1e-4;
    imu.linear_acceleration_covariance[0] = 1e-2;
    imu.linear_acceleration_covariance[4] = 1e-2;
    imu.linear_acceleration_covariance[8] = 1e-2;

    if (publish_imu_) {
      pub_imu_->publish(imu);
    }
  }

  if (!pose_usable) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "ins_status=0x%02X: position/heading not aligned, pose not published.",
      msg->ins_status);
    return;
  }

  double mx, my;
  llToMap(msg->latitude, msg->longitude, mx, my);

  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = map_frame_;
  pose.pose.pose.position.x = mx;
  pose.pose.pose.position.y = my;
  pose.pose.pose.position.z = msg->altitude;
  auto q = ginsAttitudeToMapQuat(msg->attitude.x, msg->attitude.y, msg->attitude.z);
  pose.pose.pose.orientation.x = q.x();
  pose.pose.pose.orientation.y = q.y();
  pose.pose.pose.orientation.z = q.z();
  pose.pose.pose.orientation.w = q.w();

  const double sxy  = fully_aligned ? sigma_xy_full_  : sigma_xy_degraded_;
  const double sz   = fully_aligned ? sigma_z_full_   : sigma_z_degraded_;
  const double syaw = deg2rad(fully_aligned ? sigma_yaw_full_deg_ : sigma_yaw_degraded_deg_);
  pose.pose.covariance[0]  = sxy * sxy;
  pose.pose.covariance[7]  = sxy * sxy;
  pose.pose.covariance[14] = sz  * sz;
  pose.pose.covariance[21] = 1e-4;
  pose.pose.covariance[28] = 1e-4;
  pose.pose.covariance[35] = syaw * syaw;
  pub_pose_->publish(pose);
  pub_gnss_pose_->publish(pose);

  geometry_msgs::msg::PoseStamped debug_pose;
  debug_pose.header = pose.header;
  debug_pose.pose = pose.pose.pose;
  pub_debug_pose_->publish(debug_pose);

  const double yaw_enu = M_PI / 2.0 + deg2rad(msg->attitude.z);
  const double cy = std::cos(yaw_enu), sy = std::sin(yaw_enu);
  const double vx_body =  cy * msg->velocity.x + sy * msg->velocity.y;
  const double vy_body = -sy * msg->velocity.x + cy * msg->velocity.y;

  geometry_msgs::msg::TwistWithCovarianceStamped twist;
  twist.header.stamp = stamp;
  twist.header.frame_id = base_frame_;
  twist.twist.twist.linear.x  = vx_body;
  twist.twist.twist.linear.y  = vy_body;
  twist.twist.twist.linear.z  = msg->velocity.z;
  twist.twist.twist.angular.x =  msg->angular_velocity.y;
  twist.twist.twist.angular.y = -msg->angular_velocity.x;
  twist.twist.twist.angular.z =  msg->angular_velocity.z;

  const double sv = fully_aligned ? 0.05 : 0.5;
  twist.twist.covariance[0]  = sv * sv;
  twist.twist.covariance[7]  = sv * sv;
  twist.twist.covariance[14] = sv * sv;
  twist.twist.covariance[21] = 1e-4;
  twist.twist.covariance[28] = 1e-4;
  twist.twist.covariance[35] = 1e-3;
  pub_twist_->publish(twist);
}

void GnssInsLocalizer::onVectorMap(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr /*msg*/)
{
  vector_map_received_ = true;
}

void GnssInsLocalizer::onGoal(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  last_goal_ = *msg;
}

void GnssInsLocalizer::onRouteState(
  const autoware_planning_msgs::msg::RouteState::ConstSharedPtr msg)
{
  last_route_state_ = msg->state;
}

void GnssInsLocalizer::onKinematicState(
  const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  kinematic_stamps_.push_back(msg->header.stamp);
  const auto cutoff = now() - rclcpp::Duration::from_seconds(1.0);
  while (!kinematic_stamps_.empty() && kinematic_stamps_.front() < cutoff) {
    kinematic_stamps_.pop_front();
  }
}

void GnssInsLocalizer::publishGnssInsDiagnostics()
{
  GnssIns::ConstSharedPtr snap;
  rclcpp::Time last_recv;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    snap = latest_gins_;
    last_recv = last_gins_recv_time_;
  }

  const rclcpp::Time diagnostic_stamp = now();
  const bool message_received = static_cast<bool>(snap);
  const double age_sec = message_received ? (diagnostic_stamp - last_recv).seconds() : -1.0;
  const bool fresh = message_received && age_sec >= 0.0 && age_sec <= stale_timeout_sec_;

  bool status_ok = false;
  bool finite_values = false;
  bool geographic_range_ok = false;
  bool message_stamp_ok = false;
  uint8_t status = 0;

  if (message_received) {
    status = snap->ins_status;
    status_ok = isFullyAligned(status);
    finite_values =
      std::isfinite(snap->latitude) && std::isfinite(snap->longitude) &&
      std::isfinite(snap->altitude) && std::isfinite(snap->attitude.x) &&
      std::isfinite(snap->attitude.y) && std::isfinite(snap->attitude.z) &&
      std::isfinite(snap->velocity.x) && std::isfinite(snap->velocity.y) &&
      std::isfinite(snap->velocity.z);
    geographic_range_ok =
      snap->latitude >= -90.0 && snap->latitude <= 90.0 &&
      snap->longitude >= -180.0 && snap->longitude <= 180.0;
    message_stamp_ok = snap->header.stamp.sec != 0 || snap->header.stamp.nanosec != 0;
  }

  gnss_ins_diagnostics_->clear();
  gnss_ins_diagnostics_->add_key_value("message_received", message_received);
  gnss_ins_diagnostics_->add_key_value("fresh", fresh);
  gnss_ins_diagnostics_->add_key_value("age_sec", age_sec);
  gnss_ins_diagnostics_->add_key_value("ins_status", static_cast<int>(status));
  gnss_ins_diagnostics_->add_key_value("status_ok", status_ok);
  gnss_ins_diagnostics_->add_key_value("finite_values", finite_values);
  gnss_ins_diagnostics_->add_key_value("geographic_range_ok", geographic_range_ok);
  gnss_ins_diagnostics_->add_key_value("message_stamp_ok", message_stamp_ok);
  gnss_ins_diagnostics_->add_key_value("status_mode", status_mode_);
  gnss_ins_diagnostics_->add_key_value(
    "required_ins_status_mask", required_ins_status_mask_);

  if (!message_received) {
    gnss_ins_diagnostics_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, "no GINS2000 message received");
  } else {
    if (!fresh) {
      gnss_ins_diagnostics_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR, "GINS2000 message is stale");
    }
    if (!status_ok) {
      gnss_ins_diagnostics_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR, "INS status is not ready");
    }
    if (!finite_values) {
      gnss_ins_diagnostics_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR, "GINS2000 contains non-finite values");
    }
    if (!geographic_range_ok) {
      gnss_ins_diagnostics_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR, "latitude/longitude is out of range");
    }
    if (!message_stamp_ok) {
      gnss_ins_diagnostics_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR, "GINS2000 timestamp is zero");
    }
  }

  gnss_ins_diagnostics_->publish(diagnostic_stamp);
}

void GnssInsLocalizer::handleInitialize(
  const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  GnssIns::ConstSharedPtr snap;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    snap = latest_gins_;
  }
  if (!snap) {
    res->success = false;
    res->message = "No GINS message received yet.";
    return;
  }
  if (!isPoseUsable(snap->ins_status)) {
    res->success = false;
    std::ostringstream oss;
    oss << "ins_status=0x" << std::hex << (int)snap->ins_status
        << ": position or heading not aligned. Cannot initialize.";
    res->message = oss.str();
    return;
  }

  double mx, my;
  llToMap(snap->latitude, snap->longitude, mx, my);
  auto q = ginsAttitudeToMapQuat(snap->attitude.x, snap->attitude.y, snap->attitude.z);

  geometry_msgs::msg::PoseWithCovarianceStamped ip;
  ip.header.stamp = now();
  ip.header.frame_id = map_frame_;
  ip.pose.pose.position.x = mx;
  ip.pose.pose.position.y = my;
  ip.pose.pose.position.z = snap->altitude;
  ip.pose.pose.orientation.x = q.x();
  ip.pose.pose.orientation.y = q.y();
  ip.pose.pose.orientation.z = q.z();
  ip.pose.pose.orientation.w = q.w();
  ip.pose.covariance[0] = 0.25;
  ip.pose.covariance[7] = 0.25;
  ip.pose.covariance[14] = 0.25;
  ip.pose.covariance[35] = deg2rad(1.0) * deg2rad(1.0);
  pub_initialpose_->publish(ip);

  initialize_called_ = true;

  double yaw_deg = snap->attitude.z;
  std::ostringstream oss;
  oss << "Initialpose published.\n"
      << "  lat=" << std::fixed << std::setprecision(9) << snap->latitude
      << " lon=" << snap->longitude << "\n"
      << "  map_xy=(" << mx << ", " << my << ") z=" << snap->altitude << "\n"
      << "  yaw_gins(N-CW-to-W,deg)=" << yaw_deg
      << "  yaw_enu(rad)=" << (M_PI / 2.0 + deg2rad(yaw_deg));
  res->success = true;
  res->message = oss.str();
  RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
}

void GnssInsLocalizer::handleCheckMissionReady(
  const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  std::ostringstream rep;
  bool all_ok = true;
  auto tag = [&](bool ok) { return ok ? "[OK]  " : "[FAIL]"; };

  bool ok = vector_map_received_;
  rep << tag(ok) << " vector_map received\n";
  all_ok &= ok;

  bool tf_ok = false; double tf_age = -1;
  try {
    auto tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    tf_age = (now() - rclcpp::Time(tf.header.stamp)).seconds();
    tf_ok = tf_age < 0.5;
  } catch (const std::exception & e) {
    rep << tag(false) << " TF " << map_frame_ << "->" << base_frame_
        << " lookup failed: " << e.what() << "\n";
  }
  if (tf_age >= 0) {
    rep << tag(tf_ok) << " TF " << map_frame_ << "->" << base_frame_
        << " age=" << tf_age << "s\n";
  }
  all_ok &= tf_ok;

  const double hz = kinematic_stamps_.size() / 1.0;
  bool hz_ok = hz >= 10.0;
  rep << tag(hz_ok) << " /localization/kinematic_state ~ " << hz << " Hz\n";
  all_ok &= hz_ok;

  GnssIns::ConstSharedPtr snap;
  rclcpp::Time last_recv;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    snap = latest_gins_;
    last_recv = last_gins_recv_time_;
  }
  if (!snap) {
    rep << tag(false) << " GINS: no message ever received\n";
    all_ok = false;
  } else {
    double age = (now() - last_recv).seconds();
    bool fresh = age < stale_timeout_sec_;
    rep << tag(fresh) << " GINS fresh (age=" << age << "s)\n";
    all_ok &= fresh;

    uint8_t s = snap->ins_status;
    bool s_ok = isFullyAligned(s);
    rep << tag(s_ok) << " ins_status=0x" << std::hex << (int)s << std::dec
        << " {pos=" << ((s & INS_BIT_POSITION) ? 1 : 0)
        << " vel=" << ((s & INS_BIT_VELOCITY) ? 1 : 0)
        << " att=" << ((s & INS_BIT_ATTITUDE) ? 1 : 0)
        << " hdg=" << ((s & INS_BIT_HEADING)  ? 1 : 0) << "}\n";
    all_ok &= s_ok;
  }

  rep << tag(initialize_called_) << " /initialize service was called at least once\n";
  all_ok &= initialize_called_;

  bool goal_ok = last_goal_.has_value();
  rep << tag(goal_ok) << " goal received";
  if (goal_ok) {
    rep << " (frame=" << last_goal_->header.frame_id << ")";
  } else {
    rep << " (click '2D Goal Pose' in RViz)";
  }
  rep << "\n";
  all_ok &= goal_ok;

  if (last_route_state_.has_value()) {
    rep << "[INFO] route_state = " << (int)*last_route_state_ << "\n";
  } else {
    rep << "[INFO] route_state not received yet\n";
  }

  res->success = all_ok;
  res->message = rep.str();
  RCLCPP_INFO(get_logger(), "\n%s", res->message.c_str());
}

}  // namespace gnss_ins_localizer

RCLCPP_COMPONENTS_REGISTER_NODE(gnss_ins_localizer::GnssInsLocalizer)
