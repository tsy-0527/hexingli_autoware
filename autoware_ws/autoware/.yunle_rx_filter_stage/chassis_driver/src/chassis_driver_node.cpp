#include "chassis_driver/chassis_driver_node.hpp"

#include "chassis_driver/control_command_bridge.hpp"
#include "chassis_driver/dbc_protocol.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <sstream>
#include <vector>

namespace chassis_driver
{

namespace
{
constexpr double kSteeringAngleCodeAtMaxAngle = 120.0;

/** Parse k:v parameter entries into hash map. */
/** 将 k:v 形式的参数条目解析为哈希表。 */
std::unordered_map<std::string, std::string> parseMap(const std::vector<std::string> & entries)
{
  std::unordered_map<std::string, std::string> out;
  for (const auto & e : entries) {
    const auto pos = e.find(':');
    if (pos == std::string::npos) {
      continue;
    }
    out[e.substr(0, pos)] = e.substr(pos + 1);
  }
  return out;
}

const char * boolText(bool value)
{
  return value ? "true" : "false";
}

std::string joinSet(const std::set<std::string> & values)
{
  if (values.empty()) {
    return "(none)";
  }
  std::ostringstream ss;
  bool first = true;
  for (const auto & value : values) {
    if (!first) {
      ss << ", ";
    }
    ss << value;
    first = false;
  }
  return ss.str();
}

std::string formatMap(const std::unordered_map<std::string, std::string> & values)
{
  if (values.empty()) {
    return "(none)";
  }

  std::vector<std::string> keys;
  keys.reserve(values.size());
  for (const auto & item : values) {
    keys.push_back(item.first);
  }
  std::sort(keys.begin(), keys.end());

  std::ostringstream ss;
  bool first = true;
  for (const auto & key : keys) {
    if (!first) {
      ss << ", ";
    }
    ss << key << "->" << values.at(key);
    first = false;
  }
  return ss.str();
}

/** Convert internal CAN frame to ROS message with timestamp. */
/** 将内部 CAN 帧转换为带时间戳的 ROS 消息。 */
chassis_interfaces::msg::CanFrame toRosFrame(const CanFrame & frame, const rclcpp::Time & stamp)
{
  chassis_interfaces::msg::CanFrame msg;
  msg.stamp = stamp;
  msg.can_id = frame.can_id;
  msg.is_extended = frame.is_extended;
  msg.is_remote = frame.is_remote;
  msg.dlc = frame.dlc;
  msg.data = frame.data;
  msg.channel = frame.channel;
  return msg;
}

/** Format a CAN frame as a compact hexadecimal string for operator logs. */
/** 将 CAN 帧格式化为紧凑的十六进制字符串用于操作日志。 */
std::string formatCanFrameHex(
  const CanFrame & frame, uint8_t channel, const std::string & message_name)
{
  std::ostringstream ss;
  ss << message_name << " can" << static_cast<int>(channel) << " id=0x" << std::uppercase
     << std::hex << frame.can_id << " dlc=" << std::dec << static_cast<int>(frame.dlc) << " data=[";
  const auto dlc = std::min<uint8_t>(frame.dlc, static_cast<uint8_t>(8U));
  for (uint8_t i = 0; i < dlc; ++i) {
    if (i > 0) {
      ss << ' ';
    }
    ss << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
       << static_cast<int>(frame.data[i]);
  }
  ss << "]";
  return ss.str();
}

/** Convert protocol steering-angle code to physical wheel angle in degrees. */
/** 将协议转角编码值换算为实际车轮转角，单位为度。 */
double steeringAngleCodeToDeg(double code, double max_steering_angle_deg)
{
  if (!std::isfinite(code)) {
    return 0.0;
  }
  return code / kSteeringAngleCodeAtMaxAngle * max_steering_angle_deg;
}
}  // namespace

/** Build node, setup publishers/subscribers, SocketCAN channels and RX threads. */
/** 构建节点，创建发布器/订阅器、SocketCAN 通道和接收线程。 */
ChassisDriverNode::ChassisDriverNode() : Node("chassis_driver_node")
{
  loadParameters();
  logRuntimeConfiguration();

  auto qos = rclcpp::QoS(rclcpp::KeepLast(default_qos_depth_));
  if (isPublishTopicEnabled("can_rx_raw")) {
    raw_rx_pub_ =
      create_publisher<chassis_interfaces::msg::CanFrame>(makeTopicName("can_rx/raw"), qos);
  }
  if (isPublishTopicEnabled("can_tx_raw")) {
    raw_tx_pub_ =
      create_publisher<chassis_interfaces::msg::CanFrame>(makeTopicName("can_tx/raw"), qos);
  }
  if (isPublishTopicEnabled("debug_unknown_frames")) {
    unknown_frame_pub_ =
      create_publisher<std_msgs::msg::String>(makeTopicName("debug/unknown_frames"), qos);
  }

  if (isPublishTopicEnabled("feedback_bms_status")) {
    bms_status_pub_ = create_publisher<chassis_interfaces::msg::BmsStatus>(
      makeTopicName("feedback/bms_status"), qos);
  }
  if (isPublishTopicEnabled("feedback_vcu_warning_level")) {
    vcu_warning_pub_ = create_publisher<chassis_interfaces::msg::VcuWarningLevel>(
      makeTopicName("feedback/vcu_warning_level"), qos);
  }
  if (isPublishTopicEnabled("feedback_wheel_speed")) {
    wheel_speed_pub_ = create_publisher<chassis_interfaces::msg::WheelSpeedFeedback>(
      makeTopicName("feedback/wheel_speed"), qos);
  }
  if (isPublishTopicEnabled("feedback_ccu_status")) {
    ccu_status_pub_ = create_publisher<chassis_interfaces::msg::CcuStatus>(
      makeTopicName("feedback/ccu_status"), qos);
  }
  if (isPublishTopicEnabled("feedback_sas_angle")) {
    sas_angle_pub_ = create_publisher<chassis_interfaces::msg::SasAngleFeedback>(
      makeTopicName("feedback/sas_angle"), qos);
  }
  if (isPublishTopicEnabled("feedback_target_speed_feedback")) {
    target_speed_pub_ = create_publisher<chassis_interfaces::msg::ScuTargetSpeedFeedback>(
      makeTopicName("feedback/target_speed_feedback"), qos);
  }

  initializeChannels();

  frame_router_ = std::make_shared<FrameRouter>(*this);
  control_bridge_ = std::make_shared<ControlCommandBridge>(*this);

  scu_control_timer_ = create_wall_timer(
    std::chrono::milliseconds(scu_control_period_ms_),
    std::bind(&ChassisDriverNode::sendScuControlTimerCallback, this));

  startThreads();
  RCLCPP_INFO(
    get_logger(),
    "chassis_driver_node is running successfully. / chassis_driver_node 节点已成功运行。");
}

/** Ensure threads are stopped before destruction. */
/** 析构前确保线程已经停止。 */
ChassisDriverNode::~ChassisDriverNode()
{
  if (scu_control_timer_) {
    scu_control_timer_->cancel();
  }
  stopThreads();
}

/** Declare/read ROS parameters and build message-channel lookup tables. */
/** 声明/读取 ROS 参数并构建报文-通道查找表。 */
void ChassisDriverNode::loadParameters()
{
  declare_parameter<std::string>("topic_prefix", "/yunle_chassis");
  declare_parameter<bool>("publish_raw_can", true);
  declare_parameter<bool>("publish_unknown_frames", true);
  declare_parameter<bool>("enable_debug_topics", true);
  declare_parameter<bool>("log_control_can_frames", false);
  declare_parameter<int>("default_qos_depth", 10);
  declare_parameter<std::vector<std::string>>(
    "enabled_publish_topics", std::vector<std::string>{"all"});
  declare_parameter<std::vector<std::string>>(
    "enabled_subscribe_topics", std::vector<std::string>{"all"});
  declare_parameter<std::vector<std::string>>("message_channel_map", std::vector<std::string>{});
  declare_parameter<std::vector<std::string>>(
    "control_message_channel_map", std::vector<std::string>{});
  declare_parameter<std::string>("can1_interface", "can1");
  declare_parameter<std::string>("can2_interface", "can0");
  declare_parameter<int>("socketcan_receive_timeout_ms", 100);
  declare_parameter<int>("scu_control_period_ms", 10);
  declare_parameter<int>("scu_control_command_timeout_ms", 200);
  declare_parameter<double>("scu_control_max_steering_angle_deg", 27.0);
  declare_parameter<double>("scu_control_max_target_speed_kmh", 15.0);

  get_parameter("topic_prefix", topic_prefix_);
  if (topic_prefix_.empty()) {
    topic_prefix_ = "/yunle_chassis";
  }
  if (topic_prefix_.front() != '/') {
    topic_prefix_ = "/" + topic_prefix_;
  }
  while (topic_prefix_.size() > 1 && topic_prefix_.back() == '/') {
    topic_prefix_.pop_back();
  }

  get_parameter("publish_raw_can", publish_raw_can_);
  get_parameter("publish_unknown_frames", publish_unknown_frames_);
  get_parameter("enable_debug_topics", enable_debug_topics_);
  get_parameter("log_control_can_frames", log_control_can_frames_);
  get_parameter("default_qos_depth", default_qos_depth_);

  std::vector<std::string> enabled_publish_topics;
  std::vector<std::string> enabled_subscribe_topics;
  get_parameter("enabled_publish_topics", enabled_publish_topics);
  get_parameter("enabled_subscribe_topics", enabled_subscribe_topics);
  enabled_publish_topics_ =
    std::set<std::string>(enabled_publish_topics.begin(), enabled_publish_topics.end());
  enabled_subscribe_topics_ =
    std::set<std::string>(enabled_subscribe_topics.begin(), enabled_subscribe_topics.end());

  std::vector<std::string> map_entries;
  std::vector<std::string> control_map_entries;
  get_parameter("message_channel_map", map_entries);
  get_parameter("control_message_channel_map", control_map_entries);
  feedback_channel_map_ = parseMap(map_entries);
  control_channel_map_ = parseMap(control_map_entries);

  get_parameter("can1_interface", can1_interface_);
  get_parameter("can2_interface", can2_interface_);
  get_parameter("socketcan_receive_timeout_ms", socketcan_receive_timeout_ms_);
  get_parameter("scu_control_period_ms", scu_control_period_ms_);
  get_parameter("scu_control_command_timeout_ms", scu_control_command_timeout_ms_);
  get_parameter("scu_control_max_steering_angle_deg", scu_control_max_steering_angle_deg_);
  get_parameter("scu_control_max_target_speed_kmh", scu_control_max_target_speed_kmh_);

  if (can1_interface_.empty() || can2_interface_.empty()) {
    throw std::runtime_error("SocketCAN interface names must not be empty");
  }
  if (can1_interface_ == can2_interface_) {
    throw std::runtime_error("CAN1 and CAN2 must use different SocketCAN interfaces");
  }
  if (socketcan_receive_timeout_ms_ <= 0) {
    socketcan_receive_timeout_ms_ = 100;
  }
  if (scu_control_period_ms_ <= 0) {
    scu_control_period_ms_ = 10;
  }
  if (scu_control_command_timeout_ms_ < scu_control_period_ms_) {
    scu_control_command_timeout_ms_ = 200;
  }

  if (
    !std::isfinite(scu_control_max_steering_angle_deg_) ||
    scu_control_max_steering_angle_deg_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(), "Invalid scu_control_max_steering_angle_deg %.3f, fallback to 27.0",
      scu_control_max_steering_angle_deg_);
    scu_control_max_steering_angle_deg_ = 27.0;
  }
  if (
    !std::isfinite(scu_control_max_target_speed_kmh_) || scu_control_max_target_speed_kmh_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(), "Invalid scu_control_max_target_speed_kmh %.3f, fallback to 15.0",
      scu_control_max_target_speed_kmh_);
    scu_control_max_target_speed_kmh_ = 15.0;
  }
  for (const auto & required :
       {"SCU_Control_Command", "SCU_Chassis_Command", "SCU_Torque_Command", "VCU_Debug_Enable",
        "VCU_Drive_Debug"}) {
    if (control_channel_map_.find(required) == control_channel_map_.end()) {
      RCLCPP_FATAL(get_logger(), "Missing required TX mapping: %s", required);
      throw std::runtime_error("missing tx mapping");
    }
  }
}

/** Log effective parameters so operators can verify runtime wiring. */
/** 输出实际生效参数，便于运行人员核对当前接线和配置。 */
void ChassisDriverNode::logRuntimeConfiguration() const
{
  RCLCPP_INFO(
    get_logger(), "========== chassis_driver_node runtime configuration / 当前运行配置 ==========");
  RCLCPP_INFO(
    get_logger(), "ROS topics / ROS 话题: topic_prefix=%s, default_qos_depth=%d",
    topic_prefix_.c_str(), default_qos_depth_);
  RCLCPP_INFO(
    get_logger(),
    "Topic switches / 话题开关: publish_raw_can=%s, publish_unknown_frames=%s, "
    "enable_debug_topics=%s, log_control_can_frames=%s",
    boolText(publish_raw_can_), boolText(publish_unknown_frames_), boolText(enable_debug_topics_),
    boolText(log_control_can_frames_));
  RCLCPP_INFO(
    get_logger(), "Enabled publishers / 已启用发布项: %s",
    joinSet(enabled_publish_topics_).c_str());
  RCLCPP_INFO(
    get_logger(), "Enabled subscribers / 已启用订阅项: %s",
    joinSet(enabled_subscribe_topics_).c_str());
  RCLCPP_INFO(get_logger(), "SocketCAN chassis CAN1: interface=%s", can1_interface_.c_str());
  RCLCPP_INFO(get_logger(), "SocketCAN chassis CAN2: interface=%s", can2_interface_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "SocketCAN/control timing: receive_timeout_ms=%d, 0x121 period_ms=%d, command_timeout_ms=%d",
    socketcan_receive_timeout_ms_, scu_control_period_ms_, scu_control_command_timeout_ms_);
  RCLCPP_INFO(
    get_logger(),
    "Control limits / 控制限值: scu_control_max_steering_angle_deg=%.3f, "
    "scu_control_max_target_speed_kmh=%.3f",
    scu_control_max_steering_angle_deg_, scu_control_max_target_speed_kmh_);
  RCLCPP_INFO(
    get_logger(), "Feedback channel map / 反馈报文通道映射: %s",
    formatMap(feedback_channel_map_).c_str());
  RCLCPP_INFO(
    get_logger(), "Control channel map / 控制报文通道映射: %s",
    formatMap(control_channel_map_).c_str());
}

/** Open chassis CAN1/CAN2 through their configured Linux SocketCAN interfaces. */
/** 通过配置的 Linux SocketCAN 接口打开底盘 CAN1/CAN2。 */
void ChassisDriverNode::initializeChannels()
{
  if (!can1_.open(can1_interface_, socketcan_receive_timeout_ms_)) {
    RCLCPP_FATAL(
      get_logger(), "Failed to open chassis CAN1 SocketCAN interface '%s'",
      can1_interface_.c_str());
    throw std::runtime_error("chassis CAN1 SocketCAN open failed");
  }
  if (!can2_.open(can2_interface_, socketcan_receive_timeout_ms_)) {
    RCLCPP_FATAL(
      get_logger(), "Failed to open chassis CAN2 SocketCAN interface '%s'",
      can2_interface_.c_str());
    throw std::runtime_error("chassis CAN2 SocketCAN open failed");
  }
}

/** Start background RX loops for both channels. */
/** 启动两个通道的后台接收循环。 */
void ChassisDriverNode::startThreads()
{
  running_.store(true);
  can1_rx_thread_ = std::thread([this]() { rxLoop(1); });
  can2_rx_thread_ = std::thread([this]() { rxLoop(2); });
}

/** Stop RX loops and join all worker threads. */
/** 停止接收循环并等待所有工作线程退出。 */
void ChassisDriverNode::stopThreads()
{
  running_.store(false);
  can1_.close();
  can2_.close();
  if (can1_rx_thread_.joinable()) can1_rx_thread_.join();
  if (can2_rx_thread_.joinable()) can2_rx_thread_.join();
}

/** Channel receive loop: receive one native CAN frame and route it. */
/** 通道接收循环：接收一帧原生 CAN 报文并路由。 */
void ChassisDriverNode::rxLoop(uint8_t channel_id)
{
  auto & channel = channel_id == 1 ? can1_ : can2_;
  while (running_.load()) {
    CanFrame frame;
    if (!channel.receive(frame)) {
      continue;
    }
    frame.channel = channel_id;
    publishRawRx(frame);
    frame_router_->routeFrame(frame);
  }
}

/** Encode and send control frame over resolved channel and publish TX raw topic. */
/** 通过解析出的通道编码并发送控制帧，同时发布 TX 原始话题。 */
void ChassisDriverNode::sendControlFrame(const CanFrame & frame, const std::string & message_name)
{
  std::scoped_lock<std::mutex> lock(tx_mutex_);
  const uint8_t channel = resolveChannel(message_name, true);
  const bool ok = (channel == 1) ? can1_.send(frame) : can2_.send(frame);
  if (!ok) {
    RCLCPP_ERROR(get_logger(), "Failed to send frame %s on can%u", message_name.c_str(), channel);
    return;
  }
  CanFrame tx = frame;
  tx.channel = channel;
  if (log_control_can_frames_) {
    const auto formatted = formatCanFrameHex(tx, channel, message_name);
    RCLCPP_INFO(get_logger(), "TX control CAN: %s", formatted.c_str());
  }
  publishRawTx(tx);
}

/** Cache the latest 0x121 command; the timer owns actual CAN transmission. */
void ChassisDriverNode::updateScuControlFrame(const CanFrame & frame)
{
  std::scoped_lock<std::mutex> lock(scu_command_mutex_);
  latest_scu_control_frame_ = frame;
  last_scu_command_time_ = std::chrono::steady_clock::now();
}

/** Transmit 0x121 every 10 ms while the upstream ROS command remains fresh. */
void ChassisDriverNode::sendScuControlTimerCallback()
{
  std::optional<CanFrame> frame;
  {
    std::scoped_lock<std::mutex> lock(scu_command_mutex_);
    if (!latest_scu_control_frame_) {
      return;
    }

    const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - last_scu_command_time_);
    if (age.count() > scu_control_command_timeout_ms_) {
      RCLCPP_WARN(
        get_logger(),
        "SCU command input timed out after %ld ms; stop sending 0x121 so VCU fail-safe can engage",
        static_cast<long>(age.count()));
      latest_scu_control_frame_.reset();
      return;
    }
    frame = latest_scu_control_frame_;
  }

  sendControlFrame(*frame, "SCU_Control_Command");
}

/** Decode known CAN IDs and publish typed feedback topics. */
/** 解析已知 CAN ID 并发布强类型反馈话题。 */
void ChassisDriverNode::publishDecoded(const CanFrame & frame)
{
  // A number of chassis frames are mirrored on both physical CAN buses.  Keep
  // publishing both copies on the raw diagnostic topic, but only publish the
  // typed feedback from the logical bus selected by message_channel_map.
  const auto definition = DbcProtocol::messageById().find(frame.can_id);
  if (definition != DbcProtocol::messageById().end()) {
    const auto route = feedback_channel_map_.find(definition->second.name);
    if (
      route != feedback_channel_map_.end() &&
      resolveChannel(definition->second.name, false) != frame.channel) {
      return;
    }
  }

  const rclcpp::Time stamp = now();
  auto get = [&](const std::string & name) {
    return DbcProtocol::decodeSignal(frame, name).value_or(0.0);
  };

  switch (frame.can_id) {
    case 256U: {
      chassis_interfaces::msg::BmsStatus msg;
      msg.stamp = stamp;
      msg.bms_voltage = static_cast<float>(get("BMS_Voltage"));
      msg.bms_current = static_cast<float>(get("BMS_Current"));
      msg.bms_soc = static_cast<float>(get("BMS_SOC"));
      if (bms_status_pub_) {
        bms_status_pub_->publish(msg);
      }
      break;
    }
    case 119U: {
      chassis_interfaces::msg::VcuWarningLevel msg;
      msg.stamp = stamp;
      msg.bms_soc_warning = static_cast<uint8_t>(get("BMS_SOC_Warning"));
      msg.mcu_disconnect_warning = static_cast<uint8_t>(get("MCU_Disconnect_Warning"));
      msg.mcu_motor_warning = static_cast<uint8_t>(get("MCU_Motor_Warning"));
      msg.mcu_speed_warning = static_cast<uint8_t>(get("MCU_Speed_Warning"));
      msg.steering_disconnect_warning = static_cast<uint8_t>(get("Steering_Disconnect_Warning"));
      msg.steering_lock_warning = static_cast<uint8_t>(get("Steering_Lock_Warning"));
      msg.steering_uncontrollable_warning =
        static_cast<uint8_t>(get("Steering_Uncontrollable_Warning"));
      msg.steering_error_warning = static_cast<uint8_t>(get("Steering_Error_Warning"));
      msg.brake_error_warning = static_cast<uint8_t>(get("Brake_Error_Warning"));
      if (vcu_warning_pub_) {
        vcu_warning_pub_->publish(msg);
      }
      break;
    }
    case 360U: {
      chassis_interfaces::msg::WheelSpeedFeedback msg;
      msg.stamp = stamp;
      msg.wheel_speed_front_left_rpm = static_cast<float>(get("Wheel_Speed_Front_Left_RPM"));
      msg.wheel_speed_front_right_rpm = static_cast<float>(get("Wheel_Speed_Front_Right_RPM"));
      msg.wheel_speed_rear_left_rpm = static_cast<float>(get("Wheel_Speed_Rear_Left_RPM"));
      msg.wheel_speed_rear_right_rpm = static_cast<float>(get("Wheel_Speed_Rear_Right_RPM"));
      if (wheel_speed_pub_) {
        wheel_speed_pub_->publish(msg);
      }
      break;
    }
    case 81U: {
      chassis_interfaces::msg::CcuStatus msg;
      msg.stamp = stamp;
      msg.ccu_shift_level_status = static_cast<uint8_t>(get("CCU_Shift_Level_Status"));
      msg.ccu_parking_status = get("CCU_Parking_Status") > 0.5;
      msg.ccu_ignition_status = static_cast<uint8_t>(get("CCU_Ignition_Status"));
      msg.ccu_drive_mode_shift_button = get("CCU_Drive_Mode_Shift_Button") > 0.5;
      msg.steering_wheel_direction = get("Steering_Wheel_Direction") > 0.5;
      const double ccu_steering_code = get("CCU_Steering_Wheel_Angle");
      const double ccu_steering_angle =
        steeringAngleCodeToDeg(ccu_steering_code, scu_control_max_steering_angle_deg_);
      msg.ccu_steering_wheel_angle =
        static_cast<float>(msg.steering_wheel_direction ? ccu_steering_angle : -ccu_steering_angle);
      msg.ccu_vehicle_speed = static_cast<float>(get("CCU_Vehicle_Speed"));
      msg.ccu_drive_mode = static_cast<uint8_t>(get("CCU_Drive_Mode"));
      msg.remote_brake_request_status = get("Remote_Brake_Request_Status") > 0.5;
      msg.emergency_brake_request_status = get("Emergency_Brake_Request_Status") > 0.5;
      msg.scu_brake_signal_status = get("SCU_Brake_Signal_Status") > 0.5;
      msg.touch_brake_request_status = get("Touch_Brake_Request_Status") > 0.5;
      msg.handle_brake_request_status = get("Handle_Brake_Request_Status") > 0.5;
      msg.handle_mode_flag_status = get("Handle_Mode_Flag_Status") > 0.5;
      msg.left_turn_light_status = get("Left_Turn_Light_Status") > 0.5;
      msg.right_turn_light_status = get("Right_Turn_Light_Status") > 0.5;
      msg.position_light_status = get("Position_Light_Status") > 0.5;
      msg.low_beam_status = get("Low_Beam_Status") > 0.5;
      if (ccu_status_pub_) {
        ccu_status_pub_->publish(msg);
      }
      break;
    }
    case 225U: {
      chassis_interfaces::msg::SasAngleFeedback msg;
      msg.stamp = stamp;
      msg.sas_front_angle = static_cast<float>(
        steeringAngleCodeToDeg(get("SAS_Front_Angle"), scu_control_max_steering_angle_deg_));
      msg.sas_rear_angle = static_cast<float>(
        steeringAngleCodeToDeg(get("SAS_Rear_Angle"), scu_control_max_steering_angle_deg_));
      if (sas_angle_pub_) {
        sas_angle_pub_->publish(msg);
      }
      break;
    }
    case 2033U: {
      chassis_interfaces::msg::ScuTargetSpeedFeedback msg;
      msg.stamp = stamp;
      msg.hardware_target_speed = static_cast<float>(get("Hardware_Target_Speed"));
      msg.scu_target_speed_feedback = static_cast<float>(get("SCU_Target_Speed_Feedback"));
      msg.vehicle_target_speed = static_cast<float>(get("Vehicle_Target_Speed"));
      msg.vehicle_target_speed_rpm = static_cast<float>(get("Vehicle_Target_Speed_RPM"));
      if (target_speed_pub_) {
        target_speed_pub_->publish(msg);
      }
      break;
    }
    default:
      publishUnknownFrame(frame);
      break;
  }
}

/** Publish raw RX frame when enabled. */
/** 在启用时发布原始 RX 帧。 */
void ChassisDriverNode::publishRawRx(const CanFrame & frame)
{
  if (!publish_raw_can_) return;
  if (raw_rx_pub_) {
    raw_rx_pub_->publish(toRosFrame(frame, now()));
  }
}

/** Publish raw TX frame when enabled. */
/** 在启用时发布原始 TX 帧。 */
void ChassisDriverNode::publishRawTx(const CanFrame & frame)
{
  if (!publish_raw_can_) return;
  if (raw_tx_pub_) {
    raw_tx_pub_->publish(toRosFrame(frame, now()));
  }
}

/** Publish unknown-frame debug message when enabled. */
/** 在启用时发布未知帧调试消息。 */
void ChassisDriverNode::publishUnknownFrame(const CanFrame & frame)
{
  if (!publish_unknown_frames_ || !enable_debug_topics_) return;
  std_msgs::msg::String msg;
  std::stringstream ss;
  ss << "Unknown frame: id=" << frame.can_id << " ext=" << frame.is_extended
     << " ch=" << static_cast<int>(frame.channel);
  msg.data = ss.str();
  if (unknown_frame_pub_) {
    unknown_frame_pub_->publish(msg);
  }
}

/** Build topic name by joining configured prefix and suffix. */
/** 通过拼接配置前缀和后缀构造话题名。 */
std::string ChassisDriverNode::makeTopicName(const std::string & suffix) const
{
  if (suffix.empty()) {
    return topic_prefix_;
  }
  if (suffix.front() == '/') {
    return topic_prefix_ + suffix;
  }
  return topic_prefix_ + "/" + suffix;
}

/** Check whether publisher topic key is enabled by parameter list. */
/** 检查发布话题 key 是否被参数列表启用。 */
bool ChassisDriverNode::isPublishTopicEnabled(const std::string & topic_key) const
{
  return enabled_publish_topics_.count("all") > 0 || enabled_publish_topics_.count(topic_key) > 0;
}

/** Check whether subscriber topic key is enabled by parameter list. */
/** 检查订阅话题 key 是否被参数列表启用。 */
bool ChassisDriverNode::isSubscribeTopicEnabled(const std::string & topic_key) const
{
  return enabled_subscribe_topics_.count("all") > 0 ||
         enabled_subscribe_topics_.count(topic_key) > 0;
}

/** Resolve channel ID from configured map; throw when mapping is missing. */
/** 从配置映射中解析通道 ID；缺少映射时抛出异常。 */
uint8_t ChassisDriverNode::resolveChannel(const std::string & message_name, bool tx) const
{
  const auto & map = tx ? control_channel_map_ : feedback_channel_map_;
  const auto it = map.find(message_name);
  if (it == map.end()) {
    throw std::runtime_error("missing channel mapping for " + message_name);
  }
  return (it->second == "can2") ? 2 : 1;
}

}  // namespace chassis_driver
