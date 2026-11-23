# 云乐线控底盘 ROS2 驱动

项目归属：skywilling。

本仓库提供一个 ROS2 C++ 底盘驱动，通过 Linux SocketCAN 和 USB-CAN 与云乐线控底盘通信。驱动负责接收底盘反馈 CAN 报文并发布为 ROS2 话题，同时订阅 ROS2 控制话题、封装为 CAN 报文并从指定的 `can0/can1` 接口发送到底盘。

当前代码中的 CAN 协议映射与仓库根目录的 `Yunle_CAN_release.dbc` 对齐。运行时不解析 DBC 文件，DBC 信号定义已经固化在 `chassis_driver/src/dbc_protocol.cpp` 中。

## 适用环境

- ROS2 Humble 风格工程
- C++17
- `ament_cmake`
- Linux SocketCAN 环境
- 已由内核驱动识别为 `can0/can1` 的 USB-CAN 或板载 CAN

## 工程结构

```text
yunle_chassis/
  chassis_interfaces/          自定义 ROS2 消息包
    msg/                       底盘反馈、控制指令和原始 CAN 帧消息
  chassis_driver/              底盘驱动包
    include/chassis_driver/    驱动节点、SocketCAN、CAN 编解码、DBC 协议声明
    src/                       驱动节点、控制封装、反馈解析、键盘控制实现
    config/                    统一参数配置文件
    launch/                    驱动节点和键盘控制节点启动文件
  docs/                        协议说明、ROS2 话题说明和用户文档
  Yunle_CAN_release.dbc        当前参考 DBC 文件
```

## 功能概览

驱动节点 `chassis_driver_node` 提供以下功能：

- 打开底盘 CAN1/CAN2 对应的两路 Linux SocketCAN 接口。
- 直接接收 USB-CAN/板载 CAN 提供的原生 CAN 帧。
- 按 `Yunle_CAN_release.dbc` 中的信号定义解析反馈报文。
- 发布电池、故障等级、轮速、CCU 状态、SAS 转角和目标速度反馈话题。
- 订阅 SCU 控制、底盘制动/转向速率、扭矩和 VCU 调试控制话题。
- 将控制话题封装为 CAN 报文发送到底盘。
- 可选发布原始 RX/TX CAN 帧和未知 CAN ID 调试信息。
- 可选输出已发送控制 CAN 帧的十六进制日志。

辅助节点 `keyboard_scu_control_node` 提供键盘控制能力，用于低速联调或人工检查 `/yunle_chassis/control/scu_control_command` 话题。

## 编译

在 ROS2 环境中执行：

```bash
colcon build --symlink-install --packages-select chassis_interfaces chassis_driver
source install/setup.bash
```

如果工作区中还有其他包，也可以只执行：

```bash
colcon build --symlink-install
source install/setup.bash
```

## 启动底盘驱动

默认配置文件为 `chassis_driver/config/chassis_driver.yaml`。

```bash
ros2 launch chassis_driver chassis_driver.launch.py
```

启动后节点名称为：

```text
chassis_driver_node
```

## 配置文件说明

配置文件路径：

```text
chassis_driver/config/chassis_driver.yaml
```

主要参数如下。

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `topic_prefix` | `/yunle_chassis` | 所有发布和订阅话题的统一前缀。 |
| `publish_raw_can` | `true` | 是否发布原始接收和发送 CAN 帧。 |
| `publish_unknown_frames` | `true` | 是否发布无法识别的 CAN ID 调试信息。 |
| `enable_debug_topics` | `true` | 是否启用调试类话题。 |
| `log_control_can_frames` | `false` | 是否在日志中输出成功发送到底盘的控制 CAN 报文十六进制内容。 |
| `default_qos_depth` | `10` | 发布器和订阅器默认 QoS 队列深度。 |
| `enabled_publish_topics` | `["all"]` | 发布话题开关。可使用 `all` 或列出指定 key。 |
| `enabled_subscribe_topics` | `["all"]` | 订阅话题开关。可使用 `all` 或列出指定 key。 |
| `message_channel_map` | 见 YAML | 反馈 CAN 消息名到 CAN1/CAN2 的通道映射。 |
| `control_message_channel_map` | 见 YAML | 控制 CAN 消息名到 CAN1/CAN2 的发送通道映射。 |
| `can1_interface` | `can1` | 底盘逻辑 CAN1 对应的 Linux SocketCAN 接口。 |
| `can2_interface` | `can0` | 底盘逻辑 CAN2 对应的 Linux SocketCAN 接口；`0x121` 从这里发送。 |
| `socketcan_receive_timeout_ms` | `100` | SocketCAN 接收超时，用于节点安全退出。 |
| `scu_control_period_ms` | `10` | 最新有效 `0x121` 的周期发送间隔。 |
| `scu_control_command_timeout_ms` | `200` | ROS 控制输入超时；超时后停止重复旧指令。 |
| `scu_control_max_steering_angle_deg` | `27.0` | 底盘最大物理转角；用于控制指令转角编码，也用于 SAS/CCU 转角反馈编码值换算。 |
| `scu_control_max_target_speed_kmh` | `15.0` | `/control/scu_control_command` 中目标速度最大允许值，单位 km/h。 |

### 话题开关 key

可用于 `enabled_publish_topics` 的 key：

- `can_rx_raw`
- `can_tx_raw`
- `debug_unknown_frames`
- `feedback_bms_status`
- `feedback_vcu_warning_level`
- `feedback_wheel_speed`
- `feedback_ccu_status`
- `feedback_sas_angle`
- `feedback_target_speed_feedback`

可用于 `enabled_subscribe_topics` 的 key：

- `control_scu_control_command`
- `control_scu_chassis_command`
- `control_scu_torque_command`
- `control_vcu_chassis_debug`

## SocketCAN接口

驱动使用 Linux `PF_CAN/SOCK_RAW/CAN_RAW` 直接读写经典 CAN 帧，不再需要 IP 地址、UDP 端口或以太网转 CAN 网关。接口波特率必须在启动节点前配置，例如 `sudo ip link set can0 type can bitrate 500000 restart-ms 100`。

`SCU_Control_Command (0x121)` 会由驱动每 10 ms 重复发送最新控制值。当 ROS 控制输入超过配置的超时时间没有更新时，驱动停止发送旧指令，让 VCU 自身的 500 ms 超时保护生效。

## 发布话题

默认前缀为 `/yunle_chassis`。

| 话题 | 消息类型 | 来源 CAN ID | 说明 |
|---|---|---:|---|
| `/yunle_chassis/feedback/bms_status` | `chassis_interfaces/msg/BmsStatus` | `0x100` | 电池电压、电流、SOC。 |
| `/yunle_chassis/feedback/vcu_warning_level` | `chassis_interfaces/msg/VcuWarningLevel` | `0x077` | VCU 汇总故障等级。 |
| `/yunle_chassis/feedback/wheel_speed` | `chassis_interfaces/msg/WheelSpeedFeedback` | `0x168` | 四轮转速反馈。 |
| `/yunle_chassis/feedback/ccu_status` | `chassis_interfaces/msg/CcuStatus` | `0x051` | 档位、驻车、点火、车速、灯光、制动、模式状态和已换算为实际角度的方向盘转角。 |
| `/yunle_chassis/feedback/sas_angle` | `chassis_interfaces/msg/SasAngleFeedback` | `0x0E1` | 已按最大转角换算为实际角度的前/后转角传感器反馈。 |
| `/yunle_chassis/feedback/target_speed_feedback` | `chassis_interfaces/msg/ScuTargetSpeedFeedback` | `0x7F1` | 目标速度相关反馈。 |
| `/yunle_chassis/can_rx/raw` | `chassis_interfaces/msg/CanFrame` | 全部接收帧 | 原始接收 CAN 帧，受 `publish_raw_can` 和话题开关控制。 |
| `/yunle_chassis/can_tx/raw` | `chassis_interfaces/msg/CanFrame` | 全部发送帧 | 原始发送 CAN 帧，受 `publish_raw_can` 和话题开关控制。 |
| `/yunle_chassis/debug/unknown_frames` | `std_msgs/msg/String` | 未映射 CAN ID | 未识别帧信息，受 `publish_unknown_frames`、`enable_debug_topics` 和话题开关控制。 |

## 订阅话题

默认前缀为 `/yunle_chassis`。

| 话题 | 消息类型 | 发送 CAN ID | 说明 |
|---|---|---:|---|
| `/yunle_chassis/control/scu_control_command` | `chassis_interfaces/msg/ScuControlCommand` | `0x121` | 主要自动驾驶控制报文，包含档位、前后转角、目标速度、制动、灯光和有效位。 |
| `/yunle_chassis/control/scu_chassis_command` | `chassis_interfaces/msg/ScuChassisCommand` | `0x126` | 转向角速度和四轮制动力控制。 |
| `/yunle_chassis/control/scu_torque_command` | `chassis_interfaces/msg/ScuTorqueCommand` | `0x123` | 四轮扭矩控制。 |
| `/yunle_chassis/control/vcu_chassis_debug` | `chassis_interfaces/msg/VcuChassisDebug` | `0x710`、`0x715` | 一个 ROS2 话题同时封装 VCU 调试使能和速度环 PID 参数两条 CAN 报文。 |

## `/yunle_chassis/control/scu_control_command` 使用规则

`ScuControlCommand.msg` 不包含 `scu_drive_mode_request` 输入。只要收到该话题，驱动下发的 `SCU_Drive_Mode_Request` 信号固定为 `1`。

底盘从遥控模式切换到自动驾驶模式需要同时满足：自动驾驶切换按钮闭合、底盘 CAN2 持续收到有效的 `0x121`，且其中 `SCU_Drive_Mode_Request=1`。按钮闭合本身只表示允许接管，不会单独改变驾驶模式。停止更新 ROS 控制话题后，驱动先在输入超时时停止发送 `0x121`，底盘随后按自身 500 ms 通信超时策略退回手动/遥控控制。

字段说明：

| 字段 | 说明 |
|---|---|
| `scu_shift_level_request` | 档位请求。只接受 `1=D`、`2=N`、`3=R`，其他值会丢弃整帧并输出警告。 |
| `scu_steering_angle_front` | 前轮物理转角，单位 deg，允许范围为 `±scu_control_max_steering_angle_deg`。 |
| `scu_steering_angle_rear` | 后轮物理转角，单位 deg，允许范围为 `±scu_control_max_steering_angle_deg`。 |
| `scu_target_speed` | 目标速度绝对值，单位 km/h。方向由档位决定，只接受 `[0, scu_control_max_target_speed_kmh]`。 |
| `scu_brake_enable` | 制动使能。 |
| `gw_left_turn_light_request` | 左转向灯请求。 |
| `gw_right_turn_light_request` | 右转向灯请求。 |
| `gw_position_light_request` | 位置灯请求。 |
| `gw_low_beam_request` | 近光灯请求。 |
| `scu_torque_or_speed_mode` | 扭矩/速度模式选择。 |
| `steering_angle_speed_valid` | 转角/角速度控制有效位。 |
| `brake_force_command_valid` | 制动力控制有效位。 |

转角或速度超出配置允许范围时，驱动不会丢弃整帧，而是将对应信号下发为 `0` 并输出警告。

## 控制示例

### 自动驾驶 D 档低速前进

```bash
ros2 topic pub --once /yunle_chassis/control/scu_control_command chassis_interfaces/msg/ScuControlCommand "{
  scu_shift_level_request: 1,
  scu_steering_angle_front: 0.0,
  scu_steering_angle_rear: 0.0,
  scu_target_speed: 3.0,
  scu_brake_enable: false,
  gw_left_turn_light_request: 0,
  gw_right_turn_light_request: 0,
  gw_position_light_request: 0,
  gw_low_beam_request: 0,
  scu_torque_or_speed_mode: 0,
  steering_angle_speed_valid: true,
  brake_force_command_valid: true
}"
```

### R 档低速后退

```bash
ros2 topic pub --once /yunle_chassis/control/scu_control_command chassis_interfaces/msg/ScuControlCommand "{
  scu_shift_level_request: 3,
  scu_steering_angle_front: 0.0,
  scu_steering_angle_rear: 0.0,
  scu_target_speed: 2.0,
  scu_brake_enable: false,
  gw_left_turn_light_request: 0,
  gw_right_turn_light_request: 0,
  gw_position_light_request: 0,
  gw_low_beam_request: 0,
  scu_torque_or_speed_mode: 0,
  steering_angle_speed_valid: true,
  brake_force_command_valid: true
}"
```

### 停车制动

```bash
ros2 topic pub --once /yunle_chassis/control/scu_control_command chassis_interfaces/msg/ScuControlCommand "{
  scu_shift_level_request: 1,
  scu_steering_angle_front: 0.0,
  scu_steering_angle_rear: 0.0,
  scu_target_speed: 0.0,
  scu_brake_enable: true,
  gw_left_turn_light_request: 0,
  gw_right_turn_light_request: 0,
  gw_position_light_request: 0,
  gw_low_beam_request: 0,
  scu_torque_or_speed_mode: 0,
  steering_angle_speed_valid: true,
  brake_force_command_valid: true
}"
```

### 底盘制动力/转向角速度控制

> **警示：该控制会直接影响底盘制动力和转向执行响应，可能导致车辆突然制动、转向或姿态变化。仅允许在专业人士指导下，在车辆架空、低风险测试场地或已确认急停链路有效的环境中操作。**

```bash
ros2 topic pub --once /yunle_chassis/control/scu_chassis_command chassis_interfaces/msg/ScuChassisCommand "{
  vcu_target_steering_angle_speed: 180.0,
  brake_force_front_left: 20.0,
  brake_force_front_right: 20.0,
  brake_force_rear_left: 10.0,
  brake_force_rear_right: 10.0
}"
```

### 四轮扭矩控制

> **警示：四轮扭矩控制会直接作用于驱动轮输出，错误参数可能导致车辆突然加速、打滑或失控。仅允许在专业人士指导下，并确认车辆安全固定、急停和制动链路有效后操作。**

```bash
ros2 topic pub --once /yunle_chassis/control/scu_torque_command chassis_interfaces/msg/ScuTorqueCommand "{
  torque_command_front_left: 100.0,
  torque_command_front_right: 100.0,
  torque_command_rear_left: 80.0,
  torque_command_rear_right: 80.0
}"
```

### VCU 调试控制

> **警示：VCU 调试控制会修改或使能速度环 PID 调试相关参数，可能改变车辆控制响应和稳定性。该操作仅供专业调试人员使用，必须在专业人士指导下，在安全测试环境中执行。**

```bash
ros2 topic pub --once /yunle_chassis/control/vcu_chassis_debug chassis_interfaces/msg/VcuChassisDebug "{
  pid_debug_enable: true,
  velocity_kp: 10.0,
  velocity_ki: 0.5,
  velocity_kd: 0.2
}"
```

## 键盘控制节点

键盘节点发布 `/yunle_chassis/control/scu_control_command`，适合在可控环境下做低速人工联调。

推荐直接在有键盘焦点的前台终端运行：

```bash
ros2 run chassis_driver keyboard_scu_control_node
```

交互式键盘控制应直接使用 `ros2 run` 在有键盘焦点的前台终端运行。

节点启动后的默认控制消息为 N 档、零速度、零转角、制动关闭、灯光关闭，并且 `scu_torque_or_speed_mode=0`、`steering_angle_speed_valid=false`、`brake_force_command_valid=false`。

常用按键：

| 按键 | 功能 |
|---|---|
| `w` | 增加目标速度，不改变当前档位。 |
| `s` | 降低目标速度，不改变当前档位。 |
| `1` / `2` / `3` | 选择 D / N / R 档。 |
| `q` / `e` | 降低 / 提高目标速度。 |
| `a` / `d` | 左转 / 右转。 |
| `c` | 转角回正。 |
| `x` | N 档零速命令。 |
| `Space` | N 档制动。 |
| `b` | 切换制动使能。 |
| `j` / `k` | 切换左 / 右转向灯请求。 |
| `u` / `i` | 切换位置灯 / 近光灯请求。 |
| `h` | 打印中英文帮助。 |
| `Ctrl-C` | 退出。 |

键盘节点参数可通过 `ros2 run` 的 `--ros-args -p` 方式配置，例如：

```bash
ros2 run chassis_driver keyboard_scu_control_node --ros-args -p publish_rate_hz:=20.0 -p max_speed_kmh:=15.0
```

可用参数如下：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `topic_name` | `/yunle_chassis/control/scu_control_command` | 发布目标话题。 |
| `publish_rate_hz` | `20.0` | 键盘控制命令发布频率。 |
| `speed_step_kmh` | `0.5` | 每次按键速度变化量。 |
| `default_speed_kmh` | `1.0` | 目标速度为 0 时首次按 `w` 使用的默认速度。 |
| `max_speed_kmh` | `15.0` | 键盘节点侧速度上限。 |
| `steering_step_deg` | `2.0` | 每次按键转角变化量。 |
| `max_steering_angle_deg` | `27.0` | 键盘节点侧转角上限。 |
| `rear_steering_ratio` | `0.0` | 后轮转角相对前轮转角的比例。 |
| `auto_publish_zero_on_exit` | `true` | 正常退出时是否发布一次零速/制动命令。 |

## 常用检查命令

查看话题：

```bash
ros2 topic list
```

查看某个反馈话题：

```bash
ros2 topic echo /yunle_chassis/feedback/ccu_status
```

查看原始接收 CAN 帧：

```bash
ros2 topic echo /yunle_chassis/can_rx/raw
```

查看原始发送 CAN 帧：

```bash
ros2 topic echo /yunle_chassis/can_tx/raw
```

临时开启控制 CAN 报文十六进制日志，需要修改配置：

```yaml
log_control_can_frames: true
```

重新启动驱动后，只要有节点发布控制话题并且发送成功，日志会输出对应 CAN ID、DLC 和 data 字节。

## 当前安全边界

`0x121` 使用独立的 10 ms 定时器重复发送最新控制值。若 ROS 控制话题超过 `scu_control_command_timeout_ms` 未更新，驱动停止发送旧指令，由 VCU 的 500 ms 通信超时保护退出自动控制。该机制不能替代车辆急停、制动链路和现场安全员。

当前代码中发送链路使用互斥锁保护 SocketCAN 发送；接收链路使用 CAN1/CAN2 两个后台线程分别读取原生 CAN 帧并发布 ROS2 反馈。

部分底盘反馈会同时镜像到 CAN1 和 CAN2。原始接收话题保留两路报文用于诊断；强类型反馈话题按照 `message_channel_map` 只发布指定逻辑总线上的一份，避免上层收到重复状态。

## 故障排查

- 启动时报 SocketCAN 打开失败：确认配置的接口存在、已设置 500 kbit/s 并处于 `UP/ERROR-ACTIVE`。
- 没有反馈话题数据：检查 `can1_interface`/`can2_interface` 映射、CAN_H/CAN_L、波特率和 `enabled_publish_topics`。
- 控制话题无发送：检查 `enabled_subscribe_topics`、`control_message_channel_map`、CAN2接口映射以及是否有节点持续发布控制话题。
- `scu_control_command` 被丢弃：检查 `scu_shift_level_request` 是否为 `1`、`2` 或 `3`。
- 控制速度或转角被置零：检查数值是否超过 `scu_control_max_target_speed_kmh` 或 `scu_control_max_steering_angle_deg`。
