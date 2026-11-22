# 核星力车辆 Agent 部署说明

## 1. 部署边界

Agent 主程序和 DeepSeek Key 放在 Windows 上位机中。工控机只运行 Autoware、底盘节点和 rosbridge，不保存大模型 Key。

调用链：

```text
上位机 React → Electron 主进程 → Python MCP → ws://工控机:9090 → Autoware AD API
```

## 2. Windows 上位机准备

当前 MCP 服务要求 Python 3.10 或以上，推荐安装 Python 3.11 x64。安装后执行：

```powershell
cd "C:\Users\HONOR\Documents\核星力小车监控系统\agent\ros_agent_mcp"
py -3.11 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

把 DeepSeek Key 写入当前 Windows 用户环境变量，不要写入 `App.jsx`、`public/config.json` 或 Git：

```powershell
setx DEEPSEEK_API_KEY "你的 DeepSeek Key"
setx DEEPSEEK_MODEL "deepseek-chat"
```

执行 `setx` 后必须完全退出并重新打开上位机。若账户支持的模型名不同，用控制台实际模型名替换 `deepseek-chat`。

如果没有把 Python 安装到系统 PATH，可额外设置：

```powershell
setx VEHICLE_AGENT_PYTHON "C:\完整路径\python.exe"
```

## 3. 配置站点

编辑：

```text
agent/ros_agent_mcp/config.json
```

先保持：

```json
"stations_configured": false
```

在现有上位机地图上设置一次真实终点，然后在工控机读取：

```bash
ros2 topic echo /planning/mission_planning/goal --once
```

把 `position.x`、`position.y` 写入站点配置。航向角计算为：

```text
yaw = 2 × atan2(orientation.z, orientation.w)
```

所有站点核验完成后才能改为：

```json
"stations_configured": true
```

开发环境读取项目内的 `agent/ros_agent_mcp/config.json`。安装版第一次打开 Agent 时会把模板复制到：

```text
%APPDATA%\核星力小车监控系统\vehicle-agent\config.user.json
```

安装版请编辑这份 `config.user.json`。也可以用 `VEHICLE_AGENT_CONFIG` 环境变量明确指定另一份配置文件的绝对路径。

建议同时设置 `map_bounds`，防止误填超出地图的坐标：

```json
"map_bounds": {
  "x_min": -100.0,
  "x_max": 300.0,
  "y_min": -100.0,
  "y_max": 300.0
}
```

## 4. 工控机安装 rosbridge

以下命令在 Ubuntu/ROS 2 工控机执行。将 `$ROS_DISTRO` 替换为实际发行版，例如 `humble`：

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-rosbridge-suite
```

启动 rosbridge 的终端必须先 source Autoware 和自定义消息工作空间，否则 `chassis_interfaces/msg/CcuStatus` 等消息无法解析：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source /path/to/autoware/install/setup.bash
source /path/to/chassis_ws/install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 address:=0.0.0.0
```

如果当前 rosbridge 版本不接受命令行的 `address` 参数，去掉该参数，使用 launch 文件默认监听地址。

## 5. 工控机必须存在的接口

执行：

```bash
ros2 topic type /api/operation_mode/state
ros2 topic type /api/localization/initialization_state
ros2 topic type /api/routing/state
ros2 topic type /localization/kinematic_state
ros2 topic type /yunle_chassis/feedback/ccu_status

ros2 service type /api/localization/initialize
ros2 service type /api/operation_mode/change_to_autonomous
ros2 service type /api/operation_mode/change_to_stop
ros2 service type /api/routing/clear_route
```

预期类型：

```text
/api/operation_mode/state                     autoware_adapi_v1_msgs/msg/OperationModeState
/api/localization/initialization_state        autoware_adapi_v1_msgs/msg/LocalizationInitializationState
/api/routing/state                            autoware_adapi_v1_msgs/msg/RouteState
/localization/kinematic_state                 nav_msgs/msg/Odometry
/yunle_chassis/feedback/ccu_status            chassis_interfaces/msg/CcuStatus
/api/localization/initialize                  autoware_adapi_v1_msgs/srv/InitializeLocalization
/api/operation_mode/change_to_autonomous      autoware_adapi_v1_msgs/srv/ChangeOperationMode
/api/operation_mode/change_to_stop            autoware_adapi_v1_msgs/srv/ChangeOperationMode
/api/routing/clear_route                      autoware_adapi_v1_msgs/srv/ClearRoute
```

若 `/yunle_chassis/feedback/ccu_status` 不存在，不建议直接把配置改成 `null` 后上实车。应优先让 CAN 网关或底盘驱动把制动联锁状态同步为该 ROS 话题，使 Agent 启动自动驾驶前能检查急停和其他制动请求。

配置里的 `brake_interlock_fields` 决定哪些底盘字段算“制动请求”。默认只包含 `emergency_brake_request_status`、`remote_brake_request_status`、`touch_brake_request_status`、`handle_brake_request_status`，不含 `ccu_parking_status`：自动驾驶中途停车时驻车位会自然置位，若把它算作制动请求，重新派车会被 `CHASSIS_BRAKE` / `AUTONOMY_NOT_READY` 拦截。需要恢复原行为时把五个字段全部填入。

**建议启用 `/diagnostics`**（可选，但强烈建议）：

```bash
ros2 topic type /diagnostics     # 预期 diagnostic_msgs/msg/DiagnosticArray
```

Agent 的 `get_diagnostics` 工具只读这个聚合话题，用来回答“为什么不能启动自动驾驶”“哪个模块在报错”。没有它也能正常派车，只是失败时只能看到状态层面的原因（定位/路线/模式），看不到模块级报错。若话题不存在，工具会返回 `available: false` 并说明原因，不会影响其他功能。

## 6. 配置 systemd 自动启动

创建 `/etc/systemd/system/hexingli-rosbridge.service`：

```ini
[Unit]
Description=Hexingli rosbridge websocket
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=autoware
ExecStart=/bin/bash -lc 'source /opt/ros/humble/setup.bash && source /path/to/autoware/install/setup.bash && source /path/to/chassis_ws/install/setup.bash && exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 address:=0.0.0.0'
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
```

把 `User`、ROS 发行版和工作空间路径改成现场值，然后执行：

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now hexingli-rosbridge
sudo systemctl status hexingli-rosbridge
journalctl -u hexingli-rosbridge -f
```

## 7. 网络与防火墙

只允许上位机或 WireGuard/ZeroTier 网段访问 9090，不要把 rosbridge 直接暴露到公网。

示例：只允许一台上位机访问：

```bash
sudo ufw allow from <上位机IP> to any port 9090 proto tcp
sudo ufw deny 9090/tcp
```

如果两台机器通过 `10.66.0.0/24` 连接：

```bash
sudo ufw allow from 10.66.0.0/24 to any port 9090 proto tcp
```

工控机检查监听端口：

```bash
ss -lntp | grep 9090
```

上位机检查：

```powershell
Test-NetConnection 10.66.0.2 -Port 9090
```

## 8. 启动顺序

1. 启动底盘驱动和 CAN 网关。
2. 启动 Autoware，确认 AD API 话题和服务存在。
3. 启动 rosbridge。
4. 启动 Windows 上位机。
5. 在“智能调度”中先执行“查询车辆状态”。
6. 在仿真中验证定位、目的地、停车和人工确认。
7. 最后在封闭场地进行实车测试，现场人员必须守在物理急停旁。

`stop_vehicle` 是 Autoware 的平稳停车，不是物理急停。网络或软件异常时必须使用现有上位机停车、底盘制动或物理急停。
