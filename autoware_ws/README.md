# Autoware 实车、融合感知与调试操作手册

本文档整理自 [`tsy/start_autoware.sh`](tsy/start_autoware.sh)，用于记录本车 Autoware 系统的常用启动、建图、定位、感知、上位机连接、仿真与调试命令。

> [!IMPORTANT]
> 原始文件是命令备忘录，不是可以从头到尾一次执行的脚本。NDT 定位、组合导航定位、仿真等启动方式互相独立，请根据当前场景选择对应章节执行。

## 工作区

```text
autoware_ws/
├── autoware/             # Autoware 主工作区
├── autoware_map/         # 地图
├── fastlio_ws/           # FAST-LIO 建图
├── gins2000_ws/          # GINS2000 驱动与消息
├── hunter_ws/            # Hunter 相关驱动
├── livox_ws/             # Livox 驱动
├── tsy/                  # 辅助脚本、D455 感知和调试工具
└── yunle_chassis-main/   # 云乐底盘驱动
```

以下命令沿用车辆当前的绝对路径 `/home/tang/...`。如果部署目录发生变化，需要同步修改路径。

## 一、实车启动

### 1. NDT 定位

```bash
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
source ~/gins2000_ws/install/setup.bash
source ~/yunle_chassis-main/install/setup.bash
source ~/autoware/install/setup.bash

ros2 launch autoware_launch autoware.launch.xml \
  map_path:=$HOME/autoware_map/map_task4 \
  vehicle_model:=yunle_vehicle \
  sensor_model:=livox_sensor_kit \
  pose_source:=ndt \
  twist_source:=gyro_odom \
  gnss_enabled:=false \
  automatic_pose_initializer_enabled:=false \
  use_sim_time:=false \
  system_run_mode:=online
```

### 2. NDT 定位 + D455 相机与激光雷达融合

```bash
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
source ~/gins2000_ws/install/setup.bash
source ~/yunle_chassis-main/install/setup.bash
source ~/autoware/install/setup.bash
source ~/tsy/D455_ROS2_Humble/install/setup.bash

ros2 launch autoware_launch autoware.launch.xml \
  map_path:=$HOME/autoware_map/map_task4 \
  vehicle_model:=yunle_vehicle \
  sensor_model:=livox_sensor_kit \
  pose_source:=ndt \
  twist_source:=gyro_odom \
  gnss_enabled:=false \
  automatic_pose_initializer_enabled:=false \
  use_sim_time:=false \
  system_run_mode:=online \
  perception_mode:=camera_lidar_fusion \
  launch_camera:=true \
  use_traffic_light_recognition:=false \
  use_custom_d455_yolo:=true \
  custom_d455_yolo_device:=cuda:0 \
  custom_d455_yolo_imgsz:=640 \
  custom_d455_yolo_confidence:=0.35
```

### 3. 组合导航定位（GNSS/INS）

先启动组合导航定位：

```bash
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
source ~/gins2000_ws/install/setup.bash
source ~/yunle_chassis-main/install/setup.bash
source ~/autoware/install/setup.bash
source ~/tsy/gnss_localizer_ws/install/setup.bash

ros2 launch autoware_launch autoware.launch.xml \
  map_path:=$HOME/autoware_map/map_task4 \
  vehicle_model:=yunle_vehicle \
  sensor_model:=livox_sensor_kit \
  pose_source:=gnss-ins \
  twist_source:=gnss-ins \
  gnss_enabled:=true \
  automatic_pose_initializer_enabled:=false \
  use_sim_time:=false \
  system_run_mode:=online
```

系统启动完成后，再手动触发定位初始化：

```bash
ros2 service call /api/localization/initialize \
  autoware_adapi_v1_msgs/srv/InitializeLocalization "{}"
```

## 二、上位机与网络连接

### 1. 启动 Rosbridge WebSocket（9090 端口）

```bash
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
source ~/gins2000_ws/install/setup.bash
source ~/hunter_ws/install/setup.bash
source ~/autoware/install/setup.bash
source ~/yunle_chassis-main/install/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI

ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  address:=0.0.0.0 port:=9090
```

如果上位机无法连接，检查本机地址和防火墙：

```bash
hostname -I
sudo ufw allow 9090/tcp
sudo ufw allow 9090
```

### 2. UDP 位姿传输

```bash
cd ~/tsy
python3 ros_udp_pose_relay.py --ros-args \
  -p target_port:=15000 \
  -p max_rate_hz:=0.0
```

## 三、相机检测与融合结果查看

### 1. 启动相机检测流

```bash
cd /home/tang/autoware/vision_detection_ws
./run_camera_stream.sh
```

### 2. 查看图像

```bash
ros2 run rqt_image_view rqt_image_view
```

常用图像话题：

```text
/vision_detection/debug/image
/perception/object_recognition/detection/clustering/camera_lidar_fusion/roi_cluster_fusion/debug/image_raw0
```

## 四、仿真

### 1. 启动 Planning Simulator

```bash
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
source ~/gins2000_ws/install/setup.bash
source ~/hunter_ws/install/setup.bash
source ~/autoware/install/setup.bash

ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=/home/tang/autoware_map/map_task4 \
  vehicle_model:=yunle_vehicle \
  sensor_model:=livox_sensor_kit \
  localization_sim_mode:=api \
  vehicle_simulation:=true \
  initial_engage_state:=true \
  enable_all_modules_auto_mode:=true \
  use_sim_time:=false
```

### 2. 在仿真中设置 UNKNOWN 障碍物

```bash
source ~/autoware/install/setup.bash
cd ~/tsy
python3 click_obstacle.py
```

## 五、常用操作、建图与调试

### 常用的两个操作

#### 1. 录制轨迹包，用于生成车道

```bash
cd /home/tang/tsy
bash record_indoor_route.sh /home/tang/tsy/indoor_route_11201511
```

该命令录制室内路线轨迹包，输出到 `/home/tang/tsy/indoor_route_11201511`，后续用于生成或调整车道。

#### 2. 清理开机自启动的 Autoware 会话

```bash
tmux kill-session -t autoware
```

该命令结束开机自启动创建的 `autoware` tmux 会话，从而停止会话内运行的 Autoware 进程。它不会删除开机自启动配置，仅清理当前会话。

### 建图与地图查看

#### 1. 保存激光雷达建图结果

```bash
ros2 service call /map_save std_srvs/srv/Trigger "{}"
```

#### 2. 查看 PCD 地图

```bash
pcl_viewer ~/PCD/地图文件名.pcd
```

### 进程与 ROS 2 Daemon 管理

#### 1. 启动三个 Autoware 终端

```bash
cd /home/tang/autoware
./scripts/start_autoware_three_terminals.sh
```

#### 2. 停止所有 ROS 2 节点

> [!CAUTION]
> 以下命令会终止当前用户的 ROS 2 进程，请确认没有其他任务正在运行。

```bash
pkill -f ros2
```

#### 3. 修复 `!rclpy.ok()` 或 ROS 2 Daemon 异常

```bash
ros2 daemon stop
pkill -9 -f _ros2_daemon
pkill -9 -f ros2cli
ros2 daemon start
ros2 topic list
```

### 车辆运行与调试

#### 1. 查看 Autoware 定位输出

```bash
ros2 topic echo /localization/kinematic_state --once
```

仅查看局部位置：

```bash
ros2 topic echo /localization/kinematic_state --field pose.pose.position
```

#### 2. 设置两个终点循环绕圈

```bash
cd /home/tang/autoware/scripts
python3 loop_checkpoint.py --ros-args \
  -p max_loops:=8 \
  -p loop_interval:=0.0
```

#### 3. 处理“车辆没有停止”报错

```bash
ros2 topic pub -r 10 \
  /sensing/vehicle_velocity_converter/twist_with_covariance \
  geometry_msgs/msg/TwistWithCovarianceStamped \
  "{header: auto, twist: {twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}"
```

#### 4. 设置最大速度

单位为 m/s；下面的 `1.39` 约等于 5 km/h。

```bash
ros2 topic pub --once \
  /planning/scenario_planning/max_velocity \
  autoware_internal_planning_msgs/msg/VelocityLimit \
  "{stamp: {sec: 0, nanosec: 0}, max_velocity: 1.39, use_constraints: false}"
```

#### 5. 查看实时速度

```bash
ros2 topic echo /vehicle/status/velocity_status \
  --field longitudinal_velocity
```

#### 6. 设置途经点并停入车库

```bash
cd ~/autoware/scripts
python3 loop_checkpoint.py --ros-args -p num_waypoints:=1
```

#### 7. 在 RViz 中查看停车框

```bash
python3 ~/tsy/parking_lot_rviz_marker.py --ros-args \
  -p osm_path:=/home/tang/autoware_map/map_task4/lanelet2_map.osm
```

### 底盘与 CAN 调试

#### 1. 查看 CAN 报文

```bash
candump -L can0,121:7FF can1,121:7FF
```

#### 2. 编译底盘驱动

请在包含 `chassis_driver` 的工作区执行：

```bash
colcon build --packages-select chassis_driver --symlink-install
source install/setup.bash
```

#### 3. 启动底盘

```bash
source /opt/ros/humble/setup.bash
source /home/tang/yunle_chassis-main/install/setup.bash
ros2 launch chassis_driver chassis_driver.launch.py
```

### 编译 Autoware Launch

```bash
cd ~/autoware
colcon build --packages-select autoware_launch --symlink-install
source install/setup.bash
```

### 轨迹精度数据录制与回放

#### 1. 录制轨迹、定位与感知数据

```bash
cd /home/tang/autoware

./scripts/record_trajectory_localization_perception.sh \
  ~/autoware_bag \
  tracking_test_01
```

相关输出目录：

```text
~/autoware_bag/tracking_test_01/
~/autoware_bag/plots/
~/autoware_bag/experiment_metadata/tracking_test_01/
```

#### 2. 回放和检查 ROS bag

```bash
ros2 bag play ~/autoware_bag/tracking_test_04

ros2 bag info ~/autoware_bag/tracking_test_04 | \
  grep -E "behavior_planning/path|scenario_planning/trajectory"
```

### 安全提示

- 实车启动前确认车辆周围安全，并准备好急停。
- 不要同时启动多套定位或车辆控制节点。
- `pkill` 和防火墙命令会影响系统中其他进程或网络服务。
- 修改速度、循环终点和途经点后，应先在仿真或封闭场地验证。
