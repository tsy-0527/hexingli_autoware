#!/bin/bash
bash record_indoor_route.sh /home/tang/tsy/indoor_route_11201511
tmux kill-session -t autoware
# 1 启动实车
--------------------------------------------------------------------------
# ndt定位
--------------------------------------------------------------------------
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
  
# lidar_camera_fusion 
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
--------------------------------------------------------------------------
# 组合导航定位
--------------------------------------------------------------------------
# 手动触发起点
ros2 service call /api/localization/initialize \
  autoware_adapi_v1_msgs/srv/InitializeLocalization "{}"          

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

--------------------------------------------------------------------------
# 上位机连接9090
--------------------------------------------------------------------------

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
ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0 port:=9090

--------------------------------------------------------------------------
# UDP 传输
--------------------------------------------------------------------------
cd ~/tsy
python3 ros_udp_pose_relay.py --ros-args \
  -p target_port:=15000 \
  -p max_rate_hz:=0.0

未成功：
hostname -I
sudo ufw allow 9090/tcp
sudo ufw allow 9090

--------------------------------------------------------------------------
# 启动相机检测
--------------------------------------------------------------------------
cd /home/tang/autoware/vision_detection_ws
./run_camera_stream.sh


--------------------------------------------------------------------------
# 仿真环境 
--------------------------------------------------------------------------
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

--------------------------------------------------------------------------
# 仿真设置UNKNOWN障碍物
source ~/autoware/install/setup.bash
cd ~/tsy
python3 click_obstacle.py
--------------------------------------------------------------------------
#激光雷达建图结果保存
ros2 service call /map_save std_srvs/srv/Trigger "{}"
查看PCD建图结果：pcl_viewer ~/PCD/名字.pcd
--------------------------------------------------------------------------
# 需要开机自启动，三个终端打开
cd /home/tang/autoware
./scripts/start_autoware_three_terminals.sh
--------------------------------------------------------------------------
# 停掉所有节点
pkill -f ros2
# !rclpy.ok() ROS2 daemon 挂了
# 1. 停掉 daemon
ros2 daemon stop
# 2. 强杀所有 daemon 相关进程
pkill -9 -f _ros2_daemon
pkill -9 -f ros2cli
# 3. 重启 daemon
ros2 daemon start
# 4. 再试
ros2 topic list
--------------------------------------------------------------------------


sudo bash /home/tang/autoware/vision_detection_ws/scripts/enable_camera_frp_tunnels.sh

# 查看检测视图  /vision_detection/debug/image
ros2 run rqt_image_view rqt_image_view 

# lidar_camera_fusion /debug/lidar_camera_overlay
/perception/object_recognition/detection/clustering/camera_lidar_fusion/roi_cluster_fusion/debug/image_raw0
--------------------------------------------------------------------------
# Autoware 的定位输出
ros2 topic echo /localization/kinematic_state --once
--------------------------------------------------------------------------
#查看局部位置
ros2 topic echo /localization/kinematic_state --field pose.pose.position
--------------------------------------------------------------------------
# 设置两个终点循环绕圈  
cd /home/tang/autoware/scripts
python3 loop_checkpoint.py --ros-args   -p max_loops:=8   -p loop_interval:=0.0
--------------------------------------------------------------------------
# 报错 ： 车辆没有停止  
ros2 topic pub -r 10   /sensing/vehicle_velocity_converter/twist_with_covariance   geometry_msgs/msg/TwistWithCovarianceStamped   "{header: auto, twist: {twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}"
--------------------------------------------------------------------------
# 设置最大速度 单位 m/s
ros2 topic pub --once /planning/scenario_planning/max_velocity autoware_internal_planning_msgs/msg/VelocityLimit \
"{stamp: {sec: 0, nanosec: 0}, max_velocity: 1.39, use_constraints: false}"
--------------------------------------------------------------------------
# 查看实时速度
ros2 topic echo /vehicle/status/velocity_status --field longitudinal_velocity
--------------------------------------------------------------------------
# 编译
cd ~/autoware
colcon build --packages-select autoware_launch --symlink-install
source install/setup.bash
--------------------------------------------------------------------------
# 设置途经点 停入车库
cd ~/autoware/scripts
python3 loop_checkpoint.py --ros-args -p num_waypoints:=1
--------------------------------------------------------------------------
# 查看停车框  
python3 ~/tsy/parking_lot_rviz_marker.py   --ros-args   -p osm_path:=/home/tang/autoware_map/map_task4/lanelet2_map.osm
--------------------------------------------------------------------------
#查看can
candump -L can0,121:7FF can1,121:7FF
# 编译
colcon build --packages-select chassis_driver --symlink-install
source install/setup.bash
# 启动底盘
source /opt/ros/humble/setup.bash
source /home/tang/yunle_chassis-main/install/setup.bash
ros2 launch chassis_driver chassis_driver.launch.py
--------------------------------------------------------------------------
# 录制分析精度
cd /home/tang/autoware

./scripts/record_trajectory_localization_perception.sh \
  ~/autoware_bag \
  tracking_test_01
  
~/autoware_bag/tracking_test_01/
~/autoware_bag/plots/
~/autoware_bag/experiment_metadata/tracking_test_01/
--------------------------------------------------------------------------
# 回放rosbag
ros2 bag play ~/autoware_bag/tracking_test_04
ros2 bag info ~/autoware_bag/tracking_test_04 | grep -E "behavior_planning/path|scenario_planning/trajectory"
--------------------------------------------------------------------------
# 实验
ros2 run fj_experiment runner run \
  --name small_obstacle \
  --scenarios S1 \
  --methods M0 M1_N5 M1_N10 M2 M3 \
  --seeds 0 \
  --overwrite
--------------------------------------------------------------------------
起点
attitude:
  x: -0.582275390625
  y: 0.52734375
  z: 40.1220703125
angular_velocity:
  x: -0.00047936899621426287
  y: 0.0
  z: 0.00047936899621426287
linear_acceleration:
  x: 0.10120605468749999
  y: 0.086748046875
  z: 9.898916015625
latitude: 31.9338166
longitude: 118.63309299999999
altitude: 6.889


x: -1.820917821857547
y: -3.8826371092313425
z: -0.4806007989828534

attitude:
  x: -0.933837890625
  y: 0.823974609375
  z: 48.93310546875
angular_velocity:
  x: 0.00023968449810713143
  y: 0.0007190534943213944
  z: 0.0
linear_acceleration:
  x: 0.1156640625
  y: 0.1108447265625
  z: 9.8651806640625
latitude: 31.933830999999998
longitude: 118.63313219999999
altitude: 6.875



x: 43.955233971886265
y: -4.143704256407572
z: -2.3764789719165336

attitude:
  x: 0.15380859375
  y: 0.362548828125
  z: 30.574951171875
angular_velocity:
  x: 0.00023968449810713143
  y: 0.0007190534943213944
  z: 0.0
linear_acceleration:
  x: -0.0385546875
  y: 0.033735351562499995
  z: 9.87
latitude: 31.934146199999997
longitude: 118.63281819999999
altitude: 6.877



x: 78.26959951303796
y: -78.79715761154159
z: -4.386193412795472
attitude:
  x: 0.274658203125
  y: -0.263671875
  z: 44.12109375
angular_velocity:
  x: -0.0007190534943213944
  y: 0.0
  z: 0.0009587379924285257
linear_acceleration:
  x: -0.0819287109375
  y: -0.0433740234375
  z: 9.8651806640625
latitude: 31.9348195
longitude: 118.63317459999999

x: 149.75100736602872
y: -75.2143930547291
z: -7.683990602697111
attitude:
  x: -0.692138671875
  y: 0.0
  z: 44.01123046875
angular_velocity:
  x: 0.0023968449810713144
  y: -0.00023968449810713143
  z: -0.00023968449810713143
linear_acceleration:
  x: 0.07229003906249999
  y: -0.0048193359375
  z: 9.821806640624999
latitude: 31.9352825
longitude: 118.6326488
altitude: 6.814


x: 179.06244719244887
y: -80.34305132241612
z: -9.043812730281783
attitude:
  x: -0.263671875
  y: 0.10986328125
  z: -40.045166015625
angular_velocity:
  x: -0.00023968449810713143
  y: 0.00023968449810713143
  z: -0.00023968449810713143
linear_acceleration:
  x: 0.024096679687499997
  y: 0.0048193359375
  z: 9.860361328124998
latitude: 31.9355105
longitude: 118.63248809999999


x: 29.85972234485006
y: -80.00232842090935
z: -1.9742286090410566
attitude:
  x: -0.02197265625
  y: 0.054931640625
  z: -142.525634765625
angular_velocity:
  x: 0.0007190534943213944
  y: 0.00047936899621426287
  z: 0.0
linear_acceleration:
  x: -0.07229003906249999
  y: 0.0
  z: 9.87
latitude: 31.9344936
longitude: 118.6335251
altitude: 6.854



