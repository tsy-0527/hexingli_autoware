cd ~/ros2_ws
source install/setup.bash
ros2 launch orbbec_camera gemini_330_series.launch.py #发布深度相机的话题，订阅/camera/color/image_raw或/camera/depth/image_raw查看RGB图像和深度图像

#发布GINS2000的话题/gins2000
cd ~/gins2000_ws
source install/setup.bash
ros2 launch gins2000 gins2000.launch.py 

#发布雷达的话题/livox/imu和/livox/lidar
cd ~/livox_ws
source install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py 

# 启动命令
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
source ~/autoware/install/setup.bash

ros2 launch autoware_launch autoware.launch.xml \
  map_path:=$HOME/autoware_map/map_test \
  vehicle_model:=sample_vehicle \
  sensor_model:=livox_sensor_kit \
  launch_planning:=false \
  launch_control:=false
