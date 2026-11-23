#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash
source /home/tang/livox_ws/install/setup.bash
source /home/tang/gins2000_ws/install/setup.bash
source /home/tang/yunle_chassis-main/install/setup.bash
source /home/tang/autoware/install/setup.bash
source /home/tang/tsy/gnss_localizer_ws/install/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI

# 速度规划器就绪后，把自动驾驶最大速度设置为约 5 km/h。
(
  echo "等待 Autoware 速度规划器启动..."
  until timeout 30 ros2 topic echo \
    /planning/scenario_planning/current_max_velocity \
    --once \
    --qos-durability transient_local \
    >/dev/null 2>&1
  do
    echo "速度规划器尚未就绪，2 秒后继续检查..."
    sleep 2
  done

  ros2 topic pub --once \
    --wait-matching-subscriptions 1 \
    --keep-alive 2 \
    --qos-durability transient_local \
    /planning/scenario_planning/max_velocity_default \
    autoware_internal_planning_msgs/msg/VelocityLimit \
    "{stamp: now, max_velocity: 1.39, use_constraints: false}"
  echo "Autoware 最大速度设置完成：1.39 m/s（约 5 km/h）"
) &

exec ros2 launch autoware_launch autoware.launch.xml \
  map_path:=/home/tang/autoware_map/map_task1 \
  vehicle_model:=yunle_vehicle \
  sensor_model:=livox_sensor_kit \
  pose_source:=gnss-ins \
  twist_source:=gnss-ins \
  gnss_enabled:=true \
  automatic_pose_initializer_enabled:=false \
  use_sim_time:=false \
  system_run_mode:=online

