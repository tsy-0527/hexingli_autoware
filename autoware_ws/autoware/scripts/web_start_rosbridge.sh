#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash
source /home/tang/livox_ws/install/setup.bash
source /home/tang/gins2000_ws/install/setup.bash
source /home/tang/hunter_ws/install/setup.bash
source /home/tang/autoware/install/setup.bash
source /home/tang/yunle_chassis-main/install/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI

exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  address:=0.0.0.0 \
  port:=9090

