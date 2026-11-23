#!/usr/bin/env bash

set -eo pipefail

project_dir=/home/tang/tsy/indoor_localization

source /opt/ros/humble/setup.bash
source /home/tang/livox_ws/install/setup.bash
source "${project_dir}/install/setup.bash"

set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${project_dir}/log/ros}"
mkdir -p "${ROS_LOG_DIR}"

exec ros2 launch room_global_localizer room_global_localizer.launch.py \
  param_file:="${project_dir}/room_global_localizer/config/room_global_localizer.param.yaml"
