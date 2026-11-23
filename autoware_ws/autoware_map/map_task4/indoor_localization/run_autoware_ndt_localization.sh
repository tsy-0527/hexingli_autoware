#!/usr/bin/env bash

set -eo pipefail

project_dir=/home/tang/autoware_map/map_task4/indoor_localization

source /opt/ros/humble/setup.bash
source /home/tang/livox_ws/install/setup.bash
source /home/tang/gins2000_ws/install/setup.bash
source /home/tang/yunle_chassis-main/install/setup.bash
source /home/tang/autoware/install/setup.bash
source "${project_dir}/install/setup.bash"

set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${project_dir}/log/ros}"
mkdir -p "${ROS_LOG_DIR}"

exec ros2 launch room_global_localizer autoware_ndt_room_localization.launch.py
