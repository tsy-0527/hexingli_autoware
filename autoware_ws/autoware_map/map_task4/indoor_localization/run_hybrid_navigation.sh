#!/usr/bin/env bash

set -eo pipefail

project_dir=/home/tang/autoware_map/map_task4/indoor_localization

source /opt/ros/humble/setup.bash
source /home/tang/livox_ws/install/setup.bash
source /home/tang/gins2000_ws/install/setup.bash
source /home/tang/yunle_chassis-main/install/setup.bash
source /home/tang/autoware/install/setup.bash
if [[ -r /home/tang/tsy/gnss_localizer_ws/install/setup.bash ]]; then
  source /home/tang/tsy/gnss_localizer_ws/install/setup.bash
fi
source "${project_dir}/install/setup.bash"

set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${project_dir}/log/ros_hybrid}"
mkdir -p "${ROS_LOG_DIR}"

existing_nodes="$(ros2 node list 2>/dev/null || true)"
if grep -Eq '^(/map/pointcloud_map_loader|/localization/pose_estimator/ndt_scan_matcher|/planner_server|/control/vehicle_cmd_gate)$' <<<"${existing_nodes}"; then
  echo "[错误] 已检测到 Autoware/室内导航节点，禁止重复启动融合栈。" >&2
  grep -E '^(/map/pointcloud_map_loader|/localization/pose_estimator/ndt_scan_matcher|/planner_server|/control/vehicle_cmd_gate)$' <<<"${existing_nodes}" | sort -u >&2
  exit 2
fi

exec ros2 launch indoor_navigation hybrid_navigation.launch.py "$@"

