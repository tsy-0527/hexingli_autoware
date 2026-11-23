#!/usr/bin/env bash

set -eo pipefail

project_dir=/home/tang/autoware_map/map_task4/indoor_localization
map_path="${MAP_PATH:-${project_dir}/map_20261117_denoised}"

if [[ ! -f "${map_path}/pointcloud_map.pcd" ]]; then
  echo "[错误] 找不到点云地图: ${map_path}/pointcloud_map.pcd" >&2
  exit 2
fi

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

# A second copy would create duplicate map, NDT, EKF and Nav2 node names.
# Refuse it explicitly and tell the operator which stack is still running.
existing_nodes="$(ros2 node list 2>/dev/null || true)"
if grep -Eq '^(/map/pointcloud_map_loader|/localization/pose_estimator/ndt_scan_matcher|/room_global_localizer|/planner_server)$' <<<"${existing_nodes}"; then
  echo "[错误] 已检测到室内定位/规划节点，不能重复启动。" >&2
  grep -E '^(/map/pointcloud_map_loader|/localization/pose_estimator/ndt_scan_matcher|/room_global_localizer|/planner_server)$' <<<"${existing_nodes}" | sort -u >&2
  echo "请先在原启动终端按 Ctrl-C，确认节点退出后再运行本脚本。" >&2
  exit 2
fi

echo "[地图] ${map_path}/pointcloud_map.pcd"
exec ros2 launch indoor_localization task5_combined_localization.launch.py \
  map_path:="${map_path}" "$@"
