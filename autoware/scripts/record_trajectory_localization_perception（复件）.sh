#!/usr/bin/env bash

# 录制规划轨迹、实际定位轨迹，以及定位/感知诊断数据。
# 录制结束后自动绘制横向误差、航向误差和行驶轨迹图。
#
# 用法：
#   ./scripts/record_trajectory_localization_perception.sh
#   ./scripts/record_trajectory_localization_perception.sh <录包根目录> <录包名称>
#   ./scripts/record_trajectory_localization_perception.sh --tracking-only
#   ./scripts/record_trajectory_localization_perception.sh --raw-lidar
#   ./scripts/record_trajectory_localization_perception.sh <录包根目录> <录包名称> [--tracking-only|--raw-lidar]
#
# 默认录包输出：
#   ~/autoware_bag/trajectory_eval_YYYYMMDD_HHMMSS
# 默认图片输出：
#   ~/autoware_bag/plots/trajectory_eval_YYYYMMDD_HHMMSS_*.png
#
# 计算横向/航向误差最重要的话题：
#   /planning/trajectory 或 /planning/scenario_planning/trajectory
#   /localization/pose_estimator/pose_with_covariance（优先，通常约 100 Hz）
#   /localization/kinematic_state（高频定位话题不存在时回退，通常约 20 Hz）
#
# 区分控制器、命令门和底盘执行误差的话题：
#   /control/trajectory_follower/control_cmd
#   /control/command/control_cmd
#   /vehicle/status/velocity_status
#   /vehicle/status/steering_status
#
# 脚本只录制当前 ROS graph 中已经存在的话题。不存在的话题会提示并跳过，
# 因此可兼容不同的传感器和定位配置。

# ROS 2 setup scripts may reference variables that are not defined yet, so
# enable nounset only after all workspaces have been sourced.
set -Ee -o pipefail

source /opt/ros/humble/setup.bash
for workspace_setup in \
  /home/tang/autoware/install/setup.bash \
  /home/tang/hunter_ws/install/setup.bash \
  /home/tang/gins2000_ws/install/setup.bash \
  /home/tang/livox_ws/install/setup.bash; do
  if [[ -f "$workspace_setup" ]]; then
    # shellcheck disable=SC1090
    source "$workspace_setup"
  fi
done

set -u

bag_root="${HOME}/autoware_bag"
bag_name="trajectory_eval_$(date +%Y%m%d_%H%M%S)"
with_raw_lidar=0
tracking_only=0
positional_args=()

for argument in "$@"; do
  if [[ "$argument" == "--raw-lidar" ]]; then
    with_raw_lidar=1
  elif [[ "$argument" == "--tracking-only" ]]; then
    tracking_only=1
  else
    positional_args+=("$argument")
  fi
done

if ((${#positional_args[@]} >= 1)); then
  bag_root="${positional_args[0]}"
fi
if ((${#positional_args[@]} >= 2)); then
  bag_name="${positional_args[1]}"
fi
if ((${#positional_args[@]} > 2)); then
  echo "用法：$0 [录包根目录] [录包名称] [--tracking-only|--raw-lidar]" >&2
  exit 2
fi
if ((tracking_only && with_raw_lidar)); then
  echo "错误：--tracking-only 与 --raw-lidar 不能同时使用。" >&2
  exit 2
fi

bag_path="${bag_root}/${bag_name}"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
plot_script="${script_dir}/plot_trajectory_tracking.py"
plot_output_dir="${bag_root}/plots"
metadata_output_dir="${bag_root}/experiment_metadata/${bag_name}"
mkdir -p "$bag_root"

# 规划轨迹在不同 Autoware 配置中使用不同名称。至少一个存在即可；如果两个
# 都存在则同时录制，便于确认中间轨迹和最终轨迹是否一致。
trajectory_topic_candidates=(
  "/planning/scenario_planning/trajectory"
  "/planning/trajectory"
)

# 必须保留的实际车辆状态。
required_topics=("/localization/kinematic_state")

# 规划/路线：用于确认网页端发布的起点、终点和路线是否进入 Autoware。
planning_topics=(
  "/planning/mission_planning/route"
  "/planning/mission_planning/goal"
  "/planning/scenario_planning/status/stop_reasons"
)

# 定位链路：kinematic_state 是实际车辆轨迹，其他话题用于判断误差来自
# NDT、EKF、速度估计还是 GNSS/IMU。
localization_topics=(
  "/localization/pose_estimator/pose_with_covariance"
  "/localization/twist_estimator/twist_with_covariance"
  "/localization/twist_estimator/twist_with_covariance_raw"
  "/localization/pose_twist_fusion_filter/pose"
  "/localization/pose_twist_fusion_filter/twist_with_covariance"
  "/localization/acceleration"
  "/sensing/gnss/pose_with_covariance"
  "/sensing/imu/imu_data"
  "/sensing/imu/gins2000/imu_data"
  "/vehicle/status/velocity_status"
  "/vehicle/status/steering_status"
)

# 控制链路：trajectory_follower 是控制器原始输出，command/control_cmd 是经过
# vehicle_cmd_gate 等节点后真正交给底盘桥接的命令。二者都保留才能区分控制器
# 误差、命令门限幅和底盘执行滞后。
control_topics=(
  "/control/trajectory_follower/control_cmd"
  "/control/command/control_cmd"
)

# 运行模式和挡位有助于剔除手动驾驶、急停及非前进挡区间。
vehicle_state_topics=(
  "/vehicle/status/control_mode"
  "/vehicle/status/gear_status"
  "/vehicle/status/actuation_status"
  "/system/operation_mode/state"
)

# 感知：PredictedObjects 用于确认障碍物是否进入规划；分割点云和栅格图
# 用于判断点云是否在感知链路中正常流动。
perception_topics=(
  "/perception/object_recognition/objects"
  "/perception/obstacle_segmentation/pointcloud"
  "/perception/occupancy_grid_map/map"
  "/sensing/lidar/concatenated/pointcloud"
)

# TF 用于离线检查 map/base_link 和传感器坐标系关系。
tf_topics=(
  "/tf"
  "/tf_static"
)

# 如果 control_performance_analysis 已经运行，直接记录其计算出的误差，
# 作为离线自行计算结果的交叉校验。
derived_topics=(
  "/control_performance/performance_vars"
  "/control_performance/driving_status"
  "/control/trajectory_follower/lateral/diagnostic"
)

# parameter_events 只能捕获录制期间的参数变化，不能代替实验前的参数快照，
# 但对排查运行中参数被修改的问题很有用。
system_topics=(
  "/parameter_events"
  "/diagnostics"
)

if ((with_raw_lidar)); then
  perception_topics+=(
    "/sensing/lidar/front/pointcloud_raw"
    "/sensing/lidar/front/pointcloud_raw_ex"
    "/sensing/lidar/top/pointcloud_raw"
    "/sensing/lidar/top/pointcloud_raw_ex"
  )
fi

available_topics="$(ros2 topic list 2>/dev/null || true)"
record_topics=()
missing_required=0
trajectory_topic_count=0

topic_exists()
{
  local topic="$1"
  local available_topic

  while IFS= read -r available_topic; do
    if [[ "$available_topic" == "$topic" ]]; then
      return 0
    fi
  done <<< "$available_topics"

  return 1
}

add_existing_topic()
{
  local topic="$1"
  local required="$2"

  if topic_exists "$topic"; then
    record_topics+=("$topic")
  elif [[ "$required" == "1" ]]; then
    echo "错误：未发现必需话题 $topic。请先启动 Autoware 并确认规划/定位节点正常。" >&2
    missing_required=1
  else
    echo "提示：未发现 $topic，本次跳过。" >&2
  fi
}

for topic in "${required_topics[@]}"; do
  add_existing_topic "$topic" 1
done

for topic in "${trajectory_topic_candidates[@]}"; do
  if topic_exists "$topic"; then
    record_topics+=("$topic")
    trajectory_topic_count=$((trajectory_topic_count + 1))
  fi
done
if ((trajectory_topic_count == 0)); then
  echo "错误：未发现规划轨迹话题，需要以下话题中的至少一个：" >&2
  printf '  %s\n' "${trajectory_topic_candidates[@]}" >&2
  missing_required=1
fi

for topic in \
  "${planning_topics[@]}" \
  "${localization_topics[@]}" \
  "${control_topics[@]}" \
  "${vehicle_state_topics[@]}" \
  "${tf_topics[@]}" \
  "${derived_topics[@]}" \
  "${system_topics[@]}"; do
  add_existing_topic "$topic" 0
done
if ((!tracking_only)); then
  for topic in "${perception_topics[@]}"; do
    add_existing_topic "$topic" 0
  done
fi

if ((missing_required)); then
  exit 1
fi

echo "录包目录：$bag_path"
echo "录制话题："
printf '  %s\n' "${record_topics[@]}"

missing_control_topics=()
for topic in \
  "/control/command/control_cmd" \
  "/vehicle/status/velocity_status" \
  "/vehicle/status/steering_status"; do
  if ! topic_exists "$topic"; then
    missing_control_topics+=("$topic")
  fi
done
if ((${#missing_control_topics[@]} > 0)); then
  echo "警告：以下控制/底盘话题不存在，只能完成部分跟踪分析：" >&2
  printf '  %s\n' "${missing_control_topics[@]}" >&2
fi
if ((tracking_only)); then
  echo "跟踪评估模式：未录制感知点云和栅格，可显著减小 rosbag。"
elif ((with_raw_lidar)); then
  echo "已启用原始激光点云，录包体积可能明显增大。"
else
  echo "已录制处理后感知数据；如只评估控制跟踪，请追加 --tracking-only。"
  echo "未启用原始激光点云；如需原始点云，请追加 --raw-lidar。"
fi

# rosbag 不会自动保存节点的初始参数。这里额外保存 ROS graph、实际录制话题和
# 控制链路节点参数，确保不同实验之间的误差结果可以复现和对比。
mkdir -p "$metadata_output_dir/parameters"
{
  echo "recorded_at=$(date --iso-8601=seconds)"
  echo "bag_path=$bag_path"
  echo "raw_lidar=$with_raw_lidar"
  echo "tracking_only=$tracking_only"
  echo "ros_distro=${ROS_DISTRO:-unknown}"
} >"$metadata_output_dir/session.txt"
printf '%s\n' "${record_topics[@]}" >"$metadata_output_dir/recorded_topics.txt"
ros2 topic list -t >"$metadata_output_dir/ros_topics.txt" 2>&1 || true
ros2 node list >"$metadata_output_dir/ros_nodes.txt" 2>&1 || true

while IFS= read -r node_name; do
  case "$node_name" in
    *trajectory_follower* | *lateral_controller* | *longitudinal_controller* | \
      *vehicle_cmd_gate* | *raw_vehicle_cmd_converter* | *hunter_autoware_bridge* | \
      *yunle_autoware_bridge*)
      timeout 5 ros2 param dump "$node_name" \
        --output-dir "$metadata_output_dir/parameters" \
        >>"$metadata_output_dir/parameter_dump.log" 2>&1 || \
        echo "参数保存失败或超时：$node_name" >>"$metadata_output_dir/parameter_dump.log"
      ;;
  esac
done <"$metadata_output_dir/ros_nodes.txt"

echo "实验元数据：$metadata_output_dir"
echo "按 Ctrl+C 停止录制。"

record_status=0
ros2 bag record \
  --output "$bag_path" \
  "${record_topics[@]}" || record_status=$?

# ros2 bag record normally returns 0 after Ctrl+C, but some ROS 2 versions
# return 130 because the recorder received SIGINT. Both cases mean recording
# has ended and the bag should be analyzed.
if ((record_status != 0 && record_status != 130)); then
  echo "警告：录包命令退出码为 ${record_status}，仍尝试绘图。" >&2
fi

if [[ ! -f "${bag_path}/metadata.yaml" ]]; then
  echo "错误：录制结束后未找到 rosbag metadata.yaml：${bag_path}" >&2
  exit "${record_status}"
fi

echo "录制完成，开始绘制误差和轨迹图 ..."
plot_status=0
if python3 "$plot_script" "$bag_path" --output-dir "$plot_output_dir"; then
  echo "绘图完成，图片目录：$plot_output_dir"
else
  plot_status=$?
  echo "警告：绘图失败，但录包已保留：$bag_path" >&2
fi

if ((record_status != 0 && record_status != 130)); then
  exit "$record_status"
fi
exit "$plot_status"
