#!/usr/bin/env bash

# 记录 Autoware 控制跟踪误差。
#
# 用法：
#   ./scripts/record_tracking_errors.sh
#   ./scripts/record_tracking_errors.sh <录包根目录> <录包名称>
#
# 默认输出：
#   ~/autoware_bag/tracking_errors_YYYYMMDD_HHMMSS
#
# 录包中的主要字段：
#   /control_performance/performance_vars/error/lateral_error  [m]
#   /control_performance/performance_vars/error/heading_error  [rad]
#
# 同时记录（如果当前 ROS graph 中存在）：
#   /planning/trajectory 或 /planning/scenario_planning/trajectory
#   /localization/kinematic_state

# ROS 2 setup scripts may reference variables that are not defined yet, so
# enable nounset only after all workspaces have been sourced.
set -Ee -o pipefail

# 与启动 Autoware 时保持相同的工作空间叠加顺序。
source /opt/ros/humble/setup.bash
source "$HOME/livox_ws/install/setup.bash"
source "$HOME/gins2000_ws/install/setup.bash"
source "$HOME/hunter_ws/install/setup.bash"
source "$HOME/autoware/install/setup.bash"

set -u

readonly ERROR_TOPIC="/control_performance/performance_vars"
readonly ERROR_MSG_TYPE="autoware_control_performance_analysis/msg/ErrorStamped"
readonly ODOMETRY_TOPIC="/localization/kinematic_state"
readonly CONTROL_TOPIC="/control/command/control_cmd"
readonly STEERING_TOPIC="/vehicle/status/steering_status"
readonly BAG_ROOT="${1:-$HOME/autoware_bag}"
readonly BAG_NAME="${2:-tracking_errors_$(date +%Y%m%d_%H%M%S)}"
readonly BAG_PATH="$BAG_ROOT/$BAG_NAME"
readonly ANALYSIS_LOG="$BAG_ROOT/${BAG_NAME}_control_performance.log"

mkdir -p "$BAG_ROOT"

available_topics="$(ros2 topic list 2>/dev/null || true)"
selected_trajectory_topic=""
for candidate_topic in \
  "/planning/trajectory" \
  "/planning/scenario_planning/trajectory"; do
  if rg -Fxq "$candidate_topic" <<<"$available_topics"; then
    selected_trajectory_topic="$candidate_topic"
    break
  fi
done

if [[ -z "$selected_trajectory_topic" ]]; then
  echo "未发现规划轨迹话题；请先启动 Autoware。" >&2
  echo "需要 /planning/trajectory 或 /planning/scenario_planning/trajectory。" >&2
  exit 1
fi

missing_inputs=()
for required_topic in "$ODOMETRY_TOPIC" "$CONTROL_TOPIC" "$STEERING_TOPIC" "/tf"; do
  if ! rg -Fxq "$required_topic" <<<"$available_topics"; then
    missing_inputs+=("$required_topic")
  fi
done
if ((${#missing_inputs[@]} > 0)); then
  echo "缺少 control_performance_analysis 输入话题：" >&2
  printf '  %s\n' "${missing_inputs[@]}" >&2
  exit 1
fi

analysis_pid=""

wait_for_error_message()
{
  local deadline=$((SECONDS + 30))

  while (( SECONDS < deadline )); do
    local topic_type
    topic_type="$(ros2 topic type "$ERROR_TOPIC" 2>/dev/null | head -n 1 || true)"

    if [[ "$topic_type" == "$ERROR_MSG_TYPE" ]] && \
      timeout 3 ros2 topic echo "$ERROR_TOPIC" --once >/dev/null 2>&1; then
      return 0
    fi

    if [[ -n "$analysis_pid" ]] && ! kill -0 "$analysis_pid" 2>/dev/null; then
      return 1
    fi
  done

  return 1
}

cleanup()
{
  if [[ -n "$analysis_pid" ]] && kill -0 "$analysis_pid" 2>/dev/null; then
    echo
    echo "正在停止 control_performance_analysis..."
    kill -TERM "$analysis_pid" 2>/dev/null || true
    wait "$analysis_pid" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

# 给已有的分析节点留出复用机会；没有该 topic 时自动启动分析节点。
if ros2 topic list 2>/dev/null | rg -Fxq "$ERROR_TOPIC"; then
  echo "检测到已有 $ERROR_TOPIC，直接开始录制。"
else
  echo "启动 control_performance_analysis..."
  ros2 launch autoware_control_performance_analysis \
    control_performance_analysis.launch.xml \
    input/reference_trajectory:="$selected_trajectory_topic" \
    >"$ANALYSIS_LOG" 2>&1 &
  analysis_pid=$!

fi

echo "等待第一条横向/航向误差消息 ..."
if ! wait_for_error_message; then
  echo "未收到有效的 $ERROR_MSG_TYPE 消息。请确认分析输入话题正在发布且时间/坐标系有效。" >&2
  if [[ -n "$analysis_pid" ]]; then
    echo "分析节点日志：$ANALYSIS_LOG" >&2
    tail -n 40 "$ANALYSIS_LOG" >&2 || true
  fi
  exit 1
fi

echo "开始录制：$BAG_PATH"
echo "按 Ctrl+C 停止录制。"

topics_to_record=(
  "$ERROR_TOPIC"
  "$selected_trajectory_topic"
  "$ODOMETRY_TOPIC"
  "$CONTROL_TOPIC"
  "$STEERING_TOPIC"
  "/tf"
)
for candidate_topic in \
  "/control_performance/driving_status" \
  "/control/trajectory_follower/control_cmd" \
  "/vehicle/status/velocity_status" \
  "/localization/pose_estimator/pose_with_covariance" \
  "/tf_static"; do
  if ros2 topic list 2>/dev/null | rg -Fxq "$candidate_topic"; then
    topics_to_record+=("$candidate_topic")
  else
    echo "提示：当前未发现 $candidate_topic，本次不录制。"
  fi
done

echo "录制 topic：${topics_to_record[*]}"

ros2 bag record \
  --output "$BAG_PATH" \
  "${topics_to_record[@]}"
