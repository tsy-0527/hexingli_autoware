#!/usr/bin/env bash

# Record time-synchronized GNSS/INS, FAST-LIO odometry, TF, and optional
# Autoware localization topics for map-frame alignment analysis.
#
# Usage:
#   ./scripts/record_map_alignment.sh
#   ./scripts/record_map_alignment.sh --fastlio-odom /your/fastlio/odometry \
#     --output-dir "$HOME/autoware_bag" --name map_task1_alignment

# ROS setup scripts can reference variables before defining them. Enable
# nounset only after sourcing the available workspaces.
set -Ee -o pipefail

usage()
{
  cat <<'EOF'
Usage:
  record_map_alignment.sh [options]

FAST-LIO:
  --fastlio-odom TOPIC   FAST-LIO nav_msgs/Odometry topic from the same run
                         that generated the point-cloud map (default: /Odometry).

Options:
  --gnss-fix TOPIC       GNSS NavSatFix topic (default: /sensing/gnss/fix)
  --ins TOPIC            GNSS/INS orientation topic (default: /sensing/gins2000)
  --output-dir DIR       Rosbag root directory (default: ~/autoware_bag)
  --name NAME            Bag directory name (default: map_alignment_TIMESTAMP)
  --extra-topic TOPIC    Add another topic; may be repeated.
  -h, --help             Show this help.

The script also records /tf and /tf_static, and records useful Autoware GNSS
and localization pose topics when they are present in the ROS graph.
Stop recording with Ctrl+C.
EOF
}

gnss_fix_topic="${GNSS_FIX_TOPIC:-/sensing/gnss/fix}"
ins_topic="${INS_ORIENTATION_TOPIC:-/sensing/gins2000}"
fastlio_odom_topic="${FASTLIO_ODOM_TOPIC:-/Odometry}"
bag_root="${MAP_ALIGNMENT_BAG_ROOT:-${HOME}/autoware_bag}"
bag_name="map_alignment_$(date +%Y%m%d_%H%M%S)"
extra_topics=()

while (($#)); do
  case "$1" in
    --fastlio-odom)
      (($# >= 2)) || { echo "缺少 --fastlio-odom 的话题名" >&2; exit 2; }
      fastlio_odom_topic="$2"
      shift 2
      ;;
    --gnss-fix)
      (($# >= 2)) || { echo "缺少 --gnss-fix 的话题名" >&2; exit 2; }
      gnss_fix_topic="$2"
      shift 2
      ;;
    --ins)
      (($# >= 2)) || { echo "缺少 --ins 的话题名" >&2; exit 2; }
      ins_topic="$2"
      shift 2
      ;;
    --output-dir)
      (($# >= 2)) || { echo "缺少 --output-dir 的目录名" >&2; exit 2; }
      bag_root="$2"
      shift 2
      ;;
    --name)
      (($# >= 2)) || { echo "缺少 --name 的名称" >&2; exit 2; }
      bag_name="$2"
      shift 2
      ;;
    --extra-topic)
      (($# >= 2)) || { echo "缺少 --extra-topic 的话题名" >&2; exit 2; }
      extra_topics+=("$2")
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "未知参数：$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

source_if_exists()
{
  local setup_file="$1"
  if [[ -f "$setup_file" ]]; then
    # shellcheck disable=SC1090
    source "$setup_file"
  fi
}

source_if_exists /opt/ros/humble/setup.bash
source_if_exists /home/tang/autoware/install/setup.bash
source_if_exists /home/tang/hunter_ws/install/setup.bash
source_if_exists /home/tang/gins2000_ws/install/setup.bash
source_if_exists /home/tang/livox_ws/install/setup.bash

set -u

if ! command -v ros2 >/dev/null 2>&1; then
  echo "错误：找不到 ros2。请先安装 ROS 2 或修改脚本中的 setup 路径。" >&2
  exit 1
fi

bag_path="${bag_root}/${bag_name}"
if [[ -e "$bag_path" ]]; then
  echo "错误：输出路径已经存在：$bag_path" >&2
  exit 1
fi
mkdir -p "$bag_root"

available_topics="$(ros2 topic list 2>/dev/null || true)"
if [[ -z "$available_topics" ]]; then
  echo "错误：当前没有发现 ROS 2 话题。请先 source 工作空间并启动传感器。" >&2
  exit 1
fi

topic_exists()
{
  local wanted="$1"
  local listed
  while IFS= read -r listed; do
    [[ "$listed" == "$wanted" ]] && return 0
  done <<< "$available_topics"
  return 1
}

record_topics=()
contains_topic()
{
  local wanted="$1"
  local existing
  for existing in "${record_topics[@]}"; do
    [[ "$existing" == "$wanted" ]] && return 0
  done
  return 1
}

add_required_topic()
{
  local topic="$1"
  if ! topic_exists "$topic"; then
    echo "错误：必需话题不存在：$topic" >&2
    return 1
  fi
  if ! contains_topic "$topic"; then
    record_topics+=("$topic")
  fi
}

add_optional_topic()
{
  local topic="$1"
  if topic_exists "$topic"; then
    if ! contains_topic "$topic"; then
      record_topics+=("$topic")
    fi
  else
    echo "提示：当前没有 $topic，本次跳过。" >&2
  fi
}

add_required_topic "$gnss_fix_topic" || exit 1
add_required_topic "$ins_topic" || exit 1
add_required_topic "$fastlio_odom_topic" || exit 1

for optional_topic in \
  /tf \
  /tf_static \
  /sensing/gnss/pose \
  /sensing/gnss/pose_with_covariance \
  /localization/pose_estimator/pose_with_covariance \
  /localization/kinematic_state; do
  add_optional_topic "$optional_topic"
done

for topic in "${extra_topics[@]}"; do
  add_required_topic "$topic" || exit 1
done

echo "录包目录：$bag_path"
echo "请确认车辆静止、RTK 状态稳定后再开始行驶。"
echo "录制话题及类型："
for topic in "${record_topics[@]}"; do
  topic_type="$(ros2 topic type "$topic" 2>/dev/null || echo unknown)"
  printf '  %-58s %s\n' "$topic" "$topic_type"
done
echo "建议跑一段长直线后掉头返回起点；按 Ctrl+C 停止录制。"

record_status=0
ros2 bag record --output "$bag_path" "${record_topics[@]}" || record_status=$?

if [[ -f "${bag_path}/metadata.yaml" ]]; then
  echo "录制完成：$bag_path"
else
  echo "错误：未找到 rosbag metadata.yaml，请检查录制输出。" >&2
  if ((record_status == 0 || record_status == 130)); then
    exit 1
  fi
  exit "$record_status"
fi

if ((record_status != 0 && record_status != 130)); then
  echo "ros2 bag record 退出码：$record_status" >&2
  exit "$record_status"
fi
