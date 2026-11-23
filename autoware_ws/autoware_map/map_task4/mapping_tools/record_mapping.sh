#!/usr/bin/env bash
# =============================================================================
# 建图录包脚本（ROS 2 Humble · MID-360 · FAST-LIO2）
#
# 用法：
#   ./record_mapping.sh <地图名> [备注]
#   例：./record_mapping.sh floor3_east "3F 东侧，门全开，起点=东门交接点"
#
# 前提：本机 livox_ros_driver2（PointCloud2）和建图专用 FAST-LIO2
#       （lidar_type=4, dense_publish_en=true）已经在跑。本脚本只负责检查 + 录制。
#
# 可用环境变量覆盖：
#   BAG_ROOT       录包根目录          默认 /home/tang/fastlio_ws/bags/mapping
#   STORAGE        mcap | sqlite3      默认 mcap（未装插件时自动退回 sqlite3）
#   SPLIT_SEC      分包时长(秒)        默认 300
#   EXTRA_TOPICS   额外话题(空格分隔)  例："/chassis/odom /camera/depth/points"
#   FASTLIO_CFG    FAST-LIO 参数文件   录制时复制一份进包目录
#   LIVOX_CFG      Livox 驱动 json     同上
#   MIN_FREE_GB    最小剩余空间(GB)    默认 60
# =============================================================================
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 本机快捷配置：允许从任意终端直接运行，不要求用户事先 source ROS。
FASTLIO_ENV="${FASTLIO_ENV:-/home/tang/fastlio_ws/scripts/fastlio_env.sh}"
if [[ -r "$FASTLIO_ENV" ]]; then
  # shellcheck disable=SC1090
  set +u
  source "$FASTLIO_ENV"
  set -u
fi
GINS_ENV="${GINS_ENV:-/home/tang/gins2000_ws/install/setup.bash}"
if [[ -r "$GINS_ENV" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "$GINS_ENV"
  set -u
fi

NAME="${1:-}"
NOTE="${2:-}"
if [[ -z "$NAME" ]]; then
  echo "用法: $0 <地图名> [备注]"; exit 1
fi

BAG_ROOT="${BAG_ROOT:-/home/tang/fastlio_ws/bags/mapping}"
STORAGE="${STORAGE:-mcap}"
SPLIT_SEC="${SPLIT_SEC:-300}"
MIN_FREE_GB="${MIN_FREE_GB:-60}"
EXTRA_TOPICS="${EXTRA_TOPICS:-}"
FASTLIO_CFG="${FASTLIO_CFG:-$HERE/config/fastlio_mid360s_mapping.yaml}"
LIVOX_CFG="${LIVOX_CFG:-/home/tang/livox_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360s_config.json}"

# ---- 话题（与驱动 / FAST-LIO 配置保持一致）----
LIDAR_TOPIC="${LIDAR_TOPIC:-/livox/lidar}"     # 本机原始 PointCloud2，可用同一 lidar_type=4 配置离线重跑
IMU_TOPIC="${IMU_TOPIC:-/livox/imu}"           # 原始 IMU 200 Hz —— 同上
CLOUD_TOPIC="${CLOUD_TOPIC:-/cloud_registered}"            # 世界系去畸变点云（bag2dufo 用）
CLOUD_BODY_TOPIC="${CLOUD_BODY_TOPIC:-/cloud_registered_body}"
ODOM_TOPIC="${ODOM_TOPIC:-/Odometry}"
PATH_TOPIC="${PATH_TOPIC:-/path}"
GINS_TOPIC="${GINS_TOPIC:-/gins2000}"
LIDAR_EXPECTED_TYPE="${LIDAR_EXPECTED_TYPE:-sensor_msgs/msg/PointCloud2}"

REQUIRED=("$LIDAR_TOPIC" "$IMU_TOPIC" "$CLOUD_TOPIC" "$ODOM_TOPIC" "$GINS_TOPIC")
# /cloud_registered_body 不默认录（48 B/点，体积翻倍；需要时离线重跑 FAST-LIO 生成，或加到 EXTRA_TOPICS）
OPTIONAL=("$PATH_TOPIC" /tf /tf_static \
  /sensing/gnss/fix \
  /sensing/gnss/ins_pose_with_covariance \
  /sensing/imu/imu_raw)
# 期望频率（用于录后检查）
declare -A EXPECT_HZ=( ["$LIDAR_TOPIC"]=10 ["$IMU_TOPIC"]=200 ["$CLOUD_TOPIC"]=10 ["$ODOM_TOPIC"]=10 )

red()   { echo -e "\033[31m$*\033[0m"; }
green() { echo -e "\033[32m$*\033[0m"; }
yellow(){ echo -e "\033[33m$*\033[0m"; }

# ---------------------------------------------------------------- 预检
command -v ros2 >/dev/null || { red "找不到 ros2，先 source /opt/ros/humble/setup.bash 和工作空间"; exit 1; }

mkdir -p "$BAG_ROOT"
FREE_GB=$(df -BG --output=avail "$BAG_ROOT" | tail -1 | tr -dc '0-9')
if (( FREE_GB < MIN_FREE_GB )); then
  red "磁盘剩余 ${FREE_GB}GB < ${MIN_FREE_GB}GB（每小时约 50GB），先清理"; exit 1
fi
echo "磁盘剩余 ${FREE_GB}GB"

if [[ "$STORAGE" == "mcap" ]] && ! ros2 pkg prefix rosbag2_storage_mcap >/dev/null 2>&1; then
  yellow "未安装 rosbag2_storage_mcap（sudo apt install ros-humble-rosbag2-storage-mcap），改用 sqlite3"
  STORAGE=sqlite3
fi

echo "检查话题 ..."
TOPIC_LIST=$(ros2 topic list 2>/dev/null)
fail=0
for t in "${REQUIRED[@]}"; do
  if ! grep -qx "$t" <<<"$TOPIC_LIST"; then
    red "  缺少必需话题 $t"; fail=1; continue
  fi
  # 真的有数据在发（只看 3 s）
  if timeout 5 ros2 topic echo --once --no-arr "$t" >/dev/null 2>&1; then
    green "  ✓ $t"
  else
    red "  ✗ $t 存在但 5 s 内没有消息"; fail=1
  fi
done
(( fail )) && { red "预检失败，不录制"; exit 1; }

TOPICS=("${REQUIRED[@]}")
for t in "${OPTIONAL[@]}" $EXTRA_TOPICS; do
  if grep -qx "$t" <<<"$TOPIC_LIST"; then TOPICS+=("$t"); else yellow "  (可选话题 $t 不存在，跳过)"; fi
done

# 本机 MID360S 驱动和 FAST-LIO 使用 PointCloud2（lidar_type=4）。
LTYPE=$(ros2 topic type "$LIDAR_TOPIC" 2>/dev/null || true)
if [[ "$LTYPE" != "$LIDAR_EXPECTED_TYPE" ]]; then
  red "  $LIDAR_TOPIC 类型是 '$LTYPE'，本配置要求 '$LIDAR_EXPECTED_TYPE'"
  red "  请用 $HERE/00_open_mapping_terminals.sh 启动本机驱动和 FAST-LIO"
  exit 1
fi

GTYPE=$(ros2 topic type "$GINS_TOPIC" 2>/dev/null || true)
if [[ "$GTYPE" != "gins2000/msg/Gins2000" ]]; then
  red "  $GINS_TOPIC 类型是 '$GTYPE'，要求 'gins2000/msg/Gins2000'"
  red "  请检查 GINS2000 驱动是否由 $HERE/01_gins2000_driver.sh 启动"
  exit 1
fi

# ---------------------------------------------------------------- 目录与元数据
STAMP=$(date +%Y%m%d_%H%M%S)
SESSION="$BAG_ROOT/${NAME}_${STAMP}"
mkdir -p "$SESSION/meta"
BAG_DIR="$SESSION/bag"

{
  echo "name: $NAME"
  echo "note: \"$NOTE\""
  echo "start_local: $(date --iso-8601=seconds)"
  echo "host: $(hostname)"
  echo "storage: $STORAGE"
  echo "lidar_type: $LTYPE"
  echo "split_sec: $SPLIT_SEC"
  echo "topics:"; for t in "${TOPICS[@]}"; do echo "  - $t"; done
  echo "ros_distro: ${ROS_DISTRO:-unknown}"
  echo "chrony: \"$(chronyc tracking 2>/dev/null | grep -E 'System time' | xargs || echo n/a)\""
} > "$SESSION/meta/session.yaml"

[[ -f "$FASTLIO_CFG" ]] && cp "$FASTLIO_CFG" "$SESSION/meta/"
[[ -f "$LIVOX_CFG"   ]] && cp "$LIVOX_CFG"   "$SESSION/meta/"
ros2 node list > "$SESSION/meta/nodes.txt" 2>/dev/null || true
ros2 param dump /laser_mapping > "$SESSION/meta/fastlio_params_live.yaml" 2>/dev/null || true
# 保存一帧原始组合导航数据，便于快速核对经纬度、航向角和定位状态。
timeout 5 ros2 topic echo "$GINS_TOPIC" --once > "$SESSION/meta/gins2000_first.txt" 2>/dev/null || true

# ---------------------------------------------------------------- 录制
echo
green "==> 录制到 $BAG_DIR"
echo "    话题: ${TOPICS[*]}"
yellow "    先静止 10 s（IMU 初始化）再出发；结束时回到起点再静止 10 s，然后 Ctrl+C"
echo

ros2 bag record -o "$BAG_DIR" -s "$STORAGE" \
  --max-bag-duration "$SPLIT_SEC" \
  --max-cache-size 209715200 \
  "${TOPICS[@]}" &
REC_PID=$!

finish() {
  trap - INT TERM
  echo; echo "停止录制 ..."
  kill -INT "$REC_PID" 2>/dev/null || true
  wait "$REC_PID" 2>/dev/null || true
  echo "end_local: $(date --iso-8601=seconds)" >> "$SESSION/meta/session.yaml"
  check_bag
  exit 0
}

# ---------------------------------------------------------------- 录后检查
check_bag() {
  [[ -f "$BAG_DIR/metadata.yaml" ]] || { red "没有生成 metadata.yaml，录包可能失败"; return; }
  ros2 bag info "$BAG_DIR" | tee "$SESSION/meta/bag_info.txt"
  local dur
  dur=$(grep -E '^Duration:' "$SESSION/meta/bag_info.txt" | grep -oE '[0-9.]+' | head -1)
  echo; echo "频率检查（时长 ${dur}s）："
  local bad=0
  for t in "${!EXPECT_HZ[@]}"; do
    local cnt
    cnt=$(grep -E "Topic: $t \|" "$SESSION/meta/bag_info.txt" | grep -oE 'Count: [0-9]+' | grep -oE '[0-9]+' || echo 0)
    local hz
    hz=$(awk -v c="$cnt" -v d="$dur" 'BEGIN{ if(d>0) printf "%.1f", c/d; else print 0 }')
    local ok
    ok=$(awk -v h="$hz" -v e="${EXPECT_HZ[$t]}" 'BEGIN{ print (h >= 0.95*e) ? 1 : 0 }')
    if (( ok )); then green "  ✓ $t  $hz Hz (期望 ${EXPECT_HZ[$t]})"
    else red "  ✗ $t  $hz Hz (期望 ${EXPECT_HZ[$t]}) —— 丢帧，建议重录"; bad=1; fi
  done
  (( bad )) || green "录包完成：$SESSION"
  echo "下一步：./postprocess.sh $BAG_DIR $SESSION/work"
}

trap finish INT TERM
wait "$REC_PID"
finish
