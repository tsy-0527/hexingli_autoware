#!/usr/bin/env bash
# 从完整的原始 PointCloud2 + IMU bag 离线重跑 FAST-LIO。
# 用法: ./recover_fastlio_from_bag.sh <原bag目录> <输出bag目录> [回放倍率]
# 环境变量:
#   START_OFFSET  从 bag 第几秒开始（默认 0）
#   PLAY_DURATION 只回放多少秒（bag 时间，默认全部）
#   FASTLIO_MAPPING_CFG  使用的 FAST-LIO 配置
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INPUT="${1:?用法: $0 <原bag目录> <输出bag目录> [回放倍率]}"
OUTPUT="${2:?}"
RATE="${3:-0.5}"
START_OFFSET="${START_OFFSET:-0}"
PLAY_DURATION="${PLAY_DURATION:-}"
CFG="${FASTLIO_MAPPING_CFG:-$HERE/config/fastlio_mid360s_mapping.yaml}"
FASTLIO_ENV="${FASTLIO_ENV:-/home/tang/fastlio_ws/scripts/fastlio_env.sh}"

[[ -f "$INPUT/metadata.yaml" ]] || { echo "输入不是有效 bag: $INPUT" >&2; exit 1; }
[[ ! -e "$OUTPUT" ]] || { echo "拒绝覆盖已有输出: $OUTPUT" >&2; exit 1; }
[[ -r "$CFG" ]] || { echo "找不到 FAST-LIO 配置: $CFG" >&2; exit 1; }

set +u
# shellcheck disable=SC1090
source "$FASTLIO_ENV"
set -u

# 离线恢复全部是本机进程，固定到 loopback，避免 Wi-Fi 断线/漫游改变 DDS 接口。
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${RECOVERY_ROS_DOMAIN_ID:-73}"
RUN_DIR="$(dirname "$OUTPUT")/recovery_logs"
mkdir -p "$RUN_DIR"
export ROS_LOG_DIR="$RUN_DIR/ros"

FAST_PID=""
REC_PID=""
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$REC_PID" ]]; then
    kill -INT "$REC_PID" 2>/dev/null || true
    wait "$REC_PID" 2>/dev/null || true
  fi
  if [[ -n "$FAST_PID" ]]; then
    kill -INT "$FAST_PID" 2>/dev/null || true
    wait "$FAST_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[1/4] 启动离线 FAST-LIO（ROS_LOCALHOST_ONLY=1, domain=$ROS_DOMAIN_ID）"
ros2 launch fast_lio mapping.launch.py \
  "config_path:=$(dirname "$CFG")" \
  "config_file:=$(basename "$CFG")" \
  rviz:=false >"$RUN_DIR/fastlio.log" 2>&1 &
FAST_PID=$!

for _ in $(seq 1 30); do
  ros2 topic type /cloud_registered >/dev/null 2>&1 && break
  kill -0 "$FAST_PID" 2>/dev/null || { echo "FAST-LIO 启动失败，查看 $RUN_DIR/fastlio.log" >&2; exit 1; }
  sleep 1
done
ros2 topic type /cloud_registered >/dev/null 2>&1 || { echo "FAST-LIO 输出话题未出现" >&2; exit 1; }

echo "[2/4] 录制恢复后的世界系点云和里程计"
ros2 bag record -o "$OUTPUT" -s mcap \
  /cloud_registered /Odometry /path /tf >"$RUN_DIR/record.log" 2>&1 &
REC_PID=$!
sleep 2

echo "[3/4] 以 ${RATE}x 回放原始雷达和 IMU（start=${START_OFFSET}s, duration=${PLAY_DURATION:-all}s）"
PLAY_ARGS=(--rate "$RATE" --topics /livox/lidar /livox/imu)
if [[ "$START_OFFSET" != "0" && "$START_OFFSET" != "0.0" ]]; then
  PLAY_ARGS+=(--start-offset "$START_OFFSET")
fi
if [[ -n "$PLAY_DURATION" && "$PLAY_DURATION" != "0" ]]; then
  # timeout 按 wall time 计；回放倍率 RATE，所以 wall = 段时长 / RATE，再留 5 s。
  PLAY_TIMEOUT=$(awk -v d="$PLAY_DURATION" -v r="$RATE" 'BEGIN{printf "%.1f", d/r + 5.0}')
  timeout --signal=INT "$PLAY_TIMEOUT" ros2 bag play "$INPUT" "${PLAY_ARGS[@]}" \
    >"$RUN_DIR/play.log" 2>&1 || true
else
  ros2 bag play "$INPUT" "${PLAY_ARGS[@]}" >"$RUN_DIR/play.log" 2>&1
fi

echo "等待 FAST-LIO 清空内部队列 ..."
sleep 15
kill -INT "$REC_PID" 2>/dev/null || true
wait "$REC_PID" 2>/dev/null || true
REC_PID=""
kill -INT "$FAST_PID" 2>/dev/null || true
wait "$FAST_PID" 2>/dev/null || true
FAST_PID=""

echo "[4/4] 恢复结果"
ros2 bag info "$OUTPUT"
echo "完成: $OUTPUT"
