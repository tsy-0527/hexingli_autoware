#!/usr/bin/env bash
# 启动 GINS2000 原始驱动和 Autoware 标准消息转换器。
set -euo pipefail

GINS_PORT="${GINS_PORT:-/dev/ttyUSB0}"
GINS_BAUD="${GINS_BAUD:-921600}"
GINS_TOPIC="${GINS_TOPIC:-/gins2000}"

set +u
source /opt/ros/humble/setup.bash
[[ -r /home/tang/autoware/install/setup.bash ]] && source /home/tang/autoware/install/setup.bash
[[ -r /home/tang/gins2000_ws/install/setup.bash ]] && source /home/tang/gins2000_ws/install/setup.bash
set -u

ros2 pkg prefix gins2000 >/dev/null 2>&1 || {
  echo "找不到 gins2000 ROS 2 包，请检查 /home/tang/gins2000_ws/install" >&2
  exit 1
}
[[ -e "$GINS_PORT" ]] || {
  echo "找不到 GINS2000 串口: $GINS_PORT" >&2
  echo "现有串口：" >&2
  ls -l /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id/* 2>/dev/null || true
  exit 1
}
[[ -r "$GINS_PORT" && -w "$GINS_PORT" ]] || {
  echo "当前用户无权读写 $GINS_PORT" >&2
  echo "请执行 sudo usermod -aG dialout tang，然后注销并重新登录。" >&2
  exit 1
}

DRIVER_PID=""
CONVERTER_PID=""
cleanup() {
  trap - EXIT INT TERM
  [[ -n "$CONVERTER_PID" ]] && kill -INT "$CONVERTER_PID" 2>/dev/null || true
  [[ -n "$DRIVER_PID" ]] && kill -INT "$DRIVER_PID" 2>/dev/null || true
  [[ -n "$CONVERTER_PID" ]] && wait "$CONVERTER_PID" 2>/dev/null || true
  [[ -n "$DRIVER_PID" ]] && wait "$DRIVER_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "启动 GINS2000: port=$GINS_PORT baud=$GINS_BAUD topic=$GINS_TOPIC"
ros2 run gins2000 gins2000_node --ros-args \
  -r __node:=gins2000_driver \
  -p port:="$GINS_PORT" \
  -p gins2000_baudrate:="$GINS_BAUD" \
  -p gins2000_frame_id:=gins2000_link \
  -p gins2000_topic:="$GINS_TOPIC" &
DRIVER_PID=$!

sleep 1
kill -0 "$DRIVER_PID" 2>/dev/null || { echo "GINS2000 驱动启动失败" >&2; exit 1; }

ros2 run gins2000 converter_node.py --ros-args \
  -r __node:=gins2000_converter \
  -p input_topic:="$GINS_TOPIC" \
  -p imu_frame:=gnss_link \
  -p gnss_frame:=gnss_link &
CONVERTER_PID=$!

echo "原始经纬度/航向角: $GINS_TOPIC (attitude.z 为航向角，单位度)"
echo "标准输出: /sensing/gnss/fix, /sensing/gnss/ins_pose_with_covariance, /sensing/imu/imu_raw"
wait -n "$DRIVER_PID" "$CONVERTER_PID"
