#!/bin/bash
# ==============================================================
# Autoware + GNSS+INS 定位 + rosbag 录制启动脚本
#
# 用法:
#   ./run_autoware.sh              # 正常启动
#   ./run_autoware.sh --no-bag     # 不录 rosbag
#   ./run_autoware.sh --map <path> # 覆盖默认地图路径
#
# 关键配置:
#   pose_source=external           # 不启动 Autoware 自带 pose_estimator
#   twist_source=external          # 不启动 gyro_odometer
#   gnss_enabled=false             # 不启动 autoware_gnss_poser(避免和 gnss_ins_localizer 冲突)
#   automatic_pose_initializer=false  # 只有手动 call service 才 init
#
# 依赖修复(必须已完成):
#   1. gyro_bias_estimator 已在 livox_sensor_kit/imu.launch.xml 里移除
#      (它对比 Livox IMU 和 GNSS pose,两颗不同 IMU 必然报 ERROR,阻断 engage)
# ==============================================================

set -e

# ─── 可配置项 ─────────────────────────────────────────
MAP_PATH="${HOME}/autoware_map/map_task1"
VEHICLE_MODEL="hunter_vehicle"
SENSOR_MODEL="livox_sensor_kit"
GINS_TOPIC="/sensing/gins2000"
BAG_ROOT="${HOME}/tsy/rosbags"
RECORD_BAG=true

# ─── 参数解析 ────────────────────────────────────────
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-bag) RECORD_BAG=false; shift ;;
    --map)    MAP_PATH="$2"; shift 2 ;;
    -h|--help)
      echo "用法: $0 [--no-bag] [--map <path>]"
      exit 0
      ;;
    *) echo "unknown arg: $1"; exit 1 ;;
  esac
done

# ─── 环境 source ─────────────────────────────────────
export LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
source ~/gins2000_ws/install/setup.bash
source ~/hunter_ws/install/setup.bash
source ~/autoware/install/setup.bash
source ~/tsy/gnss_localizer_ws/install/setup.bash

# ─── 清理旧进程(防止残留冲突) ────────────────────
echo "[INFO] Cleaning up stale ROS processes..."
pkill -9 -f "topic_tools/relay"       2>/dev/null || true
pkill -9 -f "component_container"     2>/dev/null || true
pkill -9 -f "ros2 launch"             2>/dev/null || true
pkill -9 -f _ros2_daemon              2>/dev/null || true
pkill -9 -f rviz                      2>/dev/null || true
pkill -9 -f simple_planning_simulator 2>/dev/null || true
pkill -9 -f gnss_ins_localizer        2>/dev/null || true
pkill -9 -f hunter_autoware_bridge    2>/dev/null || true
pkill -9 -f fake_gins_publisher       2>/dev/null || true
sleep 3

# ─── 日志目录 ────────────────────────────────────────
TS=$(date +%Y%m%d_%H%M%S)
LOG_DIR="${BAG_ROOT}/${TS}_logs"
mkdir -p "${LOG_DIR}"
echo "[INFO] Log dir: ${LOG_DIR}"

# ─── 存 PID 用于统一清理 ─────────────────────────────
PIDS=()
cleanup() {
  echo ""
  echo "[INFO] Shutting down all children..."
  for pid in "${PIDS[@]}"; do
    kill -INT "$pid" 2>/dev/null || true
  done
  sleep 3
  for pid in "${PIDS[@]}"; do
    kill -9 "$pid" 2>/dev/null || true
  done
  pkill -9 -f "topic_tools/relay"   2>/dev/null || true
  pkill -9 -f "component_container" 2>/dev/null || true
  echo "[INFO] Bye."
}
trap cleanup EXIT INT TERM

# ─── 1. 启动 Autoware ────────────────────────────────
echo "[INFO] [1/3] Launching Autoware..."
ros2 launch autoware_launch autoware.launch.xml \
  map_path:="${MAP_PATH}" \
  vehicle_model:="${VEHICLE_MODEL}" \
  sensor_model:="${SENSOR_MODEL}" \
  pose_source:=external \
  twist_source:=external \
  gnss_enabled:=false \
  pose_initializer_gnss_pose_topic:=/sensing/gnss/pose_with_covariance \
  pose_initializer_stop_check_enabled:=false \
  automatic_pose_initializer_enabled:=false \
  > "${LOG_DIR}/autoware.log" 2>&1 &
PIDS+=($!)
echo "       PID=$! log=${LOG_DIR}/autoware.log"

# 等 Autoware 起来(vector_map 出现算就绪)
echo "[INFO] Waiting for /map/vector_map (up to 60s)..."
for i in {1..60}; do
  if ros2 topic list --no-daemon 2>/dev/null | grep -q "/map/vector_map"; then
    echo "[INFO] Autoware ready after ${i}s"
    break
  fi
  sleep 1
  if [ "$i" -eq 60 ]; then
    echo "[ERROR] Autoware not ready in 60s. Check ${LOG_DIR}/autoware.log"
    exit 1
  fi
done

# ─── 2. 启动 gnss_ins_localizer ──────────────────────
echo "[INFO] [2/3] Launching gnss_ins_localizer..."
ros2 launch gnss_ins_localizer gnss_ins_localizer.launch.xml \
  map_path:="${MAP_PATH}" \
  input_gins:="${GINS_TOPIC}" \
  output_pose:=/localization/pose_estimator/pose_with_covariance \
  output_gnss_pose:=/sensing/gnss/pose_with_covariance \
  output_twist:=/localization/twist_estimator/twist_with_covariance \
  output_initialpose:=/initialpose3d \
  output_imu:=/sensing/imu/imu_data \
  > "${LOG_DIR}/gnss_ins.log" 2>&1 &
PIDS+=($!)
echo "       PID=$! log=${LOG_DIR}/gnss_ins.log"
sleep 5

# ─── 3. 启动 rosbag ──────────────────────────────────
if [ "$RECORD_BAG" = true ]; then
  BAG_DIR="${BAG_ROOT}/${TS}_bag"
  echo "[INFO] [3/3] Recording rosbag to ${BAG_DIR}"
  ros2 bag record -o "${BAG_DIR}" \
    "${GINS_TOPIC}" \
    /sensing/imu/imu_data \
    /localization/pose_estimator/pose_with_covariance \
    /localization/twist_estimator/twist_with_covariance \
    /localization/kinematic_state \
    /localization/acceleration \
    /localization/initialization_state \
    /initialpose3d \
    /planning/mission_planning/goal \
    /planning/mission_planning/route \
    /planning/scenario_planning/trajectory \
    /planning/scenario_planning/scenario \
    /control/command/control_cmd \
    /control/current_gate_mode \
    /system/operation_mode/state \
    /tf /tf_static \
    /map/vector_map \
    /diagnostics \
    /diagnostics_agg \
    > "${LOG_DIR}/rosbag.log" 2>&1 &
  PIDS+=($!)
  echo "       PID=$! log=${LOG_DIR}/rosbag.log"
else
  echo "[INFO] [3/3] rosbag recording SKIPPED (--no-bag)"
fi

# ─── 打印使用提示 ────────────────────────────────────
cat <<INFO

═══════════════════════════════════════════════════════════
  ✓ 所有进程已启动

  Log dir: ${LOG_DIR}
  Bag dir: ${BAG_DIR:-<disabled>}

  下一步(在新终端里执行):

    # ─── 1. Source 环境 ───
    source /opt/ros/humble/setup.bash
    source ~/autoware/install/setup.bash
    source ~/tsy/gnss_localizer_ws/install/setup.bash

    # ─── 2. 确认 GINS 有数据 ───
    ros2 topic hz /sensing/gins2000
    #   期望 ~100 Hz,按 Ctrl+C 停

    # ─── 3. 触发 pose 初始化 ───
    ros2 service call /gnss_ins_localizer/initialize std_srvs/srv/Trigger {}
    #   期望 success: true

    # ─── 4. 检查 engage 前置条件 ───
    bash ~/tsy/gnss_localizer_ws/scripts/check_engage.sh
    #   期望 is_autonomous_mode_available: true

    # ─── 5. RViz 里点 "2D Goal Pose" 设终点 ───

    # ─── 6. Engage ───
    ros2 service call /api/operation_mode/change_to_autonomous \\
        autoware_adapi_v1_msgs/srv/ChangeOperationMode {}
    #   期望 status.success: true

  按 Ctrl+C 停止所有进程。
═══════════════════════════════════════════════════════════

INFO

# ─── 等待任意子进程退出 ───────────────────────────────
wait -n
echo "[WARN] A child process exited. Triggering full shutdown."
