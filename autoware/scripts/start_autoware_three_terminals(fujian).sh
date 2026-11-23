#!/usr/bin/env bash

set -euo pipefail

CAMERA_START_DELAY="${CAMERA_START_DELAY:-20}"
VELOCITY_LIMIT_DELAY="${VELOCITY_LIMIT_DELAY:-60}"

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "错误：未找到 gnome-terminal。" >&2
  exit 1
fi

if [[ ! "${CAMERA_START_DELAY}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "错误：CAMERA_START_DELAY 必须是非负秒数，当前值：${CAMERA_START_DELAY}" >&2
  exit 1
fi

if [[ ! "${VELOCITY_LIMIT_DELAY}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "错误：VELOCITY_LIMIT_DELAY 必须是非负秒数，当前值：${VELOCITY_LIMIT_DELAY}" >&2
  exit 1
fi

export VELOCITY_LIMIT_DELAY

open_terminal() {
  local title="$1"
  local command="$2"
  local payload

  payload="${command}
exit_code=\$?
echo
if (( exit_code == 0 )); then
  echo '[进程已正常退出，终端保持打开]'
else
  echo \"[进程已退出，状态码：\${exit_code}，终端保持打开]\"
fi
exec bash"

  # Use an interactive login shell so the terminal gets the same runtime
  # environment as a manually opened terminal (Cyclone DDS, ACADOS, CUDA, etc.).
  gnome-terminal --title="${title}" -- bash --login -i -c "${payload}"
}

autoware_command='
source /opt/ros/humble/setup.bash
source "$HOME/livox_ws/install/setup.bash"
source "$HOME/gins2000_ws/install/setup.bash"
source "$HOME/yunle_chassis-main/install/setup.bash"
source "$HOME/autoware/install/setup.bash"
source "$HOME/tsy/gnss_localizer_ws/install/setup.bash"

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI

# Autoware 启动后延时下发限速；若订阅端尚未出现，继续等待订阅端。
(
  echo "Autoware 启动中，将在 ${VELOCITY_LIMIT_DELAY} 秒后下发最大速度..."
  sleep "${VELOCITY_LIMIT_DELAY}"
  echo "开始设置最大速度为 1.39 m/s；若速度规划器尚未就绪，将继续等待..."

  ros2 topic pub --once \
    --wait-matching-subscriptions 1 \
    --keep-alive 2 \
    --qos-reliability reliable \
    --qos-durability transient_local \
    /planning/scenario_planning/max_velocity_default \
    autoware_internal_planning_msgs/msg/VelocityLimit \
    "{stamp: now, max_velocity: 1.39, use_constraints: false}"

  echo "最大速度设置完成：1.39 m/s（约 5 km/h）"
) &

ros2 launch autoware_launch autoware.launch.xml \
  map_path:="$HOME/autoware_map/map_task1" \
  vehicle_model:=yunle_vehicle \
  sensor_model:=livox_sensor_kit \
  pose_source:=gnss-ins \
  twist_source:=gnss-ins \
  gnss_enabled:=true \
  automatic_pose_initializer_enabled:=false \
  use_sim_time:=false \
  system_run_mode:=online
'

rosbridge_command='
source /opt/ros/humble/setup.bash
source "$HOME/livox_ws/install/setup.bash"
source "$HOME/gins2000_ws/install/setup.bash"
source "$HOME/hunter_ws/install/setup.bash"
source "$HOME/autoware/install/setup.bash"
source "$HOME/yunle_chassis-main/install/setup.bash"

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI

ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  address:=0.0.0.0 \
  port:=9090
'

camera_stream_command='
cd /home/tang/autoware/vision_detection_ws

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI

while true; do
  CAMERA_NODE_WAIT_SECONDS=180 ./run_camera_stream.sh
  exit_code=$?

  case "${exit_code}" in
    0)
      break
      ;;
    3|4)
      echo "相机图像尚未就绪，10 秒后重新尝试启动推流..."
      sleep 10
      ;;
    *)
      echo "Camera Stream 发生不可重试错误，状态码：${exit_code}"
      exit "${exit_code}"
      ;;
  esac
done
'

echo "正在启动终端 1：Autoware..."
open_terminal "Autoware" "${autoware_command}"

echo "正在启动终端 2：Rosbridge..."
open_terminal "Rosbridge :9090" "${rosbridge_command}"

echo "Autoware 终端已打开；等待 ${CAMERA_START_DELAY} 秒后启动摄像头推流..."
sleep "${CAMERA_START_DELAY}"

echo "正在启动终端 3：Camera Stream..."
open_terminal "Camera Stream" "${camera_stream_command}"

echo "三个终端均已启动。"
