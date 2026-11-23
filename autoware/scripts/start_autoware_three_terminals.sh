#!/usr/bin/env bash

set -euo pipefail

CAMERA_START_DELAY="${CAMERA_START_DELAY:-20}"
VELOCITY_LIMIT_DELAY="${VELOCITY_LIMIT_DELAY:-60}"
TMUX_SESSION_NAME="${TMUX_SESSION_NAME:-autoware}"

if ! command -v tmux >/dev/null 2>&1; then
  echo "错误：未找到 tmux，请先执行：sudo apt install tmux" >&2
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

if [[ -z "${TMUX_SESSION_NAME}" ]]; then
  echo "错误：TMUX_SESSION_NAME 不能为空。" >&2
  exit 1
fi

if tmux has-session -t "${TMUX_SESSION_NAME}" 2>/dev/null; then
  echo "错误：tmux 会话 ${TMUX_SESSION_NAME} 已存在，Autoware 可能已经启动。" >&2
  echo "查看会话：tmux attach -t ${TMUX_SESSION_NAME}" >&2
  echo "停止会话：tmux kill-session -t ${TMUX_SESSION_NAME}" >&2
  exit 1
fi

export CAMERA_START_DELAY
export VELOCITY_LIMIT_DELAY

first_window=1

open_tmux_window() {
  local window_name="$1"
  local command="$2"
  local payload

  payload="${command}
exit_code=\$?
echo
if (( exit_code == 0 )); then
  echo '[进程已正常退出，tmux 窗口保持打开]'
else
  echo \"[进程已退出，状态码：\${exit_code}，tmux 窗口保持打开]\"
fi
exec bash"

  # 使用交互式登录 shell，保留与手动打开终端一致的运行环境
  #（Cyclone DDS、ACADOS、CUDA 等）。
  if (( first_window )); then
    tmux new-session -d \
      -s "${TMUX_SESSION_NAME}" \
      -n "${window_name}" \
      -c "${HOME}" \
      -- bash --login -i -c "${payload}"
    first_window=0
  else
    tmux new-window -d \
      -t "${TMUX_SESSION_NAME}:" \
      -n "${window_name}" \
      -c "${HOME}" \
      -- bash --login -i -c "${payload}"
  fi
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
echo "等待 ${CAMERA_START_DELAY} 秒后启动摄像头推流..."
sleep "${CAMERA_START_DELAY}"

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

udp_pose_relay_command='
cd "$HOME/tsy"

python3 ros_udp_pose_relay.py --ros-args \
  -p target_port:=15000 \
  -p max_rate_hz:=0.0
'

echo "正在启动 tmux 窗口 1：Autoware..."
open_tmux_window "Autoware" "${autoware_command}"

echo "正在启动 tmux 窗口 2：Rosbridge..."
open_tmux_window "Rosbridge-9090" "${rosbridge_command}"

echo "正在启动 tmux 窗口 3：Camera Stream（延时 ${CAMERA_START_DELAY} 秒）..."
open_tmux_window "Camera-Stream" "${camera_stream_command}"

echo "正在启动 tmux 窗口 4：UDP Pose Relay..."
open_tmux_window "UDP-Pose-Relay" "${udp_pose_relay_command}"

echo "四个进程均已在后台 tmux 会话中启动。"
echo "查看运行输出：tmux attach -t ${TMUX_SESSION_NAME}"
echo "退出查看但保持运行：按 Ctrl+B，再按 D"
echo "停止全部进程：tmux kill-session -t ${TMUX_SESSION_NAME}"
