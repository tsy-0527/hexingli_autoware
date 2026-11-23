#!/bin/bash
# 一键启动 planning_simulator + 回放节点
# 车挪回起点后, 按回车触发回放

source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
source ~/gins2000_ws/install/setup.bash
source ~/hunter_ws/install/setup.bash
source ~/autoware/install/setup.bash
source ~/tsy/route_repeat_ws/install/setup.bash

ROUTE_FILE="/home/tang/tsy/route_repeat_ws/recorded_route.yaml"

if [ ! -f "$ROUTE_FILE" ]; then
    echo "[run_play] ERROR: $ROUTE_FILE not found. Run ./run_record.sh first."
    exit 1
fi

echo "[run_play] Starting planning_simulator..."
ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=$HOME/autoware_map/map_task1 \
  vehicle_model:=hunter_vehicle \
  sensor_model:=sample_sensor_kit &
SIM_PID=$!

echo "[run_play] Waiting 10s for Autoware to boot..."
sleep 10

# 等待用户按键触发回放
echo ""
echo "=============================================================="
echo "[run_play] === 在 RViz 里用 2D Pose Estimate 把车放到起点附近 ==="
echo "[run_play] === 准备好后, 按 [回车] 触发回放 (Ctrl+C 退出) ==="
echo "=============================================================="
read -r

# 启动回放节点(前台)
echo "[run_play] Sending route..."
ros2 launch route_teach_repeat play.launch.py &
PLAY_PID=$!

sleep 3
echo ""
echo "[run_play] Route sent. 在 RViz 点 AUTO/Engage 启动自动驾驶"
echo "[run_play] 按 Ctrl+C 退出"

# 等待用户 Ctrl+C
wait $SIM_PID

# 清理
kill $PLAY_PID 2>/dev/null
