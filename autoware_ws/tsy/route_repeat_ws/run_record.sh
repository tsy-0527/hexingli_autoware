#!/bin/bash
# 一键启动 planning_simulator + 录制节点
# Ctrl+C 停止录制并保存 yaml

source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
source ~/gins2000_ws/install/setup.bash
source ~/hunter_ws/install/setup.bash
source ~/autoware/install/setup.bash
source ~/tsy/route_repeat_ws/install/setup.bash

# 在后台启动 planning_simulator
echo "[run_record] Starting planning_simulator..."
ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=$HOME/autoware_map/map_task1 \
  vehicle_model:=hunter_vehicle \
  sensor_model:=sample_sensor_kit &
SIM_PID=$!

# 等 Autoware 加载好
echo "[run_record] Waiting 10s for Autoware to boot..."
sleep 10

# 前台启动录制节点(Ctrl+C 会先停录制,保存 yaml)
echo "[run_record] Starting route recorder..."
echo "[run_record] === 在 RViz 里 2D Pose Estimate 放车, 2D Nav Goal 给终点, 点 AUTO ==="
echo "[run_record] === 车到终点后, 按 Ctrl+C 停止录制并保存 ==="
ros2 launch route_teach_repeat record.launch.py

# 录制结束后关掉仿真
echo "[run_record] Recorder stopped. Route saved to /home/tang/tsy/route_repeat_ws/recorded_route.yaml"
echo "[run_record] Killing planning_simulator..."
kill $SIM_PID 2>/dev/null
wait $SIM_PID 2>/dev/null

echo "[run_record] Done. Run ./run_play.sh to replay."
