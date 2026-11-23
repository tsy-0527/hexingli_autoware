#!/bin/bash
# 一体化脚本:录制 → 提示 → 回放
# 前提: Autoware planning_simulator 已单独启动

source /opt/ros/humble/setup.bash
source ~/autoware/install/setup.bash
source ~/tsy/route_repeat_ws/install/setup.bash

# ========== 录制阶段 ==========
echo "=============================================================="
echo "[录制] 确认 Autoware 已启动"
echo "[录制] 在 RViz 里 2D Pose Estimate 放车, 2D Nav Goal 给终点, 点 AUTO"
echo "[录制] 车到终点后, 按 Ctrl+C 停止录制并保存"
echo "=============================================================="

ros2 launch route_teach_repeat record.launch.py

echo ""
echo "=============================================================="
echo "[录制] 完成! 已保存到 /home/tang/tsy/route_repeat_ws/recorded_route.yaml"
echo "=============================================================="

# ========== 提示回放 ==========
echo ""
read -p "是否开始回放? 先把车用 2D Pose Estimate 挪回起点, 然后输入 y 回车: " ANS

if [[ "$ANS" != "y" && "$ANS" != "Y" ]]; then
    echo "[退出] 未回放, 结束"
    exit 0
fi

# ========== 回放阶段 ==========
echo ""
echo "=============================================================="
echo "[回放] 发送 route..."
echo "[回放] 完成后在 RViz 点 AUTO/Engage 启动自动驾驶"
echo "[回放] Ctrl+C 退出"
echo "=============================================================="

ros2 service call /planning/mission_planning/mission_planner/clear_route autoware_planning_msgs/srv/ClearRoute
sleep 1
ros2 launch route_teach_repeat play.launch.py
