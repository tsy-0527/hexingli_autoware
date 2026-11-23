#!/bin/bash
# 实车录制/回放脚本
# 前提: Autoware(带 NDT 雷达定位)已单独启动

source /opt/ros/humble/setup.bash
source ~/autoware/install/setup.bash
source ~/tsy/route_repeat_ws/install/setup.bash

ROUTE_FILE="/home/tang/tsy/route_repeat_ws/recorded_route.yaml"

# ========== 起飞前检查 ==========
echo ""
echo "=============================================================="
echo "  实车 route teach-and-repeat"
echo "=============================================================="
echo ""
read -p "是否运行起飞前检查? [Y/n]: " DO_CHECK
if [[ "$DO_CHECK" != "n" && "$DO_CHECK" != "N" ]]; then
    ~/tsy/route_repeat_ws/preflight_check.sh
    if [ $? -ne 0 ]; then
        read -p "检查未完全通过,是否仍要继续? [y/N]: " GO
        if [[ "$GO" != "y" && "$GO" != "Y" ]]; then
            echo "已退出"
            exit 1
        fi
    fi
fi

# ========== 选择模式 ==========
echo ""
echo "=============================================================="
echo "  选择模式:"
echo "    1) 录制新路径"
echo "    2) 只回放已有路径 (需要 $ROUTE_FILE 存在)"
echo "    3) 退出"
echo "=============================================================="
read -p "请选择 [1/2/3]: " MODE

case "$MODE" in
    1)
        MODE_RECORD=1
        ;;
    2)
        MODE_RECORD=0
        if [ ! -f "$ROUTE_FILE" ]; then
            echo "错误: $ROUTE_FILE 不存在,请先录制"
            exit 1
        fi
        ;;
    *)
        echo "已退出"
        exit 0
        ;;
esac

# ========== 录制阶段 ==========
if [ "$MODE_RECORD" = "1" ]; then
    echo ""
    echo "=============================================================="
    echo "  录制前准备:"
    echo "  1. 确认 NDT 雷达定位已完成初始化且匹配质量稳定"
    echo "  2. 确认 RViz 里车图标位置和朝向都正确"
    echo "  3. 车辆开到路径起点"
    echo "=============================================================="
    read -p "准备好后按回车开始录制..."
    
    # 备份旧 yaml
    if [ -f "$ROUTE_FILE" ]; then
        BAK="${ROUTE_FILE%.yaml}_$(date +%Y%m%d_%H%M%S).yaml.bak"
        if ! cp "$ROUTE_FILE" "$BAK"; then
            echo "错误: 旧路线备份失败,不会删除原文件"
            exit 1
        fi
        echo "旧文件备份到: $BAK"
    fi
    # 防止录制失败时误把旧 YAML 当成新录制结果。
    rm -f "$ROUTE_FILE"
    
    echo ""
    echo "▶ 录制中... 遥控器开车走完全程,到终点后按 Ctrl+C 保存"
    echo "   录制终端每 2 秒会显示 poses、segments 和 last_pose"
    echo "   如果持续显示 no pose received，说明定位没有传到 route_recorder"
    echo ""
    ros2 launch route_teach_repeat record.launch.py
    
    if [ ! -f "$ROUTE_FILE" ]; then
        echo "错误: yaml 未生成,录制可能失败"
        exit 1
    fi
    
    SEG_COUNT=$(awk '/preferred:/{count++} END{print count+0}' "$ROUTE_FILE")
    echo ""
    echo "=============================================================="
    echo "  录制完成! 共 $SEG_COUNT 段"
    echo "  文件: $ROUTE_FILE"
    echo "=============================================================="
    
    if [ "$SEG_COUNT" -lt 1 ]; then
        echo "错误: 没录到任何 segment,不能回放"
        exit 1
    fi
    
    echo ""
    read -p "是否立即回放? 请先把车挪回起点,输入 y 回车: " ANS
    if [[ "$ANS" != "y" && "$ANS" != "Y" ]]; then
        echo "退出,后续可用 mode 2 单独回放"
        exit 0
    fi
fi

# ========== 回放阶段 ==========
echo ""
echo "▸ 清除旧 route..."
ros2 service call /planning/mission_planning/mission_planner/clear_route \
    autoware_planning_msgs/srv/ClearRoute 2>/dev/null | grep -q "success" && echo "  ✓ route 已清除" || echo "  ! 清除失败(可能本来就没有,继续)"

sleep 1

echo ""
echo "=============================================================="
echo "  回放前最后检查:"
echo "  1. 车已在录制起点附近(RViz 观察)"
echo "  2. NDT 雷达定位稳定,车图标位置正确"
echo "  3. 遥控器急停在手边"
echo "  4. 车辆前方无人无障碍"
echo "=============================================================="
read -p "确认无误后按回车发送 route..."

echo ""
echo "▸ 发送 route..."
ros2 launch route_teach_repeat play.launch.py &
PLAY_PID=$!

sleep 3

echo ""
echo "=============================================================="
echo "  Route 已发送. 在 RViz 里操作:"
echo "  1. 确认规划出的粗线路径合理"
echo "  2. 准备好后, 点 AUTO / Engage 启动"
echo "  3. 车启动后,手一直放在急停按钮上!"
echo ""
echo "  Ctrl+C 结束"
echo "=============================================================="

wait $PLAY_PID
