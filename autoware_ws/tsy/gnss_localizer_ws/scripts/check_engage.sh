#!/bin/bash
# ==============================================================
# engage 前置条件一键检查
# 用法: bash ~/tsy/gnss_localizer_ws/scripts/check_engage.sh
# ==============================================================

echo "======================================================"
echo "  Autoware Engage 前置条件检查"
echo "======================================================"

echo ""
echo "### [1] Operation Mode State ###"
echo "    (期望 is_autonomous_mode_available: true)"
timeout 3 ros2 topic echo /system/operation_mode/state --once 2>&1 | \
    grep -E "mode:|is_autonomous_mode_available|is_stop_mode_available" | sed 's/^/    /'

echo ""
echo "### [2] Initialization State ###"
echo "    (期望 state: 3, INITIALIZED)"
timeout 3 ros2 topic echo /localization/initialization_state --once 2>&1 | \
    grep -E "state:" | sed 's/^/    /'

echo ""
echo "### [3] 关键话题频率 ###"
for t in \
    /localization/kinematic_state \
    /localization/pose_estimator/pose_with_covariance \
    /localization/twist_estimator/twist_with_covariance \
    /planning/scenario_planning/trajectory \
    /control/command/control_cmd \
    /vehicle/status/velocity_status \
    /vehicle/status/steering_status \
    /vehicle/status/gear_status \
    /vehicle/status/control_mode ; do
    RATE=$(timeout 3 ros2 topic hz "$t" 2>&1 | grep "average rate" | head -1 | awk '{print $3}')
    if [ -z "$RATE" ]; then
        printf "    [!!] %-70s NO DATA\n" "$t"
    else
        printf "    [OK] %-70s %s Hz\n" "$t" "$RATE"
    fi
done

echo ""
echo "### [4] TF map->base_link ###"
timeout 3 ros2 run tf2_ros tf2_echo map base_link 2>&1 | head -6 | sed 's/^/    /'

echo ""
echo "### [5] Gate Mode ###"
echo "    (期望 data: 0 = AUTO)"
timeout 3 ros2 topic echo /control/current_gate_mode --once 2>&1 | \
    grep -E "data:" | sed 's/^/    /'

echo ""
echo "### [6] localizer 自检 ###"
ros2 service call /gnss_ins_localizer/check_mission_ready std_srvs/srv/Trigger {} 2>&1 | \
    grep -A 30 "message:" | sed 's/^/    /'

echo ""
echo "======================================================"
echo "  完成。如果所有项都 OK 且 is_autonomous_mode_available"
echo "  为 true,可以尝试:"
echo "    ros2 service call /api/operation_mode/change_to_autonomous \\"
echo "        autoware_adapi_v1_msgs/srv/ChangeOperationMode {}"
echo "======================================================"
