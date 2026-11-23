#!/bin/bash
# 一键健康检查:替代手敲的服务调用和 topic hz
set +e

source /opt/ros/humble/setup.bash
source ~/autoware/install/setup.bash
source ~/tsy/gnss_localizer_ws/install/setup.bash

echo "=== [1] Topic frequencies (3s each) ==="
for t in \
  /sensing/gins2000 \
  /localization/pose_estimator/pose_with_covariance \
  /localization/twist_estimator/twist_with_covariance \
  /localization/kinematic_state \
  ; do
  hz=$(timeout 3 ros2 topic hz "$t" 2>&1 | grep "average rate" | tail -1 | awk '{print $3}')
  printf "  %-60s %s Hz\n" "$t" "${hz:-N/A}"
done

echo ""
echo "=== [2] GINS ins_status ==="
timeout 2 ros2 topic echo "/sensing/gins2000" --once --qos-reliability best_effort \
  2>/dev/null | grep -E "ins_status|latitude|longitude" | head -3

echo ""
echo "=== [3] check_mission_ready ==="
ros2 service call /gnss_ins_localizer/check_mission_ready std_srvs/srv/Trigger {} 2>&1 \
  | grep -A 100 "message:" | head -20
