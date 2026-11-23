#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
for setup_file in \
  /home/tang/livox_ws/install/setup.bash \
  /home/tang/autoware_map/map_task4/indoor_localization/install/setup.bash; do
  if [[ -f "${setup_file}" ]]; then
    source "${setup_file}"
  fi
done

set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

if ip -brief address show eno2 2>/dev/null | grep -q '192\.168\.1\.50/24'; then
  echo "[OK] MID-360S 主机网卡 eno2 = 192.168.1.50/24"
else
  echo "[异常] eno2 未配置 MID-360S 驱动要求的 192.168.1.50/24"
  failed=1
fi

if ping -c 1 -W 1 192.168.1.149 >/dev/null 2>&1; then
  echo "[OK] MID-360S 雷达 192.168.1.149 可达"
else
  echo "[离线] MID-360S 雷达 192.168.1.149 不可达"
  failed=1
fi

required_files=(
  /home/tang/autoware_map/map_task4/indoor_localization/map_20261117_denoised/pointcloud_map.pcd
  /home/tang/autoware_map/map_task4/indoor_localization/map_20261117_denoised/localization_map.yaml
  /home/tang/autoware_map/map_task4/indoor_localization/map_20261117_denoised/nav_map.yaml
  /home/tang/autoware_map/map_task4/indoor_localization/map_20261117_denoised/speed_mask.yaml
  /home/tang/autoware_map/map_task4/indoor_localization/room_global_localizer/config/room_global_localizer.param.yaml
)

failed=${failed:-0}
for required_file in "${required_files[@]}"; do
  if [[ -r "${required_file}" ]]; then
    echo "[OK] 文件 ${required_file}"
  else
    echo "[缺失] 文件 ${required_file}"
    failed=1
  fi
done

if ros2 pkg prefix room_global_localizer >/dev/null 2>&1; then
  echo "[OK] ROS 包 room_global_localizer"
else
  echo "[缺失] ROS 包 room_global_localizer；请先执行 colcon build"
  failed=1
fi

topics="$(ros2 topic list 2>/dev/null || true)"

for topic in \
  /sensing/lidar/mid360s/pointcloud; do
  if grep -Fxq "${topic}" <<<"${topics}"; then
    echo "[OK] 话题 ${topic}"
  else
    echo "[离线] 话题 ${topic}"
    failed=1
  fi
done

if ros2 run tf2_ros tf2_echo base_link livox_front/livox_front_base_link 2>&1 | timeout 3 sed -n '1,14p'; then
  true
fi

if (( failed )); then
  echo "环境尚未完全就绪。若只有点云话题离线，请先运行 ./run_localizer.sh。"
  exit 1
fi

echo "环境检查通过。"
