#!/usr/bin/env bash

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/fastlio_env.sh"

BAG_ROOT="${FASTLIO_WS_DIR}/bags"
TMP_BAG="${BAG_ROOT}/.recording_$(date +%Y%m%d_%H%M%S)_$$"
FINAL_BAG=""
PCD_DIR="${PCD_DIR:-${FASTLIO_WS_DIR}/PCD}"

RECORD_TOPICS=(
  /livox/lidar
  /livox/imu
  /cloud_registered
  /cloud_registered_body
  /Odometry
  /path
  /tf
)

topic_ready() {
  ros2 topic type "$1" >/dev/null 2>&1
}

echo "Waiting for Livox MID360S and FAST-LIO topics..."
for _ in $(seq 1 30); do
  ready=true
  for topic in "${RECORD_TOPICS[@]}"; do
    if ! topic_ready "${topic}"; then
      ready=false
      break
    fi
  done
  if [[ "${ready}" == true ]]; then
    break
  fi
  sleep 1
done

for topic in "${RECORD_TOPICS[@]}"; do
  if ! topic_ready "${topic}"; then
    echo "Topic not available: ${topic}" >&2
    echo "Start scripts/01_livox_driver.sh and scripts/02_fastlio_mapping.sh first." >&2
    exit 1
  fi
done

lidar_topic_type="$(ros2 topic type /livox/lidar 2>/dev/null || true)"
if [[ "${lidar_topic_type}" != "sensor_msgs/msg/PointCloud2" ]]; then
  echo "Unexpected /livox/lidar type: ${lidar_topic_type:-not available}" >&2
  echo "FAST-LIO mid360.yaml requires sensor_msgs/msg/PointCloud2." >&2
  echo "Restart the driver with scripts/01_livox_driver.sh." >&2
  exit 1
fi

save_pcd() {
  echo "Saving PCD through /map_save..."
  if timeout 30s ros2 service call /map_save std_srvs/srv/Trigger "{}"; then
    local latest_pcd
    latest_pcd="$(ls -t "${PCD_DIR}"/*.pcd 2>/dev/null | head -n 1)"
    if [[ -n "${latest_pcd}" ]]; then
      echo "PCD saved to: ${latest_pcd}"
    else
      echo "Warning: /map_save returned, but no .pcd file found in: ${PCD_DIR}" >&2
    fi
  else
    echo "Warning: failed to call /map_save; FAST-LIO may have stopped." >&2
  fi
}

finish_recording() {
  status=$?
  trap - EXIT INT TERM
  save_pcd

  if [[ -d "${TMP_BAG}" ]]; then
    FINAL_BAG="${BAG_ROOT}/mid360s_$(date +%Y%m%d_%H%M%S)"
    suffix=1
    while [[ -e "${FINAL_BAG}" ]]; do
      FINAL_BAG="${BAG_ROOT}/mid360s_$(date +%Y%m%d_%H%M%S)_${suffix}"
      suffix=$((suffix + 1))
    done
    mv -- "${TMP_BAG}" "${FINAL_BAG}"
    echo "ROS bag saved to: ${FINAL_BAG}"
  fi

  exit "${status}"
}

trap finish_recording EXIT
trap 'exit 130' INT TERM

echo "Recording started. Press Ctrl+C to stop and rename by end time."
ros2 bag record -o "${TMP_BAG}" "${RECORD_TOPICS[@]}"
