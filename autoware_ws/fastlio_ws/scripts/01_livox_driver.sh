#!/usr/bin/env bash

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/fastlio_env.sh"

LIVOX_SHARE_DIR="$(ros2 pkg prefix --share livox_ros_driver2)"
MID360S_CONFIG="${LIVOX_MID360S_CONFIG:-${LIVOX_SHARE_DIR}/config/MID360s_config.json}"

if [[ ! -r "${MID360S_CONFIG}" ]]; then
  echo "MID360S config is not readable: ${MID360S_CONFIG}" >&2
  exit 1
fi

echo "Starting Livox MID360S driver in PointCloud2 mode..."
# FAST-LIO's mid360.yaml uses lidar_type 4, whose subscriber expects
# sensor_msgs/msg/PointCloud2. The stock MID360S launch publishes CustomMsg,
# so start the same driver explicitly with xfer_format=0.
exec ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -r __node:=livox_lidar_publisher \
  -p xfer_format:=0 \
  -p multi_topic:=0 \
  -p data_src:=0 \
  -p publish_freq:=10.0 \
  -p output_data_type:=0 \
  -p frame_id:=livox_frame \
  -p lvx_file_path:=/home/livox/livox_test.lvx \
  -p user_config_path:="${MID360S_CONFIG}" \
  -p cmdline_input_bd_code:=livox0000000001

