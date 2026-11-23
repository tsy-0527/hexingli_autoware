#!/usr/bin/env bash

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/fastlio_env.sh"

FASTLIO_CONFIG_FILE="${FASTLIO_CONFIG_FILE:-mid360.yaml}"
FASTLIO_RVIZ="${FASTLIO_RVIZ:-true}"

# MID360S keeps the same PointCloud2/IMU topics and FAST-LIO preprocessing
# parameters as MID360, so the existing mid360.yaml remains applicable.
echo "Starting FAST-LIO for MID360S with ${FASTLIO_CONFIG_FILE}..."
exec ros2 launch fast_lio mapping.launch.py \
  "config_file:=${FASTLIO_CONFIG_FILE}" \
  "rviz:=${FASTLIO_RVIZ}"

