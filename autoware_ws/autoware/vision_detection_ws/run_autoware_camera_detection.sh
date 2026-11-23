#!/usr/bin/env bash
set -eo pipefail

WS_ROOT="/home/tang/autoware/vision_detection_ws"

exec "${WS_ROOT}/run_realsense_detection.sh" \
  start_realsense:=false \
  image_topic:=/sensing/camera/camera0/image_rect_color \
  depth_topic:=/sensing/camera/camera0/aligned_depth_to_color/image_raw \
  camera_info_topic:=/sensing/camera/camera0/camera_info \
  "$@"
