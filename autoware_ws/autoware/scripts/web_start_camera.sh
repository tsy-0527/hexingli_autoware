#!/usr/bin/env bash
set -uo pipefail

cd /home/tang/autoware/vision_detection_ws

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI

while true; do
  CAMERA_NODE_WAIT_SECONDS=180 ./run_camera_stream.sh
  exit_code=$?
  case "$exit_code" in
    0) exit 0 ;;
    3|4)
      echo "相机图像尚未就绪，10 秒后重新尝试启动推流..."
      sleep 10
      ;;
    *) exit "$exit_code" ;;
  esac
done

