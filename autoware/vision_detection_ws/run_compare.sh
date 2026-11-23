#!/usr/bin/env bash
set -e

TRT8_ROOT="/home/tang/autoware/vision_detection_ws/third_party/tensorrt-8.5.2"
export LD_LIBRARY_PATH="${TRT8_ROOT}/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"

source /opt/ros/humble/setup.bash
source /home/tang/autoware/install/setup.bash
source /home/tang/autoware/vision_detection_ws/install/setup.bash

exec ros2 launch vision_detection_launch compare_yolox.launch.xml \
  video_device:="${1:-/dev/video0}"
