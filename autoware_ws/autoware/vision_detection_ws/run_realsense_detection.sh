#!/usr/bin/env bash
# ROS 2 setup scripts reference optional variables that may be unset.
set -eo pipefail

WS_ROOT="/home/tang/autoware/vision_detection_ws"
TRT8_ROOT="${WS_ROOT}/third_party/tensorrt-8.5.2"

# The YOLOX engine in this workspace was built for TensorRT 8.5.
export LD_LIBRARY_PATH="${TRT8_ROOT}/usr/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"

source /opt/ros/humble/setup.bash
source /home/tang/autoware/install/setup.bash 2>/dev/null || true
source "${WS_ROOT}/install/setup.bash" 2>/dev/null || true

exec ros2 launch vision_detection_launch realsense_detection.launch.xml "$@"
