#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "$0")"
source /opt/ros/humble/setup.bash
python="${D455_PYTHON:-$PWD/.venv/bin/python}"
if [[ ! -x "$python" && -z "${D455_PYTHON:-}" ]]; then
  /usr/bin/python3 -m venv --system-site-packages .venv
fi
if [[ ! -x "$python" ]]; then echo 'D455_PYTHON must name a working Python 3.10 interpreter'; exit 1; fi
python="$(cd "$(dirname "$python")" && pwd)/$(basename "$python")"
"$python" -c 'import sys; assert sys.version_info[:2]==(3,10), "Use Ubuntu 22.04 system Python 3.10 or its venv"'
if [[ -z "${D455_PYTHON:-}" ]]; then
  "$python" -m pip install 'torch==2.7.1' 'torchvision==0.22.1' --index-url https://download.pytorch.org/whl/cpu
  "$python" -m pip install -r requirements.txt
fi
(cd src/d455_perception/runtime && "$python" -c 'import sys;sys.path.append("/usr/lib/python3/dist-packages");import rclpy,message_filters,sensor_msgs,torch,cv2;from bollard_detector import BollardDetector;print(BollardDetector("models/best.pt",imgsz=640).names)')
colcon build --packages-select d455_perception --cmake-args "-DDETECTOR_PYTHON=$python"
echo 'Ready. In this terminal run: source install/setup.bash && ros2 run d455_perception detect'
