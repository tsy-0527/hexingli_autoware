#!/usr/bin/env bash
# ROS 2 Humble's setup.bash reads optional variables that may be unset, so do not
# enable nounset until all ROS environments have been sourced.
set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
if [[ -f /home/tang/autoware/install/setup.bash ]]; then
  source /home/tang/autoware/install/setup.bash
fi
if [[ -f /home/tang/hunter_ws/install/setup.bash ]]; then
  source /home/tang/hunter_ws/install/setup.bash
fi

set -u

exec python3 "${script_dir}/cost_aware_path_proxy.py" "$@"
