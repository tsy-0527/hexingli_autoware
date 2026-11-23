#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ROS 2 Humble's setup.bash may read optional unset variables.
source /opt/ros/humble/setup.bash
if [[ -f /home/tang/autoware/install/setup.bash ]]; then
  source /home/tang/autoware/install/setup.bash
fi

exec rviz2 -d "${script_dir}/../config/cost_aware_path_comparison.rviz" "$@"
