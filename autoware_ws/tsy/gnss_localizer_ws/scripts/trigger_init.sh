#!/bin/bash
set +e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

source /opt/ros/humble/setup.bash
if [[ -f /home/tang/autoware/install/local_setup.bash ]]; then
  source /home/tang/autoware/install/local_setup.bash
fi
source "${WORKSPACE_DIR}/install/local_setup.bash"

echo "[1/2] Publishing initial pose from GINS..."
init_response=$(ros2 service call /gnss_ins_localizer/initialize std_srvs/srv/Trigger {})
echo "${init_response}"

if [[ "${init_response}" != *"success=True"* ]]; then
  echo "[ERROR] Initial pose was not published; EKF activation skipped."
  exit 1
fi

echo "[2/2] Initializing Autoware localization state..."
api_response=$(ros2 service call \
  /api/localization/initialize \
  autoware_adapi_v1_msgs/srv/InitializeLocalization "{}")
echo "${api_response}"

if [[ "${api_response}" != *"success: true"* ]]; then
  echo "[ERROR] Autoware localization initialization failed; Auto remains unavailable."
  exit 1
fi
