#!/usr/bin/env bash

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "gnome-terminal is not installed. Open two terminals manually and run:"
  echo "  ${SCRIPT_DIR}/01_livox_driver.sh"
  echo "  ${SCRIPT_DIR}/02_fastlio_mapping.sh"
  exit 1
fi

gnome-terminal --title="FAST-LIO - Terminal 1 - Livox MID360S" -- bash -lc \
  "${SCRIPT_DIR}/01_livox_driver.sh; exec bash"
gnome-terminal --title="FAST-LIO - Terminal 2 - Mapping" -- bash -lc \
  "sleep 3; FASTLIO_RVIZ=true ${SCRIPT_DIR}/02_fastlio_mapping.sh; exec bash"

echo "Terminal 1 and Terminal 2 opened."
echo "After both are running, use a third terminal for:"
echo "  ${SCRIPT_DIR}/03_record_rosbag.sh"

