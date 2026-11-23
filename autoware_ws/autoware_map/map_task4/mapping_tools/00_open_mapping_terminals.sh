#!/usr/bin/env bash
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DRIVER="${LIVOX_START_SCRIPT:-/home/tang/fastlio_ws/scripts/01_livox_driver.sh}"
GINS_DRIVER="${GINS_START_SCRIPT:-$HERE/01_gins2000_driver.sh}"

[[ -x "$DRIVER" ]] || { echo "找不到 Livox 启动脚本: $DRIVER" >&2; exit 1; }
[[ -x "$GINS_DRIVER" ]] || { echo "找不到 GINS2000 启动脚本: $GINS_DRIVER" >&2; exit 1; }
command -v gnome-terminal >/dev/null 2>&1 || {
  echo "未安装 gnome-terminal，请分别在三个终端运行："
  echo "  $DRIVER"
  echo "  $GINS_DRIVER"
  echo "  $HERE/02_fastlio_mapping_tools.sh"
  exit 1
}

gnome-terminal --title="Mapping 1/3 - Livox MID360S" -- bash -lc \
  "'$DRIVER'; exec bash"
gnome-terminal --title="Mapping 2/3 - GINS2000" -- bash -lc \
  "'$GINS_DRIVER'; exec bash"
gnome-terminal --title="Mapping 3/3 - FAST-LIO dense mapping" -- bash -lc \
  "sleep 3; '$HERE/02_fastlio_mapping_tools.sh'; exec bash"

echo "雷达、GINS2000 和建图终端已打开。RViz 与 GINS 数据稳定后，在第四个终端运行："
echo "  $HERE/record_mapping.sh <地图名> \"<备注>\""
