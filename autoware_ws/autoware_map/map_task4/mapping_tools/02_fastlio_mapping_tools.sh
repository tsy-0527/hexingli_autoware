#!/usr/bin/env bash
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FASTLIO_ENV="${FASTLIO_ENV:-/home/tang/fastlio_ws/scripts/fastlio_env.sh}"
[[ -r "$FASTLIO_ENV" ]] || { echo "找不到环境脚本: $FASTLIO_ENV" >&2; exit 1; }
# shellcheck disable=SC1090
set +u
source "$FASTLIO_ENV"
set -u

CFG="${FASTLIO_MAPPING_CFG:-$HERE/config/fastlio_mid360s_mapping.yaml}"
[[ -r "$CFG" ]] || { echo "找不到 FAST-LIO 配置: $CFG" >&2; exit 1; }

echo "启动建图专用 FAST-LIO: $CFG"
exec ros2 launch fast_lio mapping.launch.py \
  "config_path:=$(dirname "$CFG")" \
  "config_file:=$(basename "$CFG")" \
  "rviz:=${FASTLIO_RVIZ:-true}"
