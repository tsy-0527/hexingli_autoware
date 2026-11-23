#!/usr/bin/env bash
# =============================================================================
# 离线分段建图 + 清理一键流程（本机 i7-12700F / MID360S / FAST-LIO）
#
# 用法：
#   ./clean_map_pipeline.sh <raw_bag_dir> <name> [选项]
#     --start SEC       段在原始 bag 中的起点（秒，默认 0）
#     --duration SEC    段时长（秒，默认 0=整包；墙钟=SEC/rate）
#     --rate R          回放倍率（默认 0.4）
#     --stop-sec SEC    bag2dufo 只取恢复段开头 SEC 秒（默认=duration）
#     --fastlio-config  默认 config/fastlio_mid360s_mapping_indoor.yaml
#     --post-config     默认 config/postprocess.yaml
#     --no-dufomap      跳过去动态（只用 raw_merged.pcd）
#
# 典型用法（一层主体，台阶前结束）：
#   ./clean_map_pipeline.sh /path/raw_bag task4_floor1 --duration 660 --rate 0.4
# 下层单独一段（从台阶后的静止点开始）：
#   ./clean_map_pipeline.sh /path/raw_bag task4_lower --start 680 --duration 300 --rate 0.4
# =============================================================================
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FASTLIO_ENV="${FASTLIO_ENV:-/home/tang/fastlio_ws/scripts/fastlio_env.sh}"
BAG_ROOT="${BAG_ROOT:-/home/tang/fastlio_ws/bags/mapping}"
DUFOMAP_BIN="${DUFOMAP_BIN:-$HERE/../third_party/dufomap/build_gcc11/dufomap_run}"
DUFOMAP_CFG="${DUFOMAP_CONFIG:-$HERE/../third_party/dufomap/assets/config.toml}"
FCFG="$HERE/config/fastlio_mid360s_mapping_indoor.yaml"
PCFG="$HERE/config/postprocess.yaml"
START=0
DURATION=0
RATE=0.4
STOP=""
USE_DUFO=1

usage() { sed -n '4,22p' "$0"; exit 1; }
[[ $# -ge 2 ]] || usage
RAW="${1%/}"
NAME="$2"
shift 2
while [[ $# -gt 0 ]]; do
  case "$1" in
    --start) START="$2"; shift 2;;
    --duration) DURATION="$2"; shift 2;;
    --rate) RATE="$2"; shift 2;;
    --stop-sec) STOP="$2"; shift 2;;
    --fastlio-config) FCFG="$2"; shift 2;;
    --post-config) PCFG="$2"; shift 2;;
    --no-dufomap) USE_DUFO=0; shift;;
    -h|--help) usage;;
    *) echo "未知参数：$1"; usage;;
  esac
done
[[ -f "$RAW/metadata.yaml" ]] || { echo "不是有效 bag 目录：$RAW"; exit 1; }
[[ "$NAME" =~ ^[A-Za-z0-9_]+$ ]] || { echo "name 只能用字母/数字/下划线"; exit 1; }
[[ -r "$FCFG" ]] || { echo "找不到 FAST-LIO 配置：$FCFG"; exit 1; }
[[ -r "$PCFG" ]] || { echo "找不到后处理配置：$PCFG"; exit 1; }
(( USE_DUFO == 0 )) || [[ -x "$DUFOMAP_BIN" ]] || { echo "找不到 dufomap_run：$DUFOMAP_BIN"; exit 1; }
mkdir -p "$BAG_ROOT"
FREE_GB=$(df -BG --output=avail "$BAG_ROOT" | tail -1 | tr -dc '0-9')
(( FREE_GB >= 20 )) || { echo "磁盘只剩 ${FREE_GB}GB，至少留 20GB"; exit 1; }

set +u
# shellcheck disable=SC1090
source "$FASTLIO_ENV"
set -u
STAMP=$(date +%Y%m%d_%H%M%S)
SESSION="$BAG_ROOT/${NAME}_${STAMP}"
mkdir -p "$SESSION/logs"
cp "$FCFG" "$SESSION/logs/fastlio_config_used.yaml"
cp "$PCFG" "$SESSION/logs/postprocess_config_used.yaml"
echo "会话：$SESSION"

echo "== [0/6] 检查原始 bag =="
ros2 bag info "$RAW" > "$SESSION/logs/raw_bag_info.txt" 2>&1
grep -q '/livox/lidar' "$SESSION/logs/raw_bag_info.txt" || { echo "原始 bag 缺 /livox/lidar"; exit 1; }
grep -q '/livox/imu' "$SESSION/logs/raw_bag_info.txt" || { echo "原始 bag 缺 /livox/imu"; exit 1; }
grep -q 'Duration' "$SESSION/logs/raw_bag_info.txt" && echo "  ✓ 原始 bag 含雷达+IMU"

echo "== [1/6] 离线重跑 FAST-LIO（start=$START duration=$DURATION rate=$RATE）=="
if [[ "$DURATION" != "0" ]]; then
  START_OFFSET="$START" PLAY_DURATION="$DURATION" FASTLIO_MAPPING_CFG="$FCFG" \
    "$HERE/recover_fastlio_from_bag.sh" "$RAW" "$SESSION/recovered_bag" "$RATE" \
    2>&1 | tee "$SESSION/logs/recover.log"
else
  START_OFFSET="$START" FASTLIO_MAPPING_CFG="$FCFG" \
    "$HERE/recover_fastlio_from_bag.sh" "$RAW" "$SESSION/recovered_bag" "$RATE" \
    2>&1 | tee "$SESSION/logs/recover.log"
fi
[[ -f "$SESSION/recovered_bag/metadata.yaml" ]] || { echo "恢复 bag 未生成"; exit 1; }
if [[ -z "$STOP" && "$DURATION" != "0" ]]; then STOP="$DURATION"; fi

echo "== [2/6] bag2dufo（关键帧 + 位姿）=="
mkdir -p "$SESSION/work"
B2DARGS=(--config "$PCFG")
[[ -n "$STOP" ]] && B2DARGS+=(--stop-sec "$STOP")
python3 -u "$HERE/bag2dufo.py" "$SESSION/recovered_bag" "$SESSION/work/dufo" \
  "${B2DARGS[@]}" 2>&1 | tee "$SESSION/logs/bag2dufo.log"

echo "== [3/6] DUFOMap 去动态 =="
STATIC="$SESSION/work/static_map.pcd"
if (( USE_DUFO )); then
  cp "$DUFOMAP_CFG" "$SESSION/logs/dufomap_config_used.toml"
  "$DUFOMAP_BIN" "$SESSION/work/dufo" "$DUFOMAP_CFG" 2>&1 | tee "$SESSION/logs/dufomap.log"
  OUT=$(ls -t "$SESSION/work/dufo"/*output*.pcd 2>/dev/null | head -1 || true)
  [[ -n "$OUT" ]] || { echo "DUFOMap 没有输出"; exit 1; }
  cp "$OUT" "$STATIC"
else
  cp "$SESSION/work/dufo/raw_merged.pcd" "$STATIC"
fi

echo "== [4/6] mapclean 调平/去噪/去残影 =="
python3 -u "$HERE/mapclean.py" "$STATIC" "$SESSION/work/dufo/poses_lidar_tum.txt" \
  "$SESSION/work/clean" --config "$PCFG" 2>&1 | tee "$SESSION/logs/mapclean.log"

echo "== [5/6] 2D 栅格 + 0.1 m NDT 图 + 预览 =="
python3 -u "$HERE/make_grids.py" "$SESSION/work/clean/map_leveled.pcd" \
  "$SESSION/work/clean/trajectory_leveled_tum.txt" "$SESSION/work/clean/grids_preview" \
  --config "$PCFG" 2>&1 | tee "$SESSION/logs/grids.log"
python3 -u "$HERE/export_clean_map.py" "$SESSION" 2>&1 | tee "$SESSION/logs/export.log"

echo "== [6/6] 质量门控 =="
python3 "$HERE/check_map_quality.py" "$SESSION" 2>&1 | tee "$SESSION/logs/quality.log"

cat <<EOF

完成：$SESSION
  3D 地图      $SESSION/work/clean/map_leveled.pcd
  NDT 0.1 m    $SESSION/work/clean/map_leveled_0.1.pcd
  俯视预览     $SESSION/work/clean/map_preview.png
  栅格+轨迹    $SESSION/work/clean/nav_grid_overlay.png
  清理报告     $SESSION/work/clean/mapclean_report.md
  质量报告     $SESSION/work/clean/quality_report.md
  可疑点       $SESSION/work/clean/suspicious.pcd
EOF
