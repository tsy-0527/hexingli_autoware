#!/usr/bin/env bash
# =============================================================================
# 建图后处理（自动部分）：bag2dufo → DUFOMap → mapclean → 栅格预览
#
# 用法：./postprocess.sh <bag_dir> <work_dir> [config.yaml]
#
# 环境变量：
#   DUFOMAP_BIN     dufomap_run 可执行文件（默认使用本项目 third_party 中的本地构建）
#   DUFOMAP_CONFIG  DUFOMap 配置（默认 DUFOMap 仓库的 assets/config.toml）
#   未设置 DUFOMAP_BIN 时跳过去动态，直接用未去动态的整图（只适合场地清空时）
#
# 完成后：CloudCompare 修 <work>/clean/map_leveled.pcd → build_map_package.py
# =============================================================================
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
FASTLIO_ENV="${FASTLIO_ENV:-/home/tang/fastlio_ws/scripts/fastlio_env.sh}"
if [[ -r "$FASTLIO_ENV" ]]; then
  # rosbag2_py 及消息类型需要 ROS 环境。
  # shellcheck disable=SC1090
  set +u
  source "$FASTLIO_ENV"
  set -u
fi
LOCAL_DUFOMAP="$HERE/../third_party/dufomap/build_gcc11/dufomap_run"
if [[ -z "${DUFOMAP_BIN:-}" && -x "$LOCAL_DUFOMAP" ]]; then
  DUFOMAP_BIN="$LOCAL_DUFOMAP"
fi
BAG="${1:?用法: $0 <bag_dir> <work_dir> [config.yaml]}"
WORK="${2:?}"
CFG="${3:-$HERE/config/postprocess.yaml}"

mkdir -p "$WORK"
cp "$CFG" "$WORK/postprocess_used.yaml"
PY="python3 -u"

echo "== [1/4] bag2dufo"
$PY "$HERE/bag2dufo.py" "$BAG" "$WORK/dufo" --config "$CFG" 2>&1 | tee "$WORK/bag2dufo.log"

echo "== [2/4] DUFOMap 去动态"
STATIC="$WORK/static_map.pcd"
if [[ -n "${DUFOMAP_BIN:-}" && -x "$DUFOMAP_BIN" ]]; then
  DCFG="${DUFOMAP_CONFIG:-$(dirname "$(dirname "$DUFOMAP_BIN")")/assets/config.toml}"
  [[ -f "$DCFG" ]] || { echo "找不到 DUFOMap 配置 $DCFG，设置 DUFOMAP_CONFIG"; exit 1; }
  cp "$DCFG" "$WORK/dufomap_config_used.toml"
  ( time "$DUFOMAP_BIN" "$WORK/dufo" "$DCFG" ) 2>&1 | tee "$WORK/dufomap.log"
  OUT=$(ls -t "$WORK"/dufo/*output*.pcd 2>/dev/null | head -1 || true)
  [[ -n "$OUT" ]] || { echo "DUFOMap 没有输出 *output*.pcd，看 $WORK/dufomap.log"; exit 1; }
  cp "$OUT" "$STATIC"
else
  echo "  [警告] 未设置 DUFOMAP_BIN，跳过去动态，使用 raw_merged.pcd"
  cp "$WORK/dufo/raw_merged.pcd" "$STATIC"
fi

echo "== [3/4] mapclean"
$PY "$HERE/mapclean.py" "$STATIC" "$WORK/dufo/poses_lidar_tum.txt" "$WORK/clean" --config "$CFG" \
  2>&1 | tee "$WORK/mapclean.log"

echo "== [4/4] 栅格预览（人工修图前，看哪里要修）"
$PY "$HERE/make_grids.py" "$WORK/clean/map_leveled.pcd" "$WORK/clean/trajectory_leveled_tum.txt" \
  "$WORK/clean/grids_preview" --config "$CFG"

cat <<EOF

完成。接下来人工修图：
  CloudCompare 打开
    $WORK/clean/map_leveled.pcd      （要修的图）
    $WORK/clean/suspicious.pcd       （被自动删掉的点，按 label 着色复核）
    $WORK/clean/grids_preview/nav_grid.pgm  （2D 预览，找漏洞/残影）
  修完另存为 PCD，然后：
    python3 $HERE/build_map_package.py <修好的.pcd> $WORK/clean <地图名> v1.0 \\
        --zones <zones.yaml> --bag $BAG
EOF
