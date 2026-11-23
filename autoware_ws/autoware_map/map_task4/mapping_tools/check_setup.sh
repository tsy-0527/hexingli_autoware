#!/usr/bin/env bash
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FASTLIO_ENV="${FASTLIO_ENV:-/home/tang/fastlio_ws/scripts/fastlio_env.sh}"
fail=0

ok() { printf '\033[32m✓ %s\033[0m\n' "$*"; }
warn() { printf '\033[33m! %s\033[0m\n' "$*"; }
bad() { printf '\033[31m✗ %s\033[0m\n' "$*"; fail=1; }

if [[ -r "$FASTLIO_ENV" ]]; then
  # shellcheck disable=SC1090
  set +u
  source "$FASTLIO_ENV"
  set -u
  ok "ROS/FAST-LIO 环境: $FASTLIO_ENV"
else
  bad "找不到环境脚本: $FASTLIO_ENV"
fi

for path in \
  /home/tang/fastlio_ws/scripts/01_livox_driver.sh \
  "$HERE/config/fastlio_mid360s_mapping.yaml" \
  "$HERE/config/postprocess.yaml" \
  /home/tang/livox_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360s_config.json; do
  [[ -r "$path" ]] && ok "文件存在: $path" || bad "文件缺失: $path"
done

for cmd in ros2 python3; do
  command -v "$cmd" >/dev/null 2>&1 && ok "命令可用: $cmd" || bad "命令不可用: $cmd"
done

if ros2 pkg prefix rosbag2_storage_mcap >/dev/null 2>&1; then
  ok "MCAP 存储插件已安装"
else
  warn "未安装 MCAP 插件，将回退 sqlite3"
fi

python3 - "$HERE" <<'PY' || fail=1
import os
import sys
import yaml

here = sys.argv[1]
with open(os.path.join(here, "config", "fastlio_mid360s_mapping.yaml"), encoding="utf-8") as f:
    fast = yaml.safe_load(f)["/**"]["ros__parameters"]
with open(os.path.join(here, "config", "postprocess.yaml"), encoding="utf-8") as f:
    post = yaml.safe_load(f)
ft = fast["mapping"]["extrinsic_T"]
fr = fast["mapping"]["extrinsic_R"]
pt = post["vehicle"]["extrinsic_T_body_lidar"]
pr = sum(post["vehicle"]["extrinsic_R_body_lidar"], [])
assert ft == pt and fr == pr, "FAST-LIO 与后处理外参不一致"
assert fast["publish"]["dense_publish_en"] is True, "dense_publish_en 必须为 true"
assert fast["preprocess"]["lidar_type"] == 4, "本机 PointCloud2 输入应使用 lidar_type=4"
print("\033[32m✓ YAML 可读取，话题/外参/稠密点云配置一致\033[0m")
PY

free_gb=$(df -BG --output=avail /home/tang/fastlio_ws | tail -1 | tr -dc '0-9')
if (( free_gb >= 60 )); then
  ok "录包磁盘可用 ${free_gb} GB（达到 60 GB 门槛）"
else
  warn "录包磁盘仅余 ${free_gb} GB；正式录制前至少准备 60 GB，或设置 BAG_ROOT 到大容量 SSD"
fi

if command -v CloudCompare >/dev/null 2>&1 || command -v cloudcompare >/dev/null 2>&1; then
  ok "CloudCompare 已安装"
else
  warn "未发现 CloudCompare；自动处理可运行，但人工修图前需安装"
fi

if command -v dufomap_run >/dev/null 2>&1 \
  || [[ -x /home/tang/dufomap/build/dufomap_run ]] \
  || [[ -x /home/tang/DUFOMap/build/dufomap_run ]] \
  || [[ -x "$HERE/../third_party/dufomap/build_gcc11/dufomap_run" ]]; then
  ok "DUFOMap 已找到"
else
  warn "未发现 dufomap_run；后处理会使用 raw_merged.pcd，动态物体去除能力会降低"
fi

if (( fail )); then
  echo "配置检查失败。"
  exit 1
fi
echo "配置文件检查通过。黄色项目请在正式录图前处理。"
