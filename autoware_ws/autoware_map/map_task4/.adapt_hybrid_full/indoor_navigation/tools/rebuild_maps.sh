#!/usr/bin/env bash
# 重新生成 map_20261117_legacy_indoor 下的 nav_map / localization_map / speed_mask / keepout_mask。
# 在仓库根目录运行：bash indoor_navigation/tools/rebuild_maps.sh
set -euo pipefail
D=map_20261117_legacy_indoor

# 门洞、坡道通道强制可通行；路沿坎、坡道栏杆、西侧平台边、南侧路沿画成障碍（离地高度只有 0.1–0.2 m，点云阈值抓不到）
python3 indoor_navigation/tools/register_legacy_grids.py "$D/legacy_room_registration.yaml" "$D" \
  --nav map/indoor_nav_2d.yaml --loc map/indoor_loc_2d.yaml \
  --speed indoor_navigation/maps/speed_mask.yaml \
  --apron "$D/door_apron.yaml" \
  --force-free \
    26.3,-71.4,27.5,-70.2 \
    17.7,-75.35,24.0,-74.12 \
  --speed-zone \
    24.5,-78.6,31.5,-67.5,0.20 \
    12.0,-80.5,24.5,-73.8,0.20 \
  --obstacle-line \
    19.0,-75.5,23.6,-75.3,27.5,-75.05,33.5,-74.8,39.5,-74.45,41.8,-74.4,41.8,-70.0 \
    17.3,-74.3,18.6,-74.05,23.5,-73.9 \
    6.0,-76.05,11.0,-76.0,17.35,-75.6 \
    6.0,-84.0,46.0,-82.3
#   ^ 1 南侧栏杆 + 门前路沿坎（0.15–0.22 m）+ 东端拐向北的路沿
#     2 坡道北侧栏杆
#     3 坡道西侧平台边（高出路面约 0.1 m）
#     4 主路南侧路沿

python3 indoor_navigation/tools/make_keepout_mask.py "$D/nav_map.yaml" "$D"
