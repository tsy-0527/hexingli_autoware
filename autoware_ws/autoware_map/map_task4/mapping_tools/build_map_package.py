#!/usr/bin/env python3
"""人工修图之后：生成地图包 v1.x。

  <out_root>/<name>_<version>/
    manifest.yaml                 版本、来源、坐标系、文件 sha256
    map.pcd                       3D 地图（调平，地面 z=0）          → NDT scan matcher
    nav_grid.pgm/.yaml            导航栅格 0.05 m，切层 0.05–2.0 m  → costmap 静态层
    loc_grid.pgm/.yaml            定位栅格（墙体轮廓）              → 房间全局初始化
    speed_mask.pgm/.yaml          限速掩膜                          → SpeedFilter
    speed_filter_info.yaml        costmap_filter_info_server 参数片段
    zones.yaml                    房间框 / 走廊区 / 站点 / 限速区 / 交接点
    T_old_to_new.txt              调平矩阵（FAST-LIO 世界系 → 地图系）
    trajectory_leveled_tum.txt    建图轨迹（地图系）
    reports/                      mapclean / 栅格 / 验证报告

用法：
  python3 build_map_package.py <修好的.pcd> <mapclean输出目录> <地图名> <版本 v1.0> \
      [--zones zones.yaml] [--out-root ~/maps] [--bag <bag_dir>] [--config ...]
"""
import argparse
import datetime
import hashlib
import json
import os
import shutil
import sys

import numpy as np
import yaml

import common as C
from make_grids import build_grids, write_grids
from validate_map import validate

HERE = os.path.dirname(os.path.abspath(__file__))


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def points_in_polygon(px, py, poly):
    poly = np.asarray(poly, float)
    inside = np.zeros(px.shape, bool)
    x0, y0 = poly[-1]
    for x1, y1 in poly:
        cond = (y1 > py) != (y0 > py)
        with np.errstate(divide="ignore", invalid="ignore"):
            xint = (x0 - x1) * (py - y1) / (y0 - y1) + x1
        inside ^= cond & (px < xint)
        x0, y0 = x1, y1
    return inside


def speed_value_to_pixel(v):
    """map_server scale 模式（free=0, occ=1, negate=0）：mask = rint((255-p)/255*100)"""
    for p in range(255, -1, -1):
        if int(np.rint((255 - p) / 255 * 100)) == v:
            return p
    raise ValueError(v)


def build_speed_mask(zones, origin, shape, res):
    H, W = shape
    mask = np.zeros(shape, np.int32)  # 0 = 不限速
    xs = origin[0] + (np.arange(W) + 0.5) * res
    ys = origin[1] + (np.arange(H) + 0.5) * res
    gx, gy = np.meshgrid(xs, ys)
    for z in zones or []:
        v = int(round(float(z["limit"]) * 100))
        if not 1 <= v <= 100:
            raise ValueError(f"限速区 {z.get('name')} 的 limit 必须在 0.01–1.0 m/s")
        inside = points_in_polygon(gx, gy, z["polygon"])
        cur = mask[inside]
        mask[inside] = np.where(cur == 0, v, np.minimum(cur, v))  # 重叠取最严
    lut = {v: speed_value_to_pixel(v) for v in np.unique(mask)}
    img = np.vectorize(lut.get)(mask).astype(np.uint8)
    return img[::-1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pcd", help="人工修好的 3D 地图（地图系）")
    ap.add_argument("clean_dir", help="mapclean 输出目录（取 T_old_to_new / 轨迹 / 报告）")
    ap.add_argument("name")
    ap.add_argument("version", help="例如 v1.0")
    ap.add_argument("--zones", default=None)
    ap.add_argument("--out-root", default=os.path.expanduser("~/maps"))
    ap.add_argument("--bag", default=None, help="原始 bag，给了就做扫描一致性验证")
    ap.add_argument("--config", default=None)
    ap.add_argument("--force", action="store_true")
    args = ap.parse_args()

    cfg = C.load_config(args.config)
    pkg = os.path.join(args.out_root, f"{args.name}_{args.version}")
    if os.path.exists(pkg):
        if not args.force:
            sys.exit(f"{pkg} 已存在。换地图要升版本号（整包更新），或加 --force 覆盖")
        shutil.rmtree(pkg)
    os.makedirs(os.path.join(pkg, "reports"))

    # 3D 地图
    xyz, fields, _ = C.read_pcd(args.pcd)
    inten = fields.get("intensity")
    C.write_pcd(os.path.join(pkg, "map.pcd"), xyz.astype(np.float32),
                {"intensity": inten.astype(np.float32)} if inten is not None else None)
    print(f"3D 地图 {len(xyz)} 点")

    for fn in ("T_old_to_new.txt", "trajectory_leveled_tum.txt"):
        shutil.copy(os.path.join(args.clean_dir, fn), pkg)
    for fn in ("mapclean_report.md", "mapclean_report.json"):
        p = os.path.join(args.clean_dir, fn)
        if os.path.exists(p):
            shutil.copy(p, os.path.join(pkg, "reports"))

    # 栅格（用修好的地图重新生成）
    _, Tl = C.load_tum(os.path.join(pkg, "trajectory_leveled_tum.txt"))
    nav, loc, origin, st = build_grids(xyz, Tl, cfg)
    write_grids(pkg, nav, loc, origin, cfg)
    with open(os.path.join(pkg, "reports", "grids.json"), "w") as f:
        json.dump(st, f, indent=2)
    print(f"栅格 {st['size_m'][0]} × {st['size_m'][1]} m，原点 {st['origin']}")

    # 区域与限速掩膜
    zsrc = args.zones or os.path.join(HERE, "config", "zones_template.yaml")
    shutil.copy(zsrc, os.path.join(pkg, "zones.yaml"))
    with open(zsrc, encoding="utf-8") as f:
        zones = yaml.safe_load(f) or {}
    if not args.zones:
        print("[提示] 没给 --zones，已放入空模板；填好房间/站点/限速区后用新版本号重新打包")
    res = cfg["grids"]["resolution"]
    smask = build_speed_mask(zones.get("speed_zones"), origin, nav.shape, res)
    C.write_pgm(os.path.join(pkg, "speed_mask.pgm"), smask)
    C.write_map_yaml(os.path.join(pkg, "speed_mask.yaml"), "speed_mask.pgm", res, origin,
                     mode="scale", occupied_thresh=1.0, free_thresh=0.0)
    with open(os.path.join(pkg, "speed_filter_info.yaml"), "w") as f:
        f.write("# costmap_filter_info_server 参数：type 2 = 绝对限速 (m/s)\n"
                "# speed_limit = base + multiplier * mask；mask 0 = 不限速\n"
                "costmap_filter_info_server:\n  ros__parameters:\n"
                "    type: 2\n    filter_info_topic: /costmap_filter_info\n"
                "    mask_topic: /speed_filter_mask\n    base: 0.0\n    multiplier: 0.01\n")

    # manifest
    T = np.loadtxt(os.path.join(pkg, "T_old_to_new.txt"))
    files = sorted(f for f in os.listdir(pkg) if os.path.isfile(os.path.join(pkg, f)))
    manifest = {
        "name": args.name,
        "version": args.version,
        "created": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "source": {"edited_pcd": os.path.abspath(args.pcd), "clean_dir": os.path.abspath(args.clean_dir),
                   "bag": os.path.abspath(args.bag) if args.bag else None},
        "frames": {
            "map": "地面 z=0，x/y 沿用 FAST-LIO 起点",
            "T_old_to_new": T.round(9).tolist(),
            "T_map_to_outdoor": None,   # 室内外对齐后填写（联合建图或门口控制点配准）
        },
        "grids": st,
        "files": {f: sha256(os.path.join(pkg, f)) for f in files},
        "note": "换地图 = 整包更新，坐标配置跟着换",
    }
    with open(os.path.join(pkg, "manifest.yaml"), "w", encoding="utf-8") as f:
        yaml.safe_dump(manifest, f, allow_unicode=True, sort_keys=False)

    print("验证 ...")
    rep = validate(pkg, cfg, args.bag)
    print(("✅ " if rep["ok"] else "⚠️  ") + f"地图包：{pkg}")


if __name__ == "__main__":
    main()
