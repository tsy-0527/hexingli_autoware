#!/usr/bin/env python3
"""地图包离线验证。

检查项
  1. 车开过的轨迹在导航栅格上是否都是空闲（不能被占用/未知挡住）
  2. 站点 / 交接点是否落在空闲格
  3. （给了 --bag）抽帧把原始扫描按 T_old_to_new 放进地图，算内点率：
     内点率低 → 该处地图被修坏了 / 有残影 / 漂移

注意：这是地图本身的一致性检查。初始化仿真、门口规划、NDT 回放需要在工控机上用实际节点跑。

用法：python3 validate_map.py <地图包目录> [--bag <bag_dir>] [--config ...]
"""
import argparse
import json
import math
import os

import numpy as np
import yaml
from scipy.spatial import cKDTree

import common as C


def _grid_lookup(pkg, name):
    with open(os.path.join(pkg, f"{name}.yaml")) as f:
        y = yaml.safe_load(f)
    img = C.read_pgm(os.path.join(pkg, y["image"]))
    res = float(y["resolution"])
    ox, oy = y["origin"][:2]

    def look(xy):
        xy = np.atleast_2d(xy)
        ix = np.floor((xy[:, 0] - ox) / res).astype(int)
        iy = np.floor((xy[:, 1] - oy) / res).astype(int)
        r = img.shape[0] - 1 - iy
        ok = (ix >= 0) & (ix < img.shape[1]) & (r >= 0) & (r < img.shape[0])
        v = np.full(len(xy), -1)
        v[ok] = img[r[ok], ix[ok]]
        return v
    return look


def validate(pkg, cfg, bag=None):
    rep = {"checks": [], "ok": True}

    def add(name, ok, detail):
        rep["checks"].append({"name": name, "ok": bool(ok), "detail": detail})
        rep["ok"] &= bool(ok)

    look = _grid_lookup(pkg, "nav_grid")
    _, Tl = C.load_tum(os.path.join(pkg, "trajectory_leveled_tum.txt"))
    v = look(Tl[:, :2, 3])
    n_occ, n_unk = int(np.sum(v == 0)), int(np.sum(v == 205))
    bad_xy = Tl[(v == 0) | (v == 205), :2, 3]
    add("轨迹在导航栅格上可通行", n_occ == 0 and n_unk == 0,
        f"{len(v)} 个位姿中 占用 {n_occ}，未知 {n_unk}"
        + (f"；首个问题点 ({bad_xy[0,0]:.2f}, {bad_xy[0,1]:.2f})" if len(bad_xy) else ""))

    zp = os.path.join(pkg, "zones.yaml")
    if os.path.exists(zp):
        with open(zp, encoding="utf-8") as f:
            zones = yaml.safe_load(f) or {}
        pts = [(s["name"], s["x"], s["y"]) for k in ("stations", "handover") for s in (zones.get(k) or [])]
        if pts:
            vals = look(np.array([[p[1], p[2]] for p in pts]))
            bad = [p[0] for p, val in zip(pts, vals) if val != 254]
            add("站点/交接点落在空闲格", not bad, "全部空闲" if not bad else f"不在空闲格：{bad}")
        else:
            add("站点/交接点落在空闲格", True, "zones.yaml 未填站点（跳过）")

    if bag:
        vb = cfg["validate"]
        bc = cfg["bag2dufo"]
        xyz, _, _ = C.read_pcd(os.path.join(pkg, "map.pcd"))
        tree = cKDTree(xyz)
        T = np.loadtxt(os.path.join(pkg, "T_old_to_new.txt"))
        last, rows = -1e9, []
        for _, msg, _ in C.read_bag(bag, [bc["cloud_topic"]]):
            ts = C.stamp_sec(msg.header)
            if ts - last < vb["sample_every_sec"]:
                continue
            last = ts
            p, _ = C.cloud2_to_numpy(msg)
            if bc["cloud_frame"] != "world" or len(p) == 0:
                continue
            p = C.transform(T, p)
            p = p[p[:, 2] > 0.05]
            if len(p) > 20000:
                p = p[np.random.default_rng(0).choice(len(p), 20000, replace=False)]
            d, _ = tree.query(p, distance_upper_bound=vb["inlier_dist"], workers=-1)
            ratio = float(np.mean(np.isfinite(d)))
            rows.append((ts, float(p[:, 0].mean()), float(p[:, 1].mean()), ratio))
        if rows:
            r = np.array(rows)
            low = r[r[:, 3] < vb["warn_ratio"]]
            add("原始扫描与地图一致性", len(low) == 0,
                f"{len(r)} 帧，内点率 均值 {r[:,3].mean():.2f} 最低 {r[:,3].min():.2f}；"
                f"低于 {vb['warn_ratio']} 的 {len(low)} 帧"
                + ("" if not len(low) else "，位置（扫描质心）：" +
                   "; ".join(f"({x:.1f},{y:.1f})={q:.2f}" for _, x, y, q in low[:8])))
            rep["scan_inlier"] = [{"t": a, "x": b, "y": c, "ratio": d} for a, b, c, d in rows]

    with open(os.path.join(pkg, "reports", "validation.json"), "w") as f:
        json.dump(rep, f, indent=2, ensure_ascii=False)
    with open(os.path.join(pkg, "reports", "validation.md"), "w", encoding="utf-8") as f:
        f.write("# 地图包验证\n\n| 检查 | 结果 | 说明 |\n|---|---|---|\n")
        for c in rep["checks"]:
            f.write(f"| {c['name']} | {'✅' if c['ok'] else '❌'} | {c['detail']} |\n")
        f.write("\n仍需上车验证：初始化仿真、门口规划、bag 回放 NDT 健康。\n")
    for c in rep["checks"]:
        print(("  ✓ " if c["ok"] else "  ✗ ") + c["name"] + "：" + c["detail"])
    return rep


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pkg")
    ap.add_argument("--bag", default=None)
    ap.add_argument("--config", default=None)
    args = ap.parse_args()
    os.makedirs(os.path.join(args.pkg, "reports"), exist_ok=True)
    validate(args.pkg, C.load_config(args.config), args.bag)


if __name__ == "__main__":
    main()
