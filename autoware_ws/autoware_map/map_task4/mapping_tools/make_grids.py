#!/usr/bin/env python3
"""从调平后的 3D 地图生成 2D 栅格（Nav2 map_server 格式）。

  nav_grid.pgm/.yaml   导航栅格：切层 nav_slice（默认离地 0.05–2.0 m）
  loc_grid.pgm/.yaml   定位栅格：切层 loc_slice（默认 1.2–2.5 m，墙体轮廓，给房间全局初始化用）

像素：0=占用  254=空闲  205=未知；free_thresh 0.196 恰好让 205 判为未知。
空闲判定：从轨迹位姿做 2D 射线投射到障碍为止，并要求附近见过地面（可关）；车开过的地方一定空闲。

用法：python3 make_grids.py <map_leveled.pcd> <trajectory_leveled_tum.txt> <out_dir> [--config ...]
"""
import argparse
import math
import os

import numpy as np
from scipy import ndimage

import common as C

OCC, FREE, UNKNOWN = 0, 254, 205


def _cells(xy, origin, res, shape):
    ij = np.floor((xy - origin) / res).astype(np.int64)
    ok = (ij[:, 0] >= 0) & (ij[:, 0] < shape[1]) & (ij[:, 1] >= 0) & (ij[:, 1] < shape[0])
    return ij[ok], ok


def _count_grid(xyz, zr, origin, res, shape, min_pts):
    sel = (xyz[:, 2] >= zr[0]) & (xyz[:, 2] <= zr[1])
    ij, _ = _cells(xyz[sel, :2], origin, res, shape)
    cnt = np.zeros(shape, np.int32)
    np.add.at(cnt, (ij[:, 1], ij[:, 0]), 1)
    occ = cnt >= min_pts
    # 去掉孤立的单格（8 邻域内没有别的占用格）
    nb = ndimage.convolve(occ.astype(np.int32), np.ones((3, 3), np.int32), mode="constant") - occ
    return occ & (nb > 0)


def _raycast_free(occ, poses_xy, origin, res, n_rays, max_range):
    """occ[iy, ix] bool；返回 free bool。射线遇到占用即停。"""
    H, W = occ.shape
    # 给射线用的障碍膨胀 1 格，防止从墙上稀疏的缝里漏出去
    blk = ndimage.binary_dilation(occ, iterations=1)
    free = np.zeros_like(occ)
    ang = np.linspace(0, 2 * math.pi, n_rays, endpoint=False)
    dirs = np.stack([np.cos(ang), np.sin(ang)], 1)
    steps = np.arange(0, max_range, res * 0.5)
    for p in poses_xy:
        pts = p[None, None, :] + dirs[:, None, :] * steps[None, :, None]   # (R,S,2)
        ij = np.floor((pts - origin) / res).astype(np.int64)
        ix, iy = ij[..., 0], ij[..., 1]
        inb = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
        ixc, iyc = np.clip(ix, 0, W - 1), np.clip(iy, 0, H - 1)
        stop = blk[iyc, ixc] | ~inb
        first = np.where(stop.any(1), stop.argmax(1), stop.shape[1])
        m = np.arange(len(steps))[None, :] < first[:, None]
        free[iyc[m], ixc[m]] = True
    return free


def _swept(traj_T, box, origin, res, shape):
    """车体框扫过的格子"""
    sw = np.zeros(shape, bool)
    xs = np.arange(box["x"][0], box["x"][1], res * 0.5)
    ys = np.arange(box["y"][0], box["y"][1], res * 0.5)
    gx, gy = np.meshgrid(xs, ys)
    loc = np.stack([gx.ravel(), gy.ravel()], 1)
    last = None
    for T in traj_T:
        if last is not None and np.linalg.norm(T[:2, 3] - last) < res:
            continue
        last = T[:2, 3]
        yaw = C.yaw_of(T[:3, :3])
        c, s = math.cos(yaw), math.sin(yaw)
        w = loc @ np.array([[c, s], [-s, c]]) + T[:2, 3]
        ij, _ = _cells(w, origin, res, shape)
        sw[ij[:, 1], ij[:, 0]] = True
    return sw


def build_grids(xyz, traj_T, cfg):
    g, vc = cfg["grids"], cfg["vehicle"]
    res = g["resolution"]
    traj_xy = traj_T[:, :2, 3]
    nav_sel = (xyz[:, 2] >= g["nav_slice"][0]) & (xyz[:, 2] <= g["nav_slice"][1])
    allxy = np.concatenate([xyz[nav_sel | (np.abs(xyz[:, 2]) < g["ground_tol"]), :2], traj_xy])
    lo = np.floor((allxy.min(0) - g["pad"]) / res) * res
    hi = allxy.max(0) + g["pad"]
    shape = (int(math.ceil((hi[1] - lo[1]) / res)), int(math.ceil((hi[0] - lo[0]) / res)))  # (H, W)

    nav_occ = _count_grid(xyz, g["nav_slice"], lo, res, shape, g["min_points_per_cell"])
    loc_occ = _count_grid(xyz, g["loc_slice"], lo, res, shape, g["min_points_per_cell"])

    # 射线位姿：沿轨迹每 ray_pose_step 取一个
    sel, last = [], None
    for i, p in enumerate(traj_xy):
        if last is None or np.linalg.norm(p - last) >= g["ray_pose_step"]:
            sel.append(i)
            last = p
    free = _raycast_free(nav_occ, traj_xy[sel], lo, res, g["ray_count"], g["ray_max_range"])

    if g["free_requires_floor"]:
        gsel = np.abs(xyz[:, 2]) < g["ground_tol"]
        ij, _ = _cells(xyz[gsel, :2], lo, res, shape)
        floor = np.zeros(shape, bool)
        floor[ij[:, 1], ij[:, 0]] = True
        it = max(1, int(round(g["floor_dilate"] / res)))
        floor = ndimage.binary_dilation(floor, iterations=it)
        free &= floor

    swept = _swept(traj_T, vc["body_box_in_lidar"], lo, res, shape)
    free |= swept

    def to_img(occ):
        img = np.full(shape, UNKNOWN, np.uint8)
        img[free] = FREE
        img[occ] = OCC
        return img[::-1]  # 第 0 行 = y 最大

    nav_img = to_img(nav_occ)
    # 定位栅格：只要墙体轮廓，空闲区沿用导航的
    loc_img = to_img(loc_occ)

    blocked_on_route = int(np.count_nonzero(swept & nav_occ))
    stats = {
        "resolution": res, "origin": [float(lo[0]), float(lo[1])], "size_px": [shape[1], shape[0]],
        "size_m": [round(shape[1] * res, 2), round(shape[0] * res, 2)],
        "nav_occupied_cells": int(nav_occ.sum()), "free_cells": int(free.sum()),
        "loc_occupied_cells": int(loc_occ.sum()),
        "occupied_cells_on_driven_path": blocked_on_route,
    }
    return nav_img, loc_img, lo, stats


def write_grids(out, nav_img, loc_img, origin, cfg):
    g = cfg["grids"]
    os.makedirs(out, exist_ok=True)
    for name, img in (("nav_grid", nav_img), ("loc_grid", loc_img)):
        C.write_pgm(os.path.join(out, f"{name}.pgm"), img)
        C.write_map_yaml(os.path.join(out, f"{name}.yaml"), f"{name}.pgm", g["resolution"], origin,
                         occupied_thresh=g["occupied_thresh"], free_thresh=g["free_thresh"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pcd")
    ap.add_argument("traj")
    ap.add_argument("out")
    ap.add_argument("--config", default=None)
    args = ap.parse_args()
    cfg = C.load_config(args.config)
    xyz, _, _ = C.read_pcd(args.pcd)
    _, Tl = C.load_tum(args.traj)
    nav, loc, origin, st = build_grids(xyz, Tl, cfg)
    write_grids(args.out, nav, loc, origin, cfg)
    print("栅格：", st)
    if st["occupied_cells_on_driven_path"]:
        print(f"[警告] 车开过的位置上有 {st['occupied_cells_on_driven_path']} 个占用格，检查残影")


if __name__ == "__main__":
    main()
