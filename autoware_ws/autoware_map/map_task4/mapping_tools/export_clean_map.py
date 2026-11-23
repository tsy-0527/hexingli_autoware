#!/usr/bin/env python3
"""把 mapclean 结果导出为可用交付物：0.1 m NDT 图 + 两张预览图。

用法：python3 export_clean_map.py <session_dir>
输出：
  work/clean/map_leveled_0.1.pcd
  work/clean/map_preview.png
  work/clean/nav_grid_overlay.png
"""
import os
import sys

import numpy as np
import yaml
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common as C


def save_top_view(xyz, traj, out, res=0.10):
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    lo = np.floor(np.array([x.min(), y.min()]) / res) * res
    hi = np.array([x.max(), y.max()]) + res
    W = int(np.ceil((hi[0] - lo[0]) / res))
    H = int(np.ceil((hi[1] - lo[1]) / res))
    ix = np.clip(((x - lo[0]) / res).astype(np.int64), 0, W - 1)
    iy = np.clip(((y - lo[1]) / res).astype(np.int64), 0, H - 1)
    cnt = np.zeros((H, W), np.int32)
    zsum = np.zeros((H, W), np.float64)
    np.add.at(cnt, (iy, ix), 1)
    np.add.at(zsum, (iy, ix), z)
    occ = cnt > 0
    zmean = np.zeros_like(zsum)
    zmean[occ] = zsum[occ] / cnt[occ]
    img = np.full((H, W), np.nan)
    sel = occ & (np.abs(zmean) < 2.5)
    img[sel] = zmean[sel]

    fig, ax = plt.subplots(figsize=(14, 14), dpi=110)
    ax.set_facecolor("#101010")
    im = ax.imshow(np.ma.masked_invalid(img), origin="lower", cmap="turbo",
                   vmin=-0.2, vmax=2.0, interpolation="nearest",
                   extent=[lo[0], lo[0] + W * res, lo[1], lo[1] + H * res])
    if traj is not None:
        p = traj[:, :3, 3]
        ax.plot(p[:, 0], p[:, 1], color="#ffffff", lw=0.8, alpha=0.9, label="trajectory")
        ax.plot(p[0, 0], p[0, 1], "o", color="#00ff00", ms=6, label="start")
        ax.plot(p[-1, 0], p[-1, 1], "o", color="#ff3333", ms=6, label="end")
        ax.legend(loc="upper right")
    cb = fig.colorbar(im, ax=ax, shrink=0.7)
    cb.set_label("mean z (m)")
    ax.set_aspect("equal")
    ax.set_title("cleaned map top view (height colored)")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)
    print("saved", out)


def save_nav_overlay(clean_dir, traj, out):
    with open(os.path.join(clean_dir, "grids_preview", "nav_grid.yaml"), encoding="utf-8") as f:
        y = yaml.safe_load(f)
    img = C.read_pgm(os.path.join(clean_dir, "grids_preview", y["image"]))
    res = float(y["resolution"])
    ox, oy = y["origin"][:2]
    p = traj[:, :3, 3]
    H, W = img.shape
    step = max(1, int(np.ceil(max(W, H) / 1800)))
    fig, ax = plt.subplots(figsize=(12, 12), dpi=110)
    ax.imshow(img[::step, ::step], cmap="gray", origin="upper", interpolation="nearest",
              extent=[ox, ox + W * res, oy, oy + H * res])
    ax.plot(p[:, 0], p[:, 1], color="#ff3355", lw=0.8, alpha=0.9, label="trajectory")
    ax.plot(p[0, 0], p[0, 1], "o", color="#00ff66", ms=7, label="start")
    ax.plot(p[-1, 0], p[-1, 1], "o", color="#3399ff", ms=7, label="end")
    ax.set_aspect("equal")
    ax.set_xlim(ox, ox + W * res)
    ax.set_ylim(oy, oy + H * res)
    ax.set_title("nav_grid + trajectory (0=occupied, 254=free, 205=unknown)")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)
    print("saved", out, "grid", W, "x", H, "downsample", step)


def main(session):
    clean = os.path.join(session, "work", "clean")
    src = os.path.join(clean, "map_leveled.pcd")
    xyz, fields, _ = C.read_pcd(src)
    inten = fields.get("intensity", np.zeros(len(xyz), np.float32))
    print("map points", len(xyz))
    xyz2, inten2 = C.voxel_downsample(xyz, 0.1, inten)
    dst = os.path.join(clean, "map_leveled_0.1.pcd")
    C.write_pcd(dst, xyz2, {"intensity": inten2.astype(np.float32)})
    print("0.1 m", len(xyz2), "->", dst, "%.1f MB" % (os.path.getsize(dst) / 1e6))

    _, traj = C.load_tum(os.path.join(clean, "trajectory_leveled_tum.txt"))
    save_top_view(xyz, traj, os.path.join(clean, "map_preview.png"))
    save_nav_overlay(clean, traj, os.path.join(clean, "nav_grid_overlay.png"))


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("用法：python3 export_clean_map.py <session_dir>")
        sys.exit(2)
    main(os.path.abspath(sys.argv[1]))
