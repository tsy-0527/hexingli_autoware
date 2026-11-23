#!/usr/bin/env python3
"""mapclean：调平 + 地面 + 离群点 + 可疑点 + route_check。

输入：DUFOMap 输出（或 raw_merged.pcd）+ poses_lidar_tum.txt
输出 <out>/
  map_leveled.pcd           调平后、清理后的地图（地面 z=0）—— 拿去 CloudCompare 人工修
  suspicious.pcd            被删掉的点（label: 1 地面下反射, 2 离群, 3 车道残影, 4 超高）—— 人工复核
  T_old_to_new.txt          4x4 调平矩阵（FAST-LIO 世界系 → 地图系）
  trajectory_leveled_tum.txt
  mapclean_report.md / .json

用法：python3 mapclean.py <in.pcd> <poses_lidar_tum.txt> <out_dir> [--config ...]
"""
import argparse
import json
import math
import os

import numpy as np
from scipy.spatial import cKDTree

import common as C


def fit_plane_ransac(P, thresh, iters=400, rng=None):
    rng = rng or np.random.default_rng(0)
    best_n, best_d, best_cnt = None, None, -1
    for _ in range(iters):
        s = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n /= nn
        if abs(n[2]) < 0.9:  # 只要近似水平的面
            continue
        d = -n @ s[0]
        cnt = np.count_nonzero(np.abs(P @ n + d) < thresh)
        if cnt > best_cnt:
            best_n, best_d, best_cnt = n, d, cnt
    if best_n is None:
        raise RuntimeError("找不到地面平面")
    inl = P[np.abs(P @ best_n + best_d) < thresh]
    c = inl.mean(0)
    _, _, vt = np.linalg.svd(inl - c, full_matrices=False)
    n = vt[2]
    if n[2] < 0:
        n = -n
    return n, -n @ c, inl


def rot_a_to_b(a, b):
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    v = np.cross(a, b)
    c = a @ b
    if np.linalg.norm(v) < 1e-12:
        return np.eye(3)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * (1 / (1 + c))


def find_floor(P, traj_xyz, h, radius, thresh):
    """两轮：粗窗口 RANSAC → 调平后细窗口重拟合。返回 (T_old_to_new, 地面内点数, 残差std)"""
    tree = cKDTree(traj_xyz[:, :2])
    T = np.eye(4)
    for win, rad in ((0.40, radius), (0.12, radius * 1.5)):
        Q = C.transform(T, P)
        tz = C.transform(T, traj_xyz)
        dist, idx = tree.query(Q[:, :2], distance_upper_bound=rad)
        ok = np.isfinite(dist)
        expect = np.full(len(Q), np.nan)
        expect[ok] = tz[idx[ok], 2] - h
        cand = Q[ok & (np.abs(Q[:, 2] - expect) < win)]
        if len(cand) < 500:
            raise RuntimeError(f"地面候选点太少({len(cand)})，检查 lidar_height 或 floor_search_radius")
        if len(cand) > 300000:
            cand = cand[np.random.default_rng(1).choice(len(cand), 300000, replace=False)]
        n, d, inl = fit_plane_ransac(cand, thresh)
        R = rot_a_to_b(n, np.array([0.0, 0.0, 1.0]))
        dT = np.eye(4)
        dT[:3, :3] = R
        # 平面上一点旋转后 z 应为 0
        p0 = -d * n
        dT[2, 3] = -(R @ p0)[2]
        T = dT @ T
    res = np.std(C.transform(np.eye(4), inl) @ n + d)
    return T, len(inl), float(res)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("in_pcd")
    ap.add_argument("poses")
    ap.add_argument("out")
    ap.add_argument("--config", default=None)
    args = ap.parse_args()
    cfg = C.load_config(args.config)
    vc, mc = cfg["vehicle"], cfg["mapclean"]
    os.makedirs(args.out, exist_ok=True)
    rep = {}

    xyz, fields, _ = C.read_pcd(args.in_pcd)
    inten = fields.get("intensity", np.zeros(len(xyz), np.float32)).astype(np.float32)
    rep["input_points"] = int(len(xyz))
    xyz, inten = C.voxel_downsample(xyz, mc["voxel"], inten)
    rep["after_voxel"] = int(len(xyz))
    print(f"输入 {rep['input_points']} 点 → 体素 {mc['voxel']} m 后 {len(xyz)} 点")

    stamps, Tl = C.load_tum(args.poses)
    traj = Tl[:, :3, 3]

    # ---------------------------------------------------------- 1. 调平
    T, n_floor, floor_res = find_floor(xyz, traj, vc["lidar_height"],
                                       mc["floor_search_radius"], mc["floor_ransac_thresh"])
    xyz = C.transform(T, xyz)
    Tl = np.einsum("ij,njk->nik", T, Tl)
    traj = Tl[:, :3, 3]
    tilt = math.degrees(math.acos(np.clip(T[2, 2], -1, 1)))
    rep["leveling"] = {
        "tilt_corrected_deg": round(tilt, 3),
        "floor_inliers": n_floor,
        "floor_residual_std_m": round(floor_res, 4),
        "lidar_height_measured_mean": round(float(traj[:, 2].mean()), 3),
        "lidar_height_measured_std": round(float(traj[:, 2].std()), 3),
        "lidar_height_config": vc["lidar_height"],
    }
    print(f"调平：修正倾角 {tilt:.2f}°，地面残差 {floor_res*100:.1f} cm，"
          f"雷达离地 {traj[:,2].mean():.3f}±{traj[:,2].std():.3f} m（配置 {vc['lidar_height']}）")
    np.savetxt(os.path.join(args.out, "T_old_to_new.txt"), T, fmt="%.9f",
               header="FAST-LIO 世界系 -> 地图系（地面 z=0）。p_new = T @ p_old")
    C.save_tum(os.path.join(args.out, "trajectory_leveled_tum.txt"), stamps, Tl)

    label = np.zeros(len(xyz), np.uint8)  # 0 保留

    # ---------------------------------------------------------- 2. 地面以下 / 超高
    label[(label == 0) & (xyz[:, 2] < -mc["below_floor_tol"])] = 1
    if mc.get("max_z") is not None:
        label[(label == 0) & (xyz[:, 2] > mc["max_z"])] = 4

    # ---------------------------------------------------------- 3. 离群点
    keep = np.flatnonzero(label == 0)
    P = xyz[keep]
    tree = cKDTree(P)
    d, _ = tree.query(P, k=mc["sor_k"] + 1, workers=-1)
    md = d[:, 1:].mean(1)
    sor_bad = md > md.mean() + mc["sor_std"] * md.std()
    cnt = tree.query_ball_point(P, mc["ror_radius"], workers=-1, return_length=True)
    ror_bad = cnt - 1 < mc["ror_min"]
    label[keep[sor_bad | ror_bad]] = 2

    # ---------------------------------------------------------- 4. route_check
    n_route = 0
    hotspots = []
    if mc["route_check"]:
        box = vc["body_box_in_lidar"]
        s = mc["route_shrink"]
        x0, x1 = box["x"][0] + s, box["x"][1] - s
        y0, y1 = box["y"][0] + s, box["y"][1] - s
        z_top = float(traj[:, 2].mean()) + box["z"][1]
        # 轨迹按 5 cm 重采样
        seg = np.linalg.norm(np.diff(traj[:, :2], axis=0), axis=1)
        sel = [0]
        acc = 0.0
        for i, sd in enumerate(seg, 1):
            acc += sd
            if acc >= 0.05:
                sel.append(i)
                acc = 0.0
        Ts = Tl[sel]
        keep = np.flatnonzero((label == 0) & (xyz[:, 2] > mc["route_z_min"]) & (xyz[:, 2] < z_top))
        P = xyz[keep]
        hit = np.zeros(len(P), bool)
        r_circ = math.hypot(max(abs(x0), abs(x1)), max(abs(y0), abs(y1)))
        ptree = cKDTree(P[:, :2])
        for Tp in Ts:
            ids = ptree.query_ball_point(Tp[:2, 3], r_circ)
            if not ids:
                continue
            ids = np.asarray(ids)
            yaw = C.yaw_of(Tp[:3, :3])
            c, sn = math.cos(yaw), math.sin(yaw)
            dx = P[ids, 0] - Tp[0, 3]
            dy = P[ids, 1] - Tp[1, 3]
            lx = c * dx + sn * dy
            ly = -sn * dx + c * dy
            inside = (lx > x0) & (lx < x1) & (ly > y0) & (ly < y1)
            hit[ids[inside]] = True
        label[keep[hit]] = 3
        # 残影往上延伸的部分（人的上半身等）：同一 10 cm 柱子里、车顶到 route_column_top 的点一起删
        if hit.any():
            colkey = lambda a: np.floor(a[:, :2] / 0.10).astype(np.int64) @ np.array([1 << 32, 1])
            ghost_cols = np.unique(colkey(P[hit]))
            up = np.flatnonzero((label == 0) & (xyz[:, 2] >= z_top) & (xyz[:, 2] < mc["route_column_top"]))
            label[up[np.isin(colkey(xyz[up]), ghost_cols)]] = 3
        n_route = int(np.count_nonzero(label == 3))
        if n_route:
            # 聚一下，报告残影位置
            H = P[hit]
            cells = np.unique(np.floor(H[:, :2] / 0.5).astype(int), axis=0, return_counts=True)
            order = np.argsort(-cells[1])[:15]
            hotspots = [{"x": float((cells[0][i][0] + .5) * .5), "y": float((cells[0][i][1] + .5) * .5),
                         "points": int(cells[1][i])} for i in order]

    # ---------------------------------------------------------- 输出
    good = label == 0
    C.write_pcd(os.path.join(args.out, "map_leveled.pcd"), xyz[good], {"intensity": inten[good]})
    bad = ~good
    C.write_pcd(os.path.join(args.out, "suspicious.pcd"), xyz[bad],
                {"intensity": inten[bad], "label": label[bad].astype(np.float32)})

    floor_pts = int(np.count_nonzero(good & (np.abs(xyz[:, 2]) < cfg["grids"]["ground_tol"])))
    rep["removed"] = {
        "below_floor_reflection": int(np.count_nonzero(label == 1)),
        "outlier": int(np.count_nonzero(label == 2)),
        "route_ghost": int(np.count_nonzero(label == 3)),
        "above_max_z": int(np.count_nonzero(label == 4)),
    }
    rep["output_points"] = int(good.sum())
    rep["ground_points"] = floor_pts
    rep["route_ghost_hotspots"] = hotspots
    se = float(np.linalg.norm(traj[-1] - traj[0]))
    rep["start_end_distance_m"] = round(se, 3)
    rep["start_end_dz_m"] = round(float(traj[-1, 2] - traj[0, 2]), 3)
    rep["map_bbox"] = {"min": xyz[good].min(0).round(2).tolist(), "max": xyz[good].max(0).round(2).tolist()}

    with open(os.path.join(args.out, "mapclean_report.json"), "w") as f:
        json.dump(rep, f, indent=2, ensure_ascii=False)

    warn = []
    lv = rep["leveling"]
    if abs(lv["lidar_height_measured_mean"] - lv["lidar_height_config"]) > 0.05:
        warn.append(f"实测雷达离地 {lv['lidar_height_measured_mean']} m 与配置 {lv['lidar_height_config']} m 差 >5 cm，检查地面是否找对")
    if lv["lidar_height_measured_std"] > 0.03:
        warn.append(f"轨迹高度波动 {lv['lidar_height_measured_std']*100:.1f} cm（>3 cm）：可能有 z 漂移或地面不平/有坡")
    if lv["floor_residual_std_m"] > 0.02:
        warn.append(f"地面残差 {lv['floor_residual_std_m']*100:.1f} cm（>2 cm）：地面有重影，可能漂移")
    if se < 1.0 and abs(rep["start_end_dz_m"]) > 0.05:
        warn.append(f"起终点高度差 {rep['start_end_dz_m']} m：回到起点但 z 不闭合，存在漂移")
    if n_route > 200:
        warn.append(f"route_check 删了 {n_route} 个点：录制时车道上有人/物，去 suspicious.pcd 里 label=3 复核")

    with open(os.path.join(args.out, "mapclean_report.md"), "w") as f:
        f.write("# mapclean 报告\n\n")
        f.write(f"- 输入点数：{rep['input_points']}，体素后：{rep['after_voxel']}，输出：{rep['output_points']}\n")
        f.write(f"- 调平：修正 {lv['tilt_corrected_deg']}°，地面残差 {lv['floor_residual_std_m']*100:.1f} cm\n")
        f.write(f"- 雷达离地：实测 {lv['lidar_height_measured_mean']} ± {lv['lidar_height_measured_std']} m（配置 {lv['lidar_height_config']}）\n")
        f.write(f"- 起终点距离：{se:.2f} m，高度差 {rep['start_end_dz_m']} m\n")
        f.write("- 删除：" + "，".join(f"{k} {v}" for k, v in rep["removed"].items()) + "\n")
        if hotspots:
            f.write("\n## 车道残影热点（0.5 m 格）\n\n| x | y | 点数 |\n|---|---|---|\n")
            for h in hotspots:
                f.write(f"| {h['x']:.2f} | {h['y']:.2f} | {h['points']} |\n")
        f.write("\n## 警告\n\n" + ("\n".join(f"- ⚠ {w}" for w in warn) if warn else "- 无") + "\n")
        f.write("\n## 下一步\n\n1. CloudCompare 打开 map_leveled.pcd + suspicious.pcd 对照修图\n"
                "2. 修完另存为 PCD（binary）\n3. `python3 build_map_package.py ...` 生成地图包\n")
    print(open(os.path.join(args.out, "mapclean_report.md"), encoding="utf-8").read())


if __name__ == "__main__":
    main()
