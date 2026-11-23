#!/usr/bin/env python3
"""bag → DUFOMap 输入：逐关键帧 PCD（世界系）+ VIEWPOINT=雷达位姿。

输出 <out>/
  pcd/000000.pcd ...       DUFOMap 输入（点在世界系，VIEWPOINT 为该帧雷达位姿）
  poses_lidar_tum.txt      全部帧的雷达位姿（世界系），供 mapclean / route_check 使用
  poses_body_tum.txt       FAST-LIO 原始 /Odometry
  keyframes.csv            关键帧索引
  raw_merged.pcd           未去动态的整图（对比 / 没装 DUFOMap 时兜底）

用法：python3 bag2dufo.py <bag_dir> <out_dir> [--config config/postprocess.yaml] [--stop-sec N]
"""
import argparse
import math
import os
import sys

import numpy as np

import common as C


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("out")
    ap.add_argument("--config", default=None)
    ap.add_argument("--stop-sec", type=float, default=None,
                    help="只处理 bag 开头 N 秒（相对第一条里程计）；用于裁掉发散的尾部")
    args = ap.parse_args()

    cfg = C.load_config(args.config)
    vc, bc = cfg["vehicle"], cfg["bag2dufo"]
    T_body_lidar = C.extrinsic_from_cfg(vc)
    box = vc["body_box_in_lidar"]
    bx, by, bz = box["x"], box["y"], box["z"]

    os.makedirs(os.path.join(args.out, "pcd"), exist_ok=True)

    # ---------- 第 1 遍：里程计
    print("[1/2] 读取里程计 ...")
    od_t, od_T = [], []
    for _, msg, _ in C.read_bag(args.bag, [bc["odom_topic"]]):
        od_t.append(C.stamp_sec(msg.header))
        od_T.append(C.odom_to_T(msg))
    if not od_t:
        sys.exit("bag 里没有里程计消息")
    od_t = np.array(od_t)
    order = np.argsort(od_t)
    od_t, od_T = od_t[order], np.array(od_T)[order]
    if args.stop_sec is not None:
        keep_od = od_t <= od_t[0] + args.stop_sec
        n_cut = int(len(od_t) - np.count_nonzero(keep_od))
        od_t, od_T = od_t[keep_od], od_T[keep_od]
        print(f"    --stop-sec {args.stop_sec:g}s：裁掉尾部 {n_cut} 条里程计")
    lidar_T = np.einsum("nij,jk->nik", od_T, T_body_lidar)
    C.save_tum(os.path.join(args.out, "poses_body_tum.txt"), od_t, od_T)
    C.save_tum(os.path.join(args.out, "poses_lidar_tum.txt"), od_t, lidar_T)
    t0 = od_t[0]
    length = np.sum(np.linalg.norm(np.diff(od_T[:, :3, 3], axis=0), axis=1))
    print(f"    {len(od_t)} 条，时长 {od_t[-1]-t0:.1f} s，轨迹长 {length:.1f} m")

    # ---------- 第 2 遍：点云
    print("[2/2] 处理点云 ...")
    last_kf = None
    kf_rows = []
    merged, merged_i = [], []
    n_cloud = n_nomatch = 0
    for _, msg, _ in C.read_bag(args.bag, [bc["cloud_topic"]]):
        n_cloud += 1
        ts = C.stamp_sec(msg.header)
        if ts - t0 < bc["skip_start_sec"]:
            continue
        if args.stop_sec is not None and ts - t0 > args.stop_sec:
            # bag 与 FAST-LIO 的时间戳单调，可直接结束遍历。
            break
        j = np.searchsorted(od_t, ts)
        cands = [k for k in (j - 1, j) if 0 <= k < len(od_t)]
        k = min(cands, key=lambda c: abs(od_t[c] - ts))
        if abs(od_t[k] - ts) > bc["max_stamp_dt"]:
            n_nomatch += 1
            continue
        T_wl = lidar_T[k]

        # 关键帧判定
        if last_kf is not None:
            d = np.linalg.norm(T_wl[:3, 3] - last_kf[:3, 3])
            dR = last_kf[:3, :3].T @ T_wl[:3, :3]
            ang = math.degrees(math.acos(np.clip((np.trace(dR) - 1) / 2, -1, 1)))
            if d < bc["keyframe_dist"] and ang < bc["keyframe_angle_deg"]:
                continue

        xyz, inten = C.cloud2_to_numpy(msg)
        if inten is None:
            inten = np.zeros(len(xyz), np.float32)
        if bc["cloud_frame"] == "world":
            p_l = C.transform(C.inv_T(T_wl), xyz)
            p_w = xyz
        else:  # body
            p_l = C.transform(C.inv_T(T_body_lidar), xyz)
            p_w = C.transform(od_T[k], xyz)

        r = np.linalg.norm(p_l, axis=1)
        keep = (r > bc["min_range"]) & (r < bc["max_range"])
        in_box = ((p_l[:, 0] > bx[0]) & (p_l[:, 0] < bx[1]) &
                  (p_l[:, 1] > by[0]) & (p_l[:, 1] < by[1]) &
                  (p_l[:, 2] > bz[0]) & (p_l[:, 2] < bz[1]))
        keep &= ~in_box
        p_w, inten = p_w[keep], inten[keep]
        if bc["frame_voxel"] > 0:
            p_w, inten = C.voxel_downsample(p_w, bc["frame_voxel"], inten)

        idx = len(kf_rows)
        C.write_pcd(os.path.join(args.out, "pcd", f"{idx:06d}.pcd"), p_w,
                    {"intensity": inten}, viewpoint=T_wl)
        kf_rows.append((idx, ts, *T_wl[:3, 3]))
        last_kf = T_wl

        merged.append(p_w)
        merged_i.append(inten)
        if len(merged) >= 50:
            m, mi = C.voxel_downsample(np.concatenate(merged), bc["merged_voxel"], np.concatenate(merged_i))
            merged, merged_i = [m], [mi]
        if idx % 100 == 0:
            print(f"    关键帧 {idx}  t={ts-t0:.1f}s  点数 {len(p_w)}")

    if not kf_rows:
        sys.exit("没有生成任何关键帧，检查 cloud_topic / odom_topic / 时间戳")
    if n_nomatch > 0.05 * n_cloud:
        print(f"[警告] {n_nomatch}/{n_cloud} 帧点云找不到对应里程计（|dt|>{bc['max_stamp_dt']}s）")

    with open(os.path.join(args.out, "keyframes.csv"), "w") as f:
        f.write("idx,stamp,x,y,z\n")
        for r in kf_rows:
            f.write("%d,%.9f,%.4f,%.4f,%.4f\n" % r)
    m, mi = C.voxel_downsample(np.concatenate(merged), bc["merged_voxel"], np.concatenate(merged_i))
    C.write_pcd(os.path.join(args.out, "raw_merged.pcd"), m, {"intensity": mi})

    # 回环检查：终点离起点多远（建议录制时回到起点）
    se = np.linalg.norm(od_T[-1][:3, 3] - od_T[0][:3, 3])
    print(f"完成：{len(kf_rows)} 个关键帧，整图 {len(m)} 点；终点距起点 {se:.2f} m"
          + ("（若已回到起点，这就是累计漂移）" if se < 1.0 else ""))


if __name__ == "__main__":
    main()
