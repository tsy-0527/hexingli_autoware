#!/usr/bin/env python3
"""离线建图质量检查：把“地图能不能用”变成可量化的门控。

用法：python3 check_map_quality.py <session_dir>
输出：<session_dir>/work/clean/quality_report.md
"""
import json
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common as C


def bag_counts(bag_dir):
    p = os.path.join(bag_dir, "metadata.yaml")
    if not os.path.exists(p):
        return {}
    with open(p, encoding="utf-8") as f:
        m = yaml.safe_load(f)["rosbag2_bagfile_information"]
    return {t["topic_metadata"]["name"]: t["message_count"]
            for t in m["topics_with_message_count"]}


def find_recovered_bag(session):
    for name in ("recovered_bag", "recovered_fastlio_bag"):
        p = os.path.join(session, name)
        if os.path.exists(os.path.join(p, "metadata.yaml")):
            return p
    return os.path.join(session, "recovered_bag")


def grid_lookup(grid_dir, name):
    with open(os.path.join(grid_dir, name + ".yaml"), encoding="utf-8") as f:
        y = yaml.safe_load(f)
    img = C.read_pgm(os.path.join(grid_dir, y["image"]))
    res = float(y["resolution"])
    ox, oy = y["origin"][:2]

    def look(xy):
        xy = np.atleast_2d(xy)
        ix = np.floor((xy[:, 0] - ox) / res).astype(int)
        iy = np.floor((xy[:, 1] - oy) / res).astype(int)
        r = img.shape[0] - 1 - iy
        ok = (ix >= 0) & (ix < img.shape[1]) & (r >= 0) & (r < img.shape[0])
        v = np.full(len(xy), -1, int)
        v[ok] = img[r[ok], ix[ok]]
        return v
    return look


def add(checks, name, status, value, detail):
    checks.append({"name": name, "status": status, "value": value, "detail": detail})


def main(session):
    checks = []
    notes = []

    # ---------- 1. 恢复日志：扫描匹配无有效点的比例 ----------
    log = os.path.join(session, "recovery_logs", "fastlio.log")
    counts = bag_counts(find_recovered_bag(session))
    n_cloud = counts.get("/cloud_registered", 0)
    no_eff = 0
    if os.path.exists(log):
        with open(log, encoding="utf-8", errors="ignore") as f:
            for line in f:
                if "No Effective Points" in line:
                    no_eff += 1
    if n_cloud:
        ratio = no_eff / n_cloud
        st = "PASS" if ratio < 0.10 else ("WARN" if ratio < 0.25 else "FAIL")
        add(checks, "扫描匹配有效帧", st, "%.1f%%" % (100 * ratio),
            "%d/%d 帧无有效平面匹配（越低越好）" % (no_eff, n_cloud))
    else:
        add(checks, "扫描匹配有效帧", "WARN", "n/a", "找不到 recovered_bag/metadata.yaml")

    # ---------- 2. 轨迹高度（调平后） ----------
    traj_path = os.path.join(session, "work", "clean", "trajectory_leveled_tum.txt")
    if os.path.exists(traj_path):
        _, Tl = C.load_tum(traj_path)
        p = Tl[:, :3, 3]
        z = p[:, 2]
        z_med, z_std = float(np.median(z)), float(np.std(z))
        st = "PASS" if z_std < 0.10 else ("WARN" if z_std < 0.30 else "FAIL")
        add(checks, "轨迹高度波动", st, "%.3f m" % z_std,
            "中位数 %.3f m、范围 %.2f..%.2f m；单层应 <0.10 m" %
            (z_med, z.min(), z.max()))
        dz = abs(z_med - 0.57)
        st = "PASS" if dz < 0.05 else ("WARN" if dz < 0.15 else "FAIL")
        add(checks, "雷达离地中位数", st, "%.3f m" % z_med,
            "配置 0.57 m，偏差 %.3f m" % dz)
        if len(p) > 1:
            step = np.linalg.norm(np.diff(p, axis=0), axis=1)
            max_step = float(step.max())
            st = "PASS" if max_step < 1.0 else ("WARN" if max_step < 3.0 else "FAIL")
            add(checks, "轨迹最大单帧跳变", st, "%.2f m" % max_step,
                "10 Hz 下 >1 m 基本就是发散")
        se = float(np.linalg.norm(p[-1, :2] - p[0, :2]))
        add(checks, "起终点平面距离", "INFO", "%.2f m" % se,
            "闭环路线应接近 0；非闭环只看趋势")
    else:
        add(checks, "轨迹高度波动", "WARN", "n/a", "缺少 trajectory_leveled_tum.txt")

    # ---------- 3. mapclean 报告 ----------
    rep_path = os.path.join(session, "work", "clean", "mapclean_report.json")
    if os.path.exists(rep_path):
        with open(rep_path, encoding="utf-8") as f:
            rep = json.load(f)
        lv = rep.get("leveling", {})
        res = lv.get("floor_residual_std_m", 9.9)
        st = "PASS" if res < 0.02 else ("WARN" if res < 0.05 else "FAIL")
        add(checks, "地面平面残差", st, "%.2f cm" % (res * 100),
            "调平修正 %.2f°" % lv.get("tilt_corrected_deg", 0.0))
        add(checks, "地图点数", "INFO", str(rep.get("output_points", 0)),
            "删除：%s" % json.dumps(rep.get("removed", {}), ensure_ascii=False))
    else:
        add(checks, "地面平面残差", "WARN", "n/a", "缺少 mapclean_report.json")

    # ---------- 4. 轨迹在导航栅格上的可通行比例 ----------
    grid_dir = os.path.join(session, "work", "clean", "grids_preview")
    if os.path.exists(os.path.join(grid_dir, "nav_grid.yaml")) and os.path.exists(traj_path):
        try:
            look = grid_lookup(grid_dir, "nav_grid")
            _, Tl = C.load_tum(traj_path)
            v = look(Tl[:, :2, 3])
            n = len(v)
            free = int(np.sum(v == 254))
            occ = int(np.sum(v == 0))
            unk = int(np.sum(v == 205))
            r = free / n if n else 0.0
            st = "PASS" if r > 0.95 else ("WARN" if r > 0.90 else "FAIL")
            add(checks, "轨迹可通行比例", st, "%.1f%%" % (100 * r),
                "free %d / occ %d / unknown %d" % (free, occ, unk))
            if r <= 0.95:
                notes.append("轨迹有 %d 个位姿落在占用格：多为坡道/抬高走廊的 2D 绝对高度切片问题，"
                             "或车体点/残影；用 nav_grid_overlay.png 复核。" % occ)
        except Exception as e:  # noqa: BLE001
            add(checks, "轨迹可通行比例", "WARN", "n/a", "栅格读取失败：%s" % e)
    else:
        add(checks, "轨迹可通行比例", "WARN", "n/a", "缺少 nav_grid 或轨迹")

    # ---------- 5. 结论 ----------
    order = {"PASS": 0, "INFO": 0, "WARN": 1, "FAIL": 2}
    worst = max(order[c["status"]] for c in checks) if checks else 0
    overall = {0: "PASS", 1: "WARN", 2: "FAIL"}[worst]

    out = os.path.join(session, "work", "clean", "quality_report.md")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        f.write("# 地图质量报告\n\n")
        f.write("- 会话：`%s`\n" % session)
        f.write("- 结论：**%s**\n\n" % overall)
        f.write("| 检查 | 数值 | 结论 | 说明 |\n|---|---|---|---|\n")
        for c in checks:
            f.write("| %s | %s | %s | %s |\n" %
                    (c["name"], c["value"], c["status"], c["detail"]))
        if notes:
            f.write("\n## 说明\n\n")
            for n in notes:
                f.write("- %s\n" % n)
        f.write("\n## 判定建议\n\n")
        f.write("- PASS：可直接进入 CloudCompare 复核/打包。\n")
        f.write("- WARN：可用，但要复核 suspicious.pcd 和 nav_grid_overlay.png。\n")
        f.write("- FAIL：不要直接上车；先分段重跑、减速或调整 FAST-LIO 参数。\n")
    for c in checks:
        print("[%s] %s = %s（%s）" % (c["status"], c["name"], c["value"], c["detail"]))
    print("质量报告：", out)
    return overall


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("用法：python3 check_map_quality.py <session_dir>")
        sys.exit(2)
    main(os.path.abspath(sys.argv[1]))
