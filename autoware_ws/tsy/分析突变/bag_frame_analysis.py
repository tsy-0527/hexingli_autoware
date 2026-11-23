#!/usr/bin/env python3
"""帧间突变实测分析：读取 Autoware 的 ROS 2 bag 包，输出可上传的小体积汇总。

不依赖 ROS，只需要：pip install rosbags numpy matplotlib
用法：
    python bag_frame_analysis.py <bag 目录或 .db3 文件> -o out
    python bag_frame_analysis.py <bag> -o out --msg-root <Autoware 的 install 目录或 msg 文件夹>

输出（out 目录，约几十 KB 到几百 KB）：
    summary.json            汇总：话题清单、感知抖动与闪烁统计、各层轨迹的帧间突变、突变归因、求解耗时
    objects_<话题>.csv      每个静态障碍物（按位置聚类）的抖动与闪烁统计
    frames_<话题>.csv       每帧规划输出的帧间差异
    timeline.png / jitter.png  曲线图
把整个 out 目录压缩后上传即可。
"""
import argparse, csv, json, math, re, sqlite3, sys
from collections import defaultdict
from pathlib import Path

import numpy as np

try:
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
except ImportError:
    sys.exit("缺少 rosbags：请先运行  pip install rosbags numpy matplotlib")

# ---------------------------------------------------------------- 消息定义
# Humble 版 .db3 不含消息定义，这里内置 Autoware 常用消息；若与你的版本不符，
# 用 --msg-root 指向 Autoware 的 install 目录（或拷贝出来的 msg 文件夹），会自动覆盖。
PERC = "autoware_perception_msgs"
BUILTIN = {
    f"{PERC}/msg/ObjectClassification": "uint8 label\nfloat32 probability",
    f"{PERC}/msg/Shape": "uint8 type\ngeometry_msgs/Polygon footprint\ngeometry_msgs/Vector3 dimensions",
    f"{PERC}/msg/DetectedObjectKinematics":
        "geometry_msgs/PoseWithCovariance pose_with_covariance\nbool has_position_covariance\n"
        "uint8 orientation_availability\ngeometry_msgs/TwistWithCovariance twist_with_covariance\n"
        "bool has_twist\nbool has_twist_covariance",
    f"{PERC}/msg/DetectedObject":
        f"float32 existence_probability\n{PERC}/ObjectClassification[] classification\n"
        f"{PERC}/DetectedObjectKinematics kinematics\n{PERC}/Shape shape",
    f"{PERC}/msg/DetectedObjects": f"std_msgs/Header header\n{PERC}/DetectedObject[] objects",
    f"{PERC}/msg/TrackedObjectKinematics":
        "geometry_msgs/PoseWithCovariance pose_with_covariance\nuint8 orientation_availability\n"
        "geometry_msgs/TwistWithCovariance twist_with_covariance\n"
        "geometry_msgs/AccelWithCovariance acceleration_with_covariance\nbool is_stationary",
    f"{PERC}/msg/TrackedObject":
        f"unique_identifier_msgs/UUID object_id\nfloat32 existence_probability\n"
        f"{PERC}/ObjectClassification[] classification\n{PERC}/TrackedObjectKinematics kinematics\n{PERC}/Shape shape",
    f"{PERC}/msg/TrackedObjects": f"std_msgs/Header header\n{PERC}/TrackedObject[] objects",
    f"{PERC}/msg/PredictedPath": "geometry_msgs/Pose[] path\nbuiltin_interfaces/Duration time_step\nfloat32 confidence",
    f"{PERC}/msg/PredictedObjectKinematics":
        "geometry_msgs/PoseWithCovariance initial_pose_with_covariance\n"
        "geometry_msgs/TwistWithCovariance initial_twist_with_covariance\n"
        "geometry_msgs/AccelWithCovariance initial_acceleration_with_covariance\n"
        f"{PERC}/PredictedPath[] predicted_paths",
    f"{PERC}/msg/PredictedObject":
        f"unique_identifier_msgs/UUID object_id\nfloat32 existence_probability\n"
        f"{PERC}/ObjectClassification[] classification\n{PERC}/PredictedObjectKinematics kinematics\n{PERC}/Shape shape",
    f"{PERC}/msg/PredictedObjects": f"std_msgs/Header header\n{PERC}/PredictedObject[] objects",
    "autoware_planning_msgs/msg/TrajectoryPoint":
        "builtin_interfaces/Duration time_from_start\ngeometry_msgs/Pose pose\nfloat32 longitudinal_velocity_mps\n"
        "float32 lateral_velocity_mps\nfloat32 acceleration_mps2\nfloat32 heading_rate_rps\n"
        "float32 front_wheel_angle_rad\nfloat32 rear_wheel_angle_rad",
    "autoware_planning_msgs/msg/Trajectory": "std_msgs/Header header\nautoware_planning_msgs/TrajectoryPoint[] points",
    "autoware_planning_msgs/msg/PathPoint":
        "geometry_msgs/Pose pose\nfloat32 longitudinal_velocity_mps\nfloat32 lateral_velocity_mps\n"
        "float32 heading_rate_rps\nbool is_final",
    "autoware_planning_msgs/msg/Path":
        "std_msgs/Header header\nautoware_planning_msgs/PathPoint[] points\n"
        "geometry_msgs/Point[] left_bound\ngeometry_msgs/Point[] right_bound",
}
for pkg in ("tier4_planning_msgs", "autoware_internal_planning_msgs"):
    BUILTIN[f"{pkg}/msg/PathPointWithLaneId"] = "autoware_planning_msgs/PathPoint point\nint64[] lane_ids"
    BUILTIN[f"{pkg}/msg/PathWithLaneId"] = (f"std_msgs/Header header\n{pkg}/PathPointWithLaneId[] points\n"
                                             "geometry_msgs/Point[] left_bound\ngeometry_msgs/Point[] right_bound")
for pkg in ("tier4_debug_msgs", "autoware_internal_debug_msgs"):
    BUILTIN[f"{pkg}/msg/Float64Stamped"] = "builtin_interfaces/Time stamp\nfloat64 data"


def build_typestore(msg_roots):
    ts = get_typestore(Stores.ROS2_HUMBLE)
    types = {}
    for name, text in BUILTIN.items():
        types.update(get_types_from_msg(text, name))
    for root in msg_roots or []:
        for f in Path(root).rglob("*.msg"):
            pkg = f.parent.parent.name if f.parent.name == "msg" else f.parent.name
            if f.parent.name == "msg" and f.parent.parent.name not in ("share",):
                try:
                    types.update(get_types_from_msg(f.read_text(encoding="utf-8"), f"{pkg}/msg/{f.stem}"))
                except Exception as e:  # noqa
                    print(f"  跳过无法解析的定义 {f}: {e}")
    ts.register(types)
    return ts


# ---------------------------------------------------------------- 读取 bag
class Bag:
    """统一 .db3 单文件与 rosbag2 目录（含 .mcap）。"""

    def __init__(self, path, ts):
        self.path, self.ts = Path(path), ts
        if self.path.is_file() and self.path.suffix == ".db3":
            self.mode = "sqlite"
            self.db = sqlite3.connect(str(self.path))
            self.topics = {n: (i, t) for i, n, t in self.db.execute("select id,name,type from topics")}
            self.counts = dict(self.db.execute("select topic_id,count(*) from messages group by topic_id"))
        else:
            from rosbags.highlevel import AnyReader
            self.mode = "any"
            self.reader = AnyReader([self.path], default_typestore=ts)
            self.reader.open()
            self.topics = {c.topic: (c, c.msgtype) for c in self.reader.connections}

    def inventory(self):
        out = []
        for n, (h, t) in self.topics.items():
            cnt = self.counts.get(h, 0) if self.mode == "sqlite" else h.msgcount
            out.append(dict(topic=n, type=t, count=int(cnt)))
        return sorted(out, key=lambda d: d["topic"])

    def read(self, topic):
        h, typ = self.topics[topic]
        if self.mode == "sqlite":
            for t, data in self.db.execute("select timestamp,data from messages where topic_id=? order by timestamp", (h,)):
                yield t, self.ts.deserialize_cdr(data, typ)
        else:
            for conn, t, raw in self.reader.messages(connections=[h]):
                yield t, self.reader.deserialize(raw, conn.msgtype)


def stamp(msg, t_bag):
    h = getattr(msg, "header", None) or msg
    s = getattr(h, "stamp", None)
    return s.sec * 1e9 + s.nanosec if s is not None and s.sec > 0 else float(t_bag)


# ---------------------------------------------------------------- 工具函数
def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def proj(poly, pts):
    """点到折线的投影：返回 (有符号横向距离, 弧长)。左侧为正。"""
    a, b = poly[:-1], poly[1:]; d = b - a
    L2 = (d ** 2).sum(1); L2[L2 == 0] = 1e-12
    out = np.empty((len(pts), 2))
    s0 = np.concatenate([[0], np.cumsum(np.sqrt(L2))])
    for k, p in enumerate(pts):
        t = np.clip(((p - a) * d).sum(1) / L2, 0, 1)
        c = a + d * t[:, None]; dist = np.linalg.norm(p - c, axis=1); i = int(np.argmin(dist))
        cr = d[i, 0] * (p - a[i])[1] - d[i, 1] * (p - a[i])[0]
        out[k] = (math.copysign(dist[i], cr), s0[i] + t[i] * math.sqrt(L2[i]))
    return out, s0


def acf(x, ks=None, lags=(1, 2, 3, 5)):
    """自相关；给出帧序号 ks 时只用真正相隔 lag 帧的样本对（跳过漏检造成的间断）。"""
    x = np.asarray(x, float) - np.mean(x)
    ks = np.arange(len(x)) if ks is None else np.asarray(ks)
    v = (x ** 2).mean(); pos = {k: i for i, k in enumerate(ks)}
    out = {}
    for lag in lags:
        pairs = [(i, pos[k + lag]) for i, k in enumerate(ks) if k + lag in pos]
        out[lag] = float(np.mean([x[i] * x[j] for i, j in pairs]) / v) if len(pairs) > 5 and v > 0 else None
    return out


class Ego:
    def __init__(self, recs):
        recs.sort()
        self.t = np.array([r[0] for r in recs]); self.xy = np.array([[r[1], r[2]] for r in recs])
        self.yaw = np.array([r[3] for r in recs]); self.v = np.array([r[4] for r in recs])

    def at(self, t):
        i = int(np.clip(np.searchsorted(self.t, t), 0, len(self.t) - 1))
        return self.xy[i], self.yaw[i], self.v[i]


# ---------------------------------------------------------------- 感知：抖动与闪烁
def object_xy(o):
    k = o.kinematics
    for f in ("initial_pose_with_covariance", "pose_with_covariance"):
        if hasattr(k, f):
            p = getattr(k, f).pose.position; return p.x, p.y
    raise AttributeError("unknown object kinematics")


def object_speed(o):
    k = o.kinematics
    for f in ("initial_twist_with_covariance", "twist_with_covariance"):
        if hasattr(k, f):
            v = getattr(k, f).twist.linear; return math.hypot(v.x, v.y)
    return float("nan")


def load_objects(bag, topic, ego, tmin, tmax):
    frames = []
    for tb, m in bag.read(topic):
        t = stamp(m, tb)
        if not (tmin <= t <= tmax):
            continue
        fid = m.header.frame_id
        e_xy, e_yaw, _ = ego.at(t)
        dets = []
        for o in m.objects:
            x, y = object_xy(o)
            if fid not in ("map", "/map"):            # base_link 等车体坐标：用自车位姿近似变换到 map
                c, s = math.cos(e_yaw), math.sin(e_yaw)
                x, y = e_xy[0] + c * x - s * y, e_xy[1] + s * x + c * y
            oid = bytes(bytearray(o.object_id.uuid)).hex()[:8] if hasattr(o, "object_id") else ""
            lab = int(max(o.classification, key=lambda c: c.probability).label) if len(o.classification) else -1
            dets.append((x, y, object_speed(o), oid, lab))
        frames.append((t, dets))
    return frames


def analyze_objects(frames, ego, r_assoc=1.5, min_det=10, v_static=0.5, fov_range=30.0, fov_deg=60.0):
    """按位置聚类静态目标（不依赖 ID，可跨 ID 切换），统计横向抖动、相关性与闪烁。"""
    clusters = []   # 每个: dict(pts=[(k,x,y,id)], c=[x,y])
    for k, (t, dets) in enumerate(frames):
        for x, y, v, oid, lab in dets:
            if not (np.isnan(v) or v < v_static):
                continue
            best, bd = None, r_assoc
            for c in clusters:
                d = math.hypot(x - c["c"][0], y - c["c"][1])
                if d < bd and (not c["pts"] or c["pts"][-1][0] != k):
                    best, bd = c, d
            if best is None:
                best = dict(pts=[], c=[x, y]); clusters.append(best)
            best["pts"].append((k, x, y, oid, lab))
            n = len(best["pts"]); best["c"] = [best["c"][0] + (x - best["c"][0]) / n, best["c"][1] + (y - best["c"][1]) / n]
    ts = np.array([f[0] for f in frames])
    rows, transient = [], 0
    for ci, c in enumerate(clusters):
        if len(c["pts"]) < min_det:
            transient += 1; continue
        P = np.array([[p[1], p[2]] for p in c["pts"]]); ks = np.array([p[0] for p in c["pts"]])
        cen = np.median(P, axis=0)
        # 横向方向：自车离该目标最近时的航向的法向
        dist_e = np.linalg.norm(ego.xy - cen, axis=1); j = int(np.argmin(dist_e))
        h = ego.yaw[j]; n_vec = np.array([-math.sin(h), math.cos(h)]); t_vec = np.array([math.cos(h), math.sin(h)])
        lat = (P - cen) @ n_vec; lon = (P - cen) @ t_vec
        # 出现区间内、且自车在探测范围内的帧中，统计漏检
        k0, k1 = ks.min(), ks.max()
        seen = np.zeros(k1 - k0 + 1, bool); seen[ks - k0] = True
        valid = np.zeros(k1 - k0 + 1, bool)
        for i, k in enumerate(range(k0, k1 + 1)):        # 只统计障碍物在自车前方扇区内的帧（排除车后、远处）
            exy, eyaw, _ = ego.at(ts[k]); dv = cen - exy; dd = np.linalg.norm(dv)
            ang = abs((math.atan2(dv[1], dv[0]) - eyaw + math.pi) % (2 * math.pi) - math.pi)
            valid[i] = dd < fov_range and math.degrees(ang) < fov_deg
        pres = seen[valid]
        if len(pres) == 0:
            pres = np.array([True])
        gaps, run = [], 0
        for s in pres:
            if not s: run += 1
            elif run: gaps.append(run); run = 0
        n_pres = int(pres.sum()); n_trans = sum(1 for a, b in zip(pres[:-1], pres[1:]) if a and not b)
        # 连续帧上的横向残差相关
        dist_obs = np.array([np.linalg.norm(ego.at(ts[k])[0] - cen) for k in ks])
        bins = {}
        for lo_, hi_ in ((0, 10), (10, 20), (20, 40), (40, 80)):
            m = (dist_obs >= lo_) & (dist_obs < hi_)
            if m.sum() >= 5:
                bins[f"{lo_}-{hi_}m"] = float(np.std(lat[m]))
        labs = [p[4] for p in c["pts"]]
        rows.append(dict(cluster=ci, x=float(cen[0]), y=float(cen[1]), n_det=len(ks),
                         label=int(max(set(labs), key=labs.count)), n_fov_frames=int(valid.sum()),
                         static=bool(np.std(lat) < 0.3 and np.std(lon) < 0.3),
                         n_ids=len({p[3] for p in c["pts"]}), sigma_lat=float(np.std(lat)), sigma_lon=float(np.std(lon)),
                         acf_lat=acf(lat, ks), sigma_lat_by_dist=bins,
                         miss_rate=float(1 - n_pres / max(len(pres), 1)),
                         p_miss=float(n_trans / max(n_pres, 1)),
                         mean_gap_frames=float(np.mean(gaps)) if gaps else 0.0,
                         p_recover=float(1 / np.mean(gaps)) if gaps else None,
                         max_gap_frames=int(max(gaps)) if gaps else 0,
                         lat_series=[float(v) for v in lat[:2000]], min_ego_dist=float(dist_e.min())))
    return rows, transient, len(clusters)


# ---------------------------------------------------------------- 规划：各层帧间差异
def load_paths(bag, topic, tmin, tmax):
    out = []
    for tb, m in bag.read(topic):
        t = stamp(m, tb)
        if not (tmin <= t <= tmax) or len(m.points) < 3:
            continue
        pts = []
        for p in m.points:
            pose = p.pose if hasattr(p, "pose") else p.point.pose
            pts.append((pose.position.x, pose.position.y))
        out.append((t, np.array(pts)))
    return out


def analyze_paths(paths, ego, obstacles, near=5.0, mid=25.0, tail=3.0):
    rows, side_hist = [], defaultdict(list)
    prev = None
    for k, (t, P) in enumerate(paths):
        e_xy, _, v = ego.at(t)
        seg = np.linalg.norm(np.diff(P, axis=0), axis=1); sc = np.concatenate([[0], np.cumsum(seg)])
        (e_lat, e_s), = proj(P, [e_xy])[0]
        r = dict(k=k, t=t, length=float(sc[-1]), v_ego=float(v), ego_lat=float(e_lat))
        if prev is not None:
            pr, s0 = proj(prev, P)
            for name, lim in (("near", near), ("mid", mid)):
                m = (sc >= e_s) & (sc <= e_s + lim) & (pr[:, 1] > 0.01) & (pr[:, 1] < s0[-1] - tail)
                r[f"{name}_dev"] = float(np.abs(pr[m, 0]).max()) if m.sum() >= 2 else None
        # 相对每个静态障碍物从哪一侧通过：+1 障碍物在路径左侧（从右侧通过），-1 相反
        for o in obstacles:
            c = np.array([o["x"], o["y"]])
            (lat_o, s_o), = proj(P, [c])[0]
            if e_s < s_o < e_s + 40 and 0 < s_o < sc[-1] - 0.5 and abs(lat_o) < 4.0:
                side = int(np.sign(lat_o)) if abs(lat_o) >= 0.5 else 0      # 0：障碍物压在路径上（未绕行）
                side_hist[o["cluster"]].append((k, t, side, float(abs(lat_o))))
        rows.append(r); prev = P
    flips = []
    for cid, h in side_hist.items():
        for (k0, t0, s_a, _), (k1, t1, s_b, _) in zip(h[:-1], h[1:]):
            if s_a != s_b and k1 == k0 + 1:
                kind = "side_flip" if s_a * s_b == -1 else ("start_avoid" if s_a == 0 else "stop_avoid")
                flips.append(dict(cluster=cid, k=k1, t=t1, from_side=s_a, to_side=s_b, kind=kind))
    passes = {cid: dict(frames=len(h), min_clearance=min(x[3] for x in h),
                        on_path_frames=sum(1 for x in h if x[2] == 0)) for cid, h in side_hist.items()}
    return rows, flips, passes, side_hist


def perception_events(frames, obstacles, r=2.0, jump_sigma=3.0):
    """每帧：障碍物出现/消失、观测位置跳变（超过 jump_sigma 倍抖动）。只传入与路径相关的障碍物。"""
    ev = []
    for o in obstacles:
        c = np.array([o["x"], o["y"]]); present_prev, lat_prev = None, None
        for t, dets in frames:
            d = [math.hypot(x - c[0], y - c[1]) for x, y, *_ in dets]
            pres = bool(d) and min(d) < r
            if present_prev is not None and pres != present_prev:
                ev.append((t, "appear" if pres else "vanish", o["cluster"]))
            if pres:
                i = int(np.argmin(d)); x, y = dets[i][0], dets[i][1]
                if lat_prev is not None and math.hypot(x - lat_prev[0], y - lat_prev[1]) > jump_sigma * max(o["sigma_lat"], 0.03):
                    ev.append((t, "jump", o["cluster"]))
                lat_prev = (x, y)
            present_prev = pres
    return sorted(ev)


# ---------------------------------------------------------------- 主流程
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag"); ap.add_argument("-o", "--out", default="bag_analysis_out")
    ap.add_argument("--msg-root", action="append", help="Autoware install 目录或 msg 文件夹，可多次指定")
    ap.add_argument("--start", type=float, default=0, help="从 bag 开始后第几秒分析")
    ap.add_argument("--end", type=float, default=1e9, help="分析到 bag 开始后第几秒")
    ap.add_argument("--objects", action="append", help="障碍物话题（默认自动识别全部 Detected/Tracked/PredictedObjects）")
    ap.add_argument("--paths", action="append", help="规划路径/轨迹话题（默认自动识别全部 Path/PathWithLaneId/Trajectory）")
    ap.add_argument("--ego", default=None, help="自车 Odometry 话题，默认 /localization/kinematic_state")
    ap.add_argument("--spike", type=float, default=0.3, help="判为帧间突变的近端横向偏差阈值 (m)")
    a = ap.parse_args()
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    bag = Bag(a.bag, build_typestore(a.msg_root))
    inv = bag.inventory()
    summary = dict(bag=str(a.bag), topics=inv)
    print(f"共 {len(inv)} 个话题")

    ego_t = a.ego or next((d["topic"] for d in inv if d["topic"] == "/localization/kinematic_state"),
                          next((d["topic"] for d in inv if d["type"].endswith("nav_msgs/msg/Odometry")), None))
    if not ego_t:
        sys.exit("找不到自车 Odometry 话题，请用 --ego 指定")
    recs = []
    for tb, m in bag.read(ego_t):
        p = m.pose.pose
        recs.append((stamp(m, tb), p.position.x, p.position.y, yaw_of(p.orientation), m.twist.twist.linear.x))
    ego = Ego(recs)
    t0 = ego.t[0]; tmin, tmax = t0 + a.start * 1e9, t0 + a.end * 1e9
    summary["ego"] = dict(topic=ego_t, duration_s=float((ego.t[-1] - t0) / 1e9),
                          v_mean=float(ego.v.mean()), v_max=float(ego.v.max()))
    rel = lambda t: round((t - t0) / 1e9, 3)

    obj_topics = a.objects or [d["topic"] for d in inv if re.search(r"(Detected|Tracked|Predicted)Objects$", d["type"]) and d["count"]]
    path_topics = a.paths or [d["topic"] for d in inv if re.search(r"/(Path|PathWithLaneId|Trajectory)$", d["type"]) and d["count"]]
    summary["perception"], obstacles_main, frames_main = {}, [], None
    for tp in obj_topics:
        print(f"感知话题 {tp} ...")
        try:
            frames = load_objects(bag, tp, ego, tmin, tmax)
        except Exception as e:
            summary["perception"][tp] = dict(error=f"解析失败：{e}。请用 --msg-root 指定 Autoware 的消息定义。")
            print("  解析失败：", e); continue
        rows, transient, n_cl = analyze_objects(frames, ego)
        n_per = [len(d) for _, d in frames]
        summary["perception"][tp] = dict(
            frames=len(frames), mean_objects=float(np.mean(n_per)) if n_per else 0,
            static_obstacles=len(rows), transient_clusters=transient, total_clusters=n_cl,
            obstacles=[{k: v for k, v in r.items() if k != "lat_series"} for r in rows])
        with open(out / f"objects_{tp.strip('/').replace('/', '_')}.csv", "w", newline="", encoding="utf-8-sig") as f:
            w = csv.writer(f); w.writerow(["cluster", "x", "y", "n_det", "n_ids", "sigma_lat", "sigma_lon", "acf1",
                                           "miss_rate", "p_miss", "p_recover", "max_gap", "min_ego_dist"])
            for r in rows:
                w.writerow([r["cluster"], round(r["x"], 2), round(r["y"], 2), r["n_det"], r["n_ids"], round(r["sigma_lat"], 4),
                            round(r["sigma_lon"], 4), r["acf_lat"][1], round(r["miss_rate"], 4), round(r["p_miss"], 4),
                            r["p_recover"], r["max_gap_frames"], round(r["min_ego_dist"], 2)])
        if frames_main is None or "objects" in tp.split("/")[-1] and "object_recognition" in tp:
            if rows:
                frames_main, obstacles_main, main_tp = frames, rows, tp
        if rows:  # 供玩具仿真使用的参数（取被自车接近过的障碍物）
            near = [r for r in rows if r["min_ego_dist"] < 15] or rows
            summary["perception"][tp]["toy_sim_params"] = dict(
                sigma_l=float(np.median([r["sigma_lat"] for r in near])),
                rho=float(np.median([r["acf_lat"][1] for r in near if r["acf_lat"][1] is not None] or [np.nan])),
                p_miss=float(np.median([r["p_miss"] for r in near])),
                p_recover=float(np.median([r["p_recover"] for r in near if r["p_recover"]] or [np.nan])))

    summary["planning"] = {}
    path_res = {}
    for tp in path_topics:
        print(f"规划话题 {tp} ...")
        try:
            paths = load_paths(bag, tp, tmin, tmax)
        except Exception as e:
            summary["planning"][tp] = dict(error=f"解析失败：{e}"); print("  解析失败：", e); continue
        if len(paths) < 3:
            continue
        path_res[tp] = analyze_paths(paths, ego, obstacles_main)
    # 与路径相关的障碍物：曾出现在某条路径前方 40 m、横向 4 m 以内
    rel_ids = set()
    for _, _, _, sh in path_res.values():
        rel_ids |= set(sh)
    relevant = [o for o in obstacles_main if o["cluster"] in rel_ids]
    events = perception_events(frames_main, relevant) if frames_main else []
    ev_t = np.array([e[0] for e in events]) if events else np.array([])
    all_rows = {}
    for tp, (rows, flips, sides, sh) in path_res.items():
        all_rows[tp] = rows
        nd = np.array([r.get("near_dev") for r in rows if r.get("near_dev") is not None])
        md = np.array([r.get("mid_dev") for r in rows if r.get("mid_dev") is not None])
        spikes = [r for r in rows if (r.get("near_dev") or 0) > a.spike]
        attrib = dict(perception=0, planner=0)
        for r in spikes:   # 前 0.3 s 内，路径相关障碍物有出现/消失/跳变 -> 感知触发
            hit = len(ev_t) and ((ev_t >= r["t"] - 0.3e9) & (ev_t <= r["t"])).any()
            r["cause"] = "perception" if hit else "planner"; attrib[r["cause"]] += 1
        kinds = defaultdict(int)
        for f in flips: kinds[f["kind"]] += 1
        summary["planning"][tp] = dict(
            frames=len(rows), rate_hz=float(len(rows) / max((rows[-1]["t"] - rows[0]["t"]) / 1e9, 1e-6)),
            near_dev=dict(median=float(np.median(nd)), p95=float(np.percentile(nd, 95)), p99=float(np.percentile(nd, 99)), max=float(nd.max())) if len(nd) else None,
            mid_dev=dict(median=float(np.median(md)), p95=float(np.percentile(md, 95)), p99=float(np.percentile(md, 99)), max=float(md.max())) if len(md) else None,
            spikes=len(spikes), spike_threshold_m=a.spike, spike_cause=attrib,
            spike_times_s=[rel(r["t"]) for r in spikes][:200],
            decision_changes=dict(kinds), decision_change_list=[dict(f, t=rel(f["t"])) for f in flips][:300],
            obstacle_pass={k: v for k, v in sides.items()})
        with open(out / f"frames_{tp.strip('/').replace('/', '_')}.csv", "w", newline="", encoding="utf-8-sig") as f:
            w = csv.writer(f); w.writerow(["t_s", "length_m", "v_ego", "ego_lat", "near_dev", "mid_dev", "cause"])
            for r in rows:
                w.writerow([rel(r["t"]), round(r["length"], 2), round(r["v_ego"], 3), round(r["ego_lat"], 3),
                            r.get("near_dev"), r.get("mid_dev"), r.get("cause", "")])
    # 每次突变的细节：各层同一时刻的偏差、附近的感知事件与决策变化
    detail = []
    spike_ts = sorted({r["t"] for rows in all_rows.values() for r in rows if (r.get("near_dev") or 0) > a.spike})
    for ts_ in spike_ts[:100]:
        d = dict(t=rel(ts_), v_ego=float(ego.at(ts_)[2]), layers={})
        for tp, rows in all_rows.items():
            tt = np.array([r["t"] for r in rows]); i = int(np.argmin(np.abs(tt - ts_)))
            win = [rows[j].get("near_dev") or 0 for j in range(max(0, i - 2), min(len(rows), i + 3))]
            d["layers"][tp] = round(float(max(win)), 3)
        d["events"] = [dict(t=rel(te), kind=kd, cluster=c) for te, kd, c in events if ts_ - 1e9 <= te <= ts_ + 0.2e9][:20]
        d["decision_changes"] = [dict(tp=tp, **{k: v for k, v in f.items() if k in ("cluster", "kind", "t")})
                                 for tp, (_, fl, _, _) in path_res.items() for f in fl if abs(f["t"] - ts_) <= 1e9]
        for x in d["decision_changes"]: x["t"] = rel(x["t"])
        detail.append(d)
    summary["spike_detail"] = detail
    summary["perception_events"] = dict(source=main_tp if frames_main else None, relevant_obstacles=len(relevant),
                                        counts={k: sum(1 for e in events if e[1] == k) for k in ("appear", "vanish", "jump")})
    # 供玩具仿真使用的参数：静态、与路径相关、在前方扇区内观测足够多帧的障碍物
    good = [o for o in relevant if o["static"] and o["n_fov_frames"] >= 20]
    if good:
        summary["toy_sim_params_relevant_static"] = dict(
            n_obstacles=len(good),
            sigma_l=float(np.median([o["sigma_lat"] for o in good])),
            rho=float(np.median([o["acf_lat"][1] for o in good if o["acf_lat"][1] is not None] or [np.nan])),
            miss_rate=float(np.median([o["miss_rate"] for o in good])),
            p_miss=float(np.median([o["p_miss"] for o in good])),
            p_recover=float(np.median([o["p_recover"] for o in good if o["p_recover"]] or [np.nan])),
            id_switch_frac=float(np.mean([o["n_ids"] > 1 for o in good])))

    # 求解耗时
    pt = {}
    for d in inv:
        if "processing_time" in d["topic"] and d["type"].endswith("Float64Stamped") and d["count"]:
            try:
                vals = np.array([m.data for _, m in bag.read(d["topic"])])
                pt[d["topic"]] = dict(mean_ms=float(vals.mean()), p95_ms=float(np.percentile(vals, 95)),
                                      max_ms=float(vals.max()), over_100ms=int((vals > 100).sum()))
            except Exception as e:
                pt[d["topic"]] = dict(error=str(e))
    summary["processing_time"] = pt

    json.dump(summary, open(out / "summary.json", "w", encoding="utf-8"), ensure_ascii=False, indent=1, default=float)
    plot(out, all_rows, obstacles_main, events, summary, t0)
    print(f"完成，结果在 {out.resolve()}")


def plot(out, all_rows, obstacles, events, summary, t0):
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        from matplotlib import font_manager as fm
    except ImportError:
        print("未安装 matplotlib，跳过画图"); return
    for name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "PingFang SC", "WenQuanYi Zen Hei"):
        if any(name in f.name for f in fm.fontManager.ttflist):
            plt.rcParams["font.family"] = name; break
    plt.rcParams["axes.unicode_minus"] = False
    cols = ["#2a78d6", "#eb6834", "#1baf7a", "#6250d6", "#e34948"]
    if all_rows:
        fig, ax = plt.subplots(figsize=(12, 4.5), dpi=130)
        for i, (tp, rows) in enumerate(all_rows.items()):
            x = [(r["t"] - t0) / 1e9 for r in rows]; y = [r.get("near_dev") or 0 for r in rows]
            ax.plot(x, y, lw=1.2, color=cols[i % len(cols)], label=tp)
        for te, kind, _ in events:
            ax.axvline((te - t0) / 1e9, color={"appear": "#1baf7a", "vanish": "#e34948", "jump": "#eda100"}[kind], lw=0.6, alpha=0.5)
        for tp, d in summary["planning"].items():
            for f in d.get("decision_change_list", []):
                mk = {"side_flip": "v", "start_avoid": "^", "stop_avoid": "x"}[f["kind"]]
                ax.plot(f["t"], 0, marker=mk, color="#0b0b0b", ms=7)
        ax.set_xlabel("时间 (s)"); ax.set_ylabel("相邻帧近端最大横向偏差 (m)")
        ax.set_title("各层规划输出的帧间差异（竖线：绿=障碍物出现，红=消失，黄=位置跳变；▼绕行侧翻转 ▲开始绕行 ×停止绕行）", loc="left", fontsize=10)
        ax.legend(frameon=False, fontsize=8); ax.grid(alpha=0.3)
        fig.tight_layout(); fig.savefig(out / "timeline.png")
    if obstacles:
        top = sorted(obstacles, key=lambda r: r["min_ego_dist"])[:4]
        fig, axes = plt.subplots(1, 2, figsize=(12, 4), dpi=130)
        for i, r in enumerate(top):
            s = np.array(r["lat_series"])
            axes[0].plot(s, lw=0.9, color=cols[i], label=f"障碍物 {r['cluster']}（σ={r['sigma_lat']:.3f} m）")
            lags = range(1, 11); xs = s - s.mean(); v = (xs ** 2).mean() or 1
            axes[1].plot(list(lags), [(xs[:-l] * xs[l:]).mean() / v if len(xs) > l else np.nan for l in lags], marker="o", color=cols[i])
        axes[0].set_title("最接近的静态障碍物：横向观测残差", loc="left"); axes[0].set_xlabel("检测序号"); axes[0].set_ylabel("m")
        axes[1].set_title("横向残差的自相关（滞后帧数）", loc="left"); axes[1].set_xlabel("滞后 (帧)"); axes[1].axhline(0, color="#999", lw=0.8)
        axes[0].legend(frameon=False, fontsize=8)
        for ax in axes: ax.grid(alpha=0.3)
        fig.tight_layout(); fig.savefig(out / "jitter.png")


if __name__ == "__main__":
    main()
