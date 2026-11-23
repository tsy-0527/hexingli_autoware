"""建图后处理公共函数：PCD 读写、位姿变换、ROS 2 bag 读取、配置加载。

只依赖 numpy / scipy / pyyaml；读 bag 时才需要 rclpy + rosbag2_py（Humble 自带）。
"""
import io
import math
import os
import struct

import numpy as np
import yaml

# --------------------------------------------------------------------------- 配置

_DEFAULT_CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config", "postprocess.yaml")


def load_config(path=None):
    with open(path or _DEFAULT_CFG, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


# --------------------------------------------------------------------------- 变换

def quat_to_rot(qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def rot_to_quat(R):
    """返回 (qx, qy, qz, qw)"""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    q = np.array([qx, qy, qz, qw])
    if q[3] < 0:
        q = -q
    return tuple(q / np.linalg.norm(q))


def make_T(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def inv_T(T):
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


def transform(T, xyz):
    return xyz @ T[:3, :3].T + T[:3, 3]


def yaw_of(R):
    return math.atan2(R[1, 0], R[0, 0])


def extrinsic_from_cfg(vcfg):
    R = np.array(vcfg.get("extrinsic_R_body_lidar", np.eye(3).tolist()), dtype=float).reshape(3, 3)
    t = np.array(vcfg.get("extrinsic_T_body_lidar", [0, 0, 0]), dtype=float)
    return make_T(R, t)


# --------------------------------------------------------------------------- 轨迹 (TUM)

def save_tum(path, stamps, Ts):
    with open(path, "w") as f:
        f.write("# timestamp x y z qx qy qz qw\n")
        for s, T in zip(stamps, Ts):
            qx, qy, qz, qw = rot_to_quat(T[:3, :3])
            x, y, z = T[:3, 3]
            f.write(f"{s:.9f} {x:.6f} {y:.6f} {z:.6f} {qx:.8f} {qy:.8f} {qz:.8f} {qw:.8f}\n")


def load_tum(path):
    data = np.loadtxt(path, comments="#", ndmin=2)
    stamps = data[:, 0]
    Ts = np.array([make_T(quat_to_rot(*r[4:8]), r[1:4]) for r in data])
    return stamps, Ts


# --------------------------------------------------------------------------- 体素 / 几何

def voxel_keys(xyz, voxel):
    k = np.floor(xyz / voxel).astype(np.int64) + (1 << 20)
    return (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]


def voxel_downsample(xyz, voxel, *extra):
    """每个体素保留一个点（第一个）。extra 为同长度的附加数组。"""
    if len(xyz) == 0 or voxel <= 0:
        return (xyz, *extra) if extra else xyz
    _, idx = np.unique(voxel_keys(xyz, voxel), return_index=True)
    idx.sort()
    if extra:
        return (xyz[idx], *[e[idx] for e in extra])
    return xyz[idx]


# --------------------------------------------------------------------------- PCD 读写

_PCD_NP = {
    ("F", 4): np.float32, ("F", 8): np.float64,
    ("U", 1): np.uint8, ("U", 2): np.uint16, ("U", 4): np.uint32, ("U", 8): np.uint64,
    ("I", 1): np.int8, ("I", 2): np.int16, ("I", 4): np.int32, ("I", 8): np.int64,
}


def _lzf_decompress(data, out_len):
    try:
        import lzf  # pip install python-lzf（快）
        return lzf.decompress(data, out_len)
    except ImportError:
        pass
    out = bytearray(out_len)
    ip = op = 0
    n = len(data)
    while ip < n:
        ctrl = data[ip]
        ip += 1
        if ctrl < 32:
            ln = ctrl + 1
            out[op:op + ln] = data[ip:ip + ln]
            ip += ln
            op += ln
        else:
            ln = ctrl >> 5
            ref = op - ((ctrl & 0x1F) << 8) - 1
            if ln == 7:
                ln += data[ip]
                ip += 1
            ref -= data[ip]
            ip += 1
            ln += 2
            if ref + ln <= op:
                out[op:op + ln] = out[ref:ref + ln]
            else:
                for i in range(ln):
                    out[op + i] = out[ref + i]
            op += ln
    return bytes(out)


def read_pcd(path):
    """返回 (xyz[N,3] float64, fields dict{name: array}, header dict)。支持 ascii/binary/binary_compressed。"""
    with open(path, "rb") as f:
        header = {}
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f"{path}: PCD 头不完整")
            s = line.decode("ascii", errors="ignore").strip()
            if not s or s.startswith("#"):
                continue
            k, *v = s.split()
            header[k.upper()] = v
            if k.upper() == "DATA":
                break
        body = f.read()

    names = header["FIELDS"]
    sizes = [int(x) for x in header["SIZE"]]
    types = header["TYPE"]
    counts = [int(x) for x in header.get("COUNT", ["1"] * len(names))]
    npts = int(header["POINTS"][0])
    seen = {}
    uniq = []
    for nm in names:  # PCL 的 "_" 填充字段会重名
        if nm in seen:
            seen[nm] += 1
            nm = f"{nm}_{seen[nm]}"
        else:
            seen[nm] = 0
        uniq.append(nm)
    dt = np.dtype([(nm, _PCD_NP[(t, s)], (c,)) if c > 1 else (nm, _PCD_NP[(t, s)])
                   for nm, t, s, c in zip(uniq, types, sizes, counts)])
    fmt = header["DATA"][0].lower()

    if fmt == "ascii":
        raw = np.loadtxt(io.BytesIO(body), ndmin=2)
        arr = np.zeros(raw.shape[0], dtype=dt)
        col = 0
        for nm, c in zip(uniq, counts):
            arr[nm] = raw[:, col] if c == 1 else raw[:, col:col + c]
            col += c
    elif fmt == "binary":
        arr = np.frombuffer(body, dtype=dt, count=npts)
    elif fmt == "binary_compressed":
        csize, usize = struct.unpack("<II", body[:8])
        buf = _lzf_decompress(body[8:8 + csize], usize)
        arr = np.zeros(npts, dtype=dt)
        off = 0
        for nm, t, s, c in zip(uniq, types, sizes, counts):
            nbytes = npts * s * c
            col = np.frombuffer(buf[off:off + nbytes], dtype=_PCD_NP[(t, s)])
            arr[nm] = col.reshape(npts, c) if c > 1 else col
            off += nbytes
    else:
        raise ValueError(f"不支持的 PCD DATA 格式: {fmt}")

    xyz = np.stack([arr["x"], arr["y"], arr["z"]], 1).astype(np.float64)
    fields = {nm: np.asarray(arr[nm]) for nm in uniq if nm not in ("x", "y", "z")}
    ok = np.isfinite(xyz).all(1)
    if not ok.all():
        xyz = xyz[ok]
        fields = {k: v[ok] for k, v in fields.items()}
    return xyz, fields, header


def write_pcd(path, xyz, fields=None, viewpoint=None):
    """binary PCD。fields: {name: 1D array}，统一存 float32。viewpoint: 4x4 位姿。"""
    fields = fields or {}
    n = len(xyz)
    names = ["x", "y", "z"] + list(fields.keys())
    dt = np.dtype([(nm, np.float32) for nm in names])
    arr = np.empty(n, dtype=dt)
    arr["x"], arr["y"], arr["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    for k, v in fields.items():
        arr[k] = v
    if viewpoint is None:
        vp = "0 0 0 1 0 0 0"
    else:
        qx, qy, qz, qw = rot_to_quat(viewpoint[:3, :3])
        t = viewpoint[:3, 3]
        vp = f"{t[0]:.6f} {t[1]:.6f} {t[2]:.6f} {qw:.8f} {qx:.8f} {qy:.8f} {qz:.8f}"
    hdr = (
        "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
        f"FIELDS {' '.join(names)}\nSIZE {' '.join(['4'] * len(names))}\n"
        f"TYPE {' '.join(['F'] * len(names))}\nCOUNT {' '.join(['1'] * len(names))}\n"
        f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT {vp}\nPOINTS {n}\nDATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(hdr.encode("ascii"))
        f.write(arr.tobytes())


# --------------------------------------------------------------------------- 栅格 PGM/YAML

def write_pgm(path, img):
    img = np.asarray(img, dtype=np.uint8)
    with open(path, "wb") as f:
        f.write(f"P5\n{img.shape[1]} {img.shape[0]}\n255\n".encode("ascii"))
        f.write(img.tobytes())


def read_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    tokens = []
    i = 0
    while len(tokens) < 4:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while data[i:i + 1] not in (b"\n", b""):
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    i += 1
    w, h = int(tokens[1]), int(tokens[2])
    return np.frombuffer(data[i:i + w * h], dtype=np.uint8).reshape(h, w)


def write_map_yaml(path, image, resolution, origin_xy, mode="trinary",
                   occupied_thresh=0.65, free_thresh=0.196):
    with open(path, "w") as f:
        f.write(f"image: {image}\nmode: {mode}\nresolution: {resolution}\n"
                f"origin: [{origin_xy[0]:.4f}, {origin_xy[1]:.4f}, 0.0]\n"
                f"negate: 0\noccupied_thresh: {occupied_thresh}\nfree_thresh: {free_thresh}\n")


# --------------------------------------------------------------------------- ROS 2 bag

def stamp_sec(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def _bag_storage_id(bag_dir):
    meta = os.path.join(bag_dir, "metadata.yaml")
    with open(meta, "r") as f:
        m = yaml.safe_load(f)
    return m["rosbag2_bagfile_information"]["storage_identifier"]


def read_bag(bag_dir, topics):
    """逐条产出 (topic, msg, t_ns)。"""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=_bag_storage_id(bag_dir)),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = [t for t in topics if t not in types]
    if missing:
        raise RuntimeError(f"bag 里没有话题 {missing}，现有: {sorted(types)}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    msg_cls = {t: get_message(types[t]) for t in topics}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        yield topic, deserialize_message(data, msg_cls[topic]), t_ns


_PF_NP = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
          5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}


def cloud2_to_numpy(msg):
    """sensor_msgs/PointCloud2 -> (xyz[N,3] float64, intensity[N] or None)"""
    names, fmts, offs = [], [], []
    for f in msg.fields:
        if f.count != 1 or f.datatype not in _PF_NP:
            continue
        names.append(f.name)
        dtp = np.dtype(_PF_NP[f.datatype])
        fmts.append(dtp.newbyteorder(">") if msg.is_bigendian else dtp)
        offs.append(f.offset)
    dt = np.dtype({"names": names, "formats": fmts, "offsets": offs, "itemsize": msg.point_step})
    n = msg.width * msg.height
    buf = memoryview(msg.data) if not isinstance(msg.data, (bytes, bytearray)) else msg.data
    if msg.height > 1 and msg.row_step != msg.width * msg.point_step:
        rows = np.frombuffer(buf, dtype=np.uint8).reshape(msg.height, msg.row_step)
        rows = rows[:, :msg.width * msg.point_step].copy()
        arr = np.frombuffer(rows.tobytes(), dtype=dt, count=n)
    else:
        arr = np.frombuffer(buf, dtype=dt, count=n)
    xyz = np.stack([arr["x"], arr["y"], arr["z"]], 1).astype(np.float64)
    inten = arr["intensity"].astype(np.float32) if "intensity" in names else None
    ok = np.isfinite(xyz).all(1)
    return xyz[ok], (inten[ok] if inten is not None else None)


def odom_to_T(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return make_T(quat_to_rot(q.x, q.y, q.z, q.w), [p.x, p.y, p.z])
