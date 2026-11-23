#!/usr/bin/env python3
"""把旧室内地图坐标系下的 2D 栅格，按 legacy_room_registration.yaml 变换到统一 map 坐标系，
并（可选）合并门前区域、打开门洞、加限速区。

为什么：20261117 点云里这栋楼的室内结构比较稀（走廊地板几乎没有点），旧室内地图的栅格更完整；
stations.yaml 也是按同一个配准换算的，用同一个变换最一致。

usage:
  register_legacy_grids.py <registration.yaml> <out_dir>
      --nav  map/indoor_nav_2d.yaml   --loc  map/indoor_loc_2d.yaml
      [--speed indoor_navigation/maps/speed_mask.yaml]
      [--apron door_apron.yaml]                 # 门前区域（统一坐标系）：障碍一律覆盖，空闲只填未知格
      [--force-free x0,y0,x1,y1 ...]            # 强制可通行的矩形（门洞）
      [--speed-zone x0,y0,x1,y1,speed ...]      # 追加限速矩形（m/s）
      [--obstacle-line x0,y0,x1,y1[,x2,y2...] ...]  # 画成障碍的折线（路沿坎、栏杆等点云高度体现不出来的矮台阶）
      [--line-width 0.15]                       # 折线宽度 m
处理顺序：旧室内图 → 门前区域 → force-free → obstacle-line（折线最后画，不会被 force-free 擦掉）
输出：nav_map / localization_map / speed_mask 的 .yaml/.pgm（统一坐标系，origin 不带 yaw）
"""
import argparse
import math
import os

import numpy as np
import yaml
from scipy import ndimage


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    toks, i = [], 0
    while len(toks) < 4:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while data[i:i + 1] not in (b'\n', b''):
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        toks.append(data[i:j])
        i = j
    w, h = int(toks[1]), int(toks[2])
    return np.frombuffer(data[i + 1:i + 1 + w * h], dtype=np.uint8).reshape(h, w)


def load_grid(path):
    meta = yaml.safe_load(open(path))
    img = read_pgm(os.path.join(os.path.dirname(path), meta['image']))
    return img, meta


def sample(img, meta, X, Y, cval):
    """在统一坐标 (X, Y) 处按最近邻取 img 的值（img 本身在统一坐标系，origin 不带 yaw）"""
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    H, W = img.shape
    col = np.floor((X - ox) / res).astype(int)
    row = H - 1 - np.floor((Y - oy) / res).astype(int)
    ok = (col >= 0) & (col < W) & (row >= 0) & (row < H)
    out = np.full(X.shape, cval, np.uint8)
    out[ok] = img[row[ok], col[ok]]
    return out


def transform_grid(src_yaml, R2, t2, cval, close_occupied, extra_bounds=None):
    img, meta = load_grid(src_yaml)
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    H, W = img.shape
    corners = np.array([[ox, oy], [ox + W * res, oy], [ox, oy + H * res], [ox + W * res, oy + H * res]])
    c2 = corners @ R2.T + t2
    lo, hi = c2.min(0), c2.max(0)
    if extra_bounds is not None:
        lo, hi = np.minimum(lo, extra_bounds[0]), np.maximum(hi, extra_bounds[1])
    nlo = np.floor(lo / res) * res
    NW = int(math.ceil((hi[0] - nlo[0]) / res))
    NH = int(math.ceil((hi[1] - nlo[1]) / res))
    gx = nlo[0] + (np.arange(NW) + 0.5) * res
    gy = nlo[1] + (np.arange(NH) + 0.5) * res
    GX, GY = np.meshgrid(gx, gy)
    Q = (np.stack([GX.ravel(), GY.ravel()], 1) - t2) @ R2          # 反变换回旧坐标
    col = (Q[:, 0] - ox) / res - 0.5
    row = (H - 1) - ((Q[:, 1] - oy) / res - 0.5)
    out = ndimage.map_coordinates(img, [row, col], order=0, mode='constant', cval=cval).reshape(NH, NW)
    if close_occupied:                                              # 补旋转造成的单像素缺口
        occ = out == 0
        out[ndimage.binary_closing(occ, structure=np.ones((3, 3), bool)) & ~occ] = 0
    # out 目前第 0 行 = y 最小；GX/GY 同序
    return out, GX, GY, meta, (nlo, res)


def write_grid(out_dir, name, img_ylow, meta, nlo, res, free_thresh=None, comment=''):
    img = img_ylow[::-1]
    with open(os.path.join(out_dir, name + '.pgm'), 'wb') as f:
        f.write(f'P5\n{img.shape[1]} {img.shape[0]}\n255\n'.encode())
        f.write(img.astype(np.uint8).tobytes())
    ft = meta['free_thresh'] if free_thresh is None else free_thresh
    with open(os.path.join(out_dir, name + '.yaml'), 'w') as f:
        f.write(comment)
        f.write(f"image: {name}.pgm\nmode: {meta.get('mode', 'trinary')}\nresolution: {res}\n"
                f'origin: [{nlo[0]:.3f}, {nlo[1]:.3f}, 0.0]\nnegate: {meta.get("negate", 0)}\n'
                f"occupied_thresh: {meta['occupied_thresh']}\nfree_thresh: {ft}\n")


def rect_mask(GX, GY, r):
    x0, y0, x1, y1 = r[:4]
    return (GX >= min(x0, x1)) & (GX <= max(x0, x1)) & (GY >= min(y0, y1)) & (GY <= max(y0, y1))


def polyline_mask(GX, GY, pts, width):
    m = np.zeros(GX.shape, bool)
    half = width / 2.0
    for (ax_, ay), (bx, by) in zip(pts[:-1], pts[1:]):
        dx, dy = bx - ax_, by - ay
        L2 = dx * dx + dy * dy
        t = np.clip(((GX - ax_) * dx + (GY - ay) * dy) / L2, 0.0, 1.0) if L2 > 0 else 0.0
        m |= np.hypot(GX - (ax_ + t * dx), GY - (ay + t * dy)) <= half
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('registration')
    ap.add_argument('out_dir')
    ap.add_argument('--nav', required=True)
    ap.add_argument('--loc', required=True)
    ap.add_argument('--speed', default='')
    ap.add_argument('--apron', default='')
    ap.add_argument('--force-free', nargs='*', default=[])
    ap.add_argument('--speed-zone', nargs='*', default=[])
    ap.add_argument('--obstacle-line', nargs='*', default=[])
    ap.add_argument('--line-width', type=float, default=0.15)
    a = ap.parse_args()
    reg = yaml.safe_load(open(a.registration))
    R = np.array(reg['rotation'], float)
    yaw = math.atan2(R[1, 0], R[0, 0])
    R2 = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    t2 = np.array(reg['translation'][:2], float)
    os.makedirs(a.out_dir, exist_ok=True)
    note = f'# 由旧室内栅格按 {os.path.basename(a.registration)}（yaw {math.degrees(yaw):.3f}°）变换到统一 map 坐标系\n'
    print(f'2D 变换：yaw {math.degrees(yaw):.3f}°，平移 ({t2[0]:.3f}, {t2[1]:.3f})')

    apron = None
    bounds = None
    if a.apron:
        aimg, ameta = load_grid(a.apron)
        ares = float(ameta['resolution'])
        alo = np.array(ameta['origin'][:2], float)
        ahi = alo + np.array([aimg.shape[1], aimg.shape[0]]) * ares
        apron = (aimg, ameta)
        bounds = (alo, ahi)
    rects = [[float(v) for v in s.split(',')] for s in a.force_free]
    zones = [[float(v) for v in s.split(',')] for s in a.speed_zone]
    lines = []
    for s in a.obstacle_line:
        v = [float(t) for t in s.split(',')]
        if len(v) < 4 or len(v) % 2:
            raise SystemExit(f'--obstacle-line 需要偶数个坐标（至少两点）：{s}')
        lines.append(list(zip(v[0::2], v[1::2])))

    # ---------------- 导航栅格
    nav, GX, GY, meta, (nlo, res) = transform_grid(a.nav, R2, t2, 205, True, bounds)
    if apron is not None:
        # 门前区域的障碍一律覆盖（旧室内图在玻璃立面处常记成空闲），空闲只填未知格
        av = sample(apron[0], apron[1], GX, GY, 205)
        nav[av == 0] = 0
        fill = (nav == 205) & (av == 254)
        nav[fill] = 254
    for r in rects:
        nav[rect_mask(GX, GY, r)] = 254
    for pl in lines:
        nav[polyline_mask(GX, GY, pl, a.line_width)] = 0
    write_grid(a.out_dir, 'nav_map', nav, meta, nlo, res, comment=note)
    print('nav_map', nav.shape[::-1], 'origin', np.round(nlo, 3))

    # ---------------- 定位栅格（房间全局初始化用，不加门前区域）
    loc, _, _, lmeta, (llo, lres) = transform_grid(a.loc, R2, t2, 205, True)
    # 旧 loc 图 free_thresh 0.25 会把 205（未知）当成空闲，统一改成 0.10
    write_grid(a.out_dir, 'localization_map', loc, lmeta, llo, lres, free_thresh=0.10, comment=note)

    # ---------------- 限速掩膜（与导航栅格同网格）
    if a.speed:
        sp, _, _, smeta, _ = transform_grid(a.speed, R2, t2, 255, False, bounds)
        # scale 模式：值 v = (255 - 像素)/255*100，限速 = v*0.01 m/s；v=0 表示不限速。重叠时取更严（v 更小且非 0）
        vals = np.rint((255.0 - sp.astype(float)) / 255.0 * 100.0)
        for z in zones:
            v = float(round(z[4] * 100))
            m = rect_mask(GX, GY, z)
            vals[m] = np.where(vals[m] == 0, v, np.minimum(vals[m], v))
        sp = np.clip(np.rint(255.0 * (1.0 - vals / 100.0)), 0, 255).astype(np.uint8)
        write_grid(a.out_dir, 'speed_mask', sp, smeta, nlo, res, comment=note)


if __name__ == '__main__':
    main()
