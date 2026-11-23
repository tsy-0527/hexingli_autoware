#!/usr/bin/env python3
"""生成 Nav2 KeepoutFilter 掩膜：只允许在导航栅格"已建图"的范围内规划。

为什么需要：planner 的 allow_unknown: false 只挡住静态图里的"未知"格子。但全局代价地图的
VoxelLayer 开着 clearing，雷达射线扫过的未知格会被改成"空闲"（combination_method=1 是取最大值，
NO_INFORMATION 会被 FREE 覆盖），门外几米到十米的室外区域就又能规划了。KeepoutFilter 在所有
图层之后把掩膜外的格子强制设成禁行，不受雷达清除影响。

允许区域 = 栅格里已知（空闲或障碍）的格子 → 闭运算补掉 ≤ fill_hole 米的小未知洞 → 外扩 margin 米；
可选再与 --allowed-polygons 里的多边形求交（只保留室内 + 门口交接带）。

usage:
  make_keepout_mask.py <nav_map.yaml> <out_dir> [--margin 0.3] [--fill-hole 1.0]
                       [--allowed-polygons zones.yaml]   # yaml: allowed_polygons: [[[x,y],...], ...]
输出：keepout_mask.pgm / keepout_mask.yaml（mode trinary：黑=禁行，白=允许）
"""
import argparse
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


def in_polygon(GX, GY, poly):
    poly = np.asarray(poly, float)
    inside = np.zeros(GX.shape, bool)
    x0, y0 = poly[-1]
    for x1, y1 in poly:
        cond = (y1 > GY) != (y0 > GY)
        with np.errstate(divide='ignore', invalid='ignore'):
            xi = (x0 - x1) * (GY - y1) / (y0 - y1) + x1
        inside ^= cond & (GX <= xi)
        x0, y0 = x1, y1
    return inside


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('nav_yaml')
    ap.add_argument('out_dir')
    ap.add_argument('--margin', type=float, default=0.3)
    ap.add_argument('--fill-hole', type=float, default=1.0)
    ap.add_argument('--allowed-polygons', default='')
    a = ap.parse_args()

    meta = yaml.safe_load(open(a.nav_yaml))
    img = read_pgm(os.path.join(os.path.dirname(a.nav_yaml), meta['image']))
    res = float(meta['resolution'])
    x0, y0 = float(meta['origin'][0]), float(meta['origin'][1])
    neg = int(meta.get('negate', 0))
    occ_t, free_t = float(meta['occupied_thresh']), float(meta['free_thresh'])
    p = (img.astype(float) / 255.0) if neg else (255.0 - img.astype(float)) / 255.0
    known = (p > occ_t) | (p < free_t)

    r = max(1, int(round(a.fill_hole / 2 / res)))
    allowed = ndimage.binary_closing(known, structure=np.ones((2 * r + 1, 2 * r + 1), bool))
    allowed = ndimage.binary_fill_holes(allowed) & ndimage.binary_dilation(
        known, iterations=max(1, int(round(a.fill_hole / res))))   # 只补内部小洞，不补外围大片
    m = int(round(a.margin / res))
    if m > 0:
        allowed = ndimage.binary_dilation(allowed, iterations=m)

    if a.allowed_polygons:
        polys = (yaml.safe_load(open(a.allowed_polygons)) or {}).get('allowed_polygons') or []
        if polys:
            h, w = img.shape
            gx = x0 + (np.arange(w) + 0.5) * res
            gy = y0 + (np.arange(h) + 0.5) * res
            GX, GY = np.meshgrid(gx, gy)
            pm = np.zeros((h, w), bool)
            for poly in polys:
                pm |= in_polygon(GX, GY, poly)
            allowed &= np.flipud(pm)           # PGM 第 0 行 = y 最大

    os.makedirs(a.out_dir, exist_ok=True)
    out = np.where(allowed, 254, 0).astype(np.uint8)
    with open(os.path.join(a.out_dir, 'keepout_mask.pgm'), 'wb') as f:
        f.write(f'P5\n{out.shape[1]} {out.shape[0]}\n255\n'.encode())
        f.write(out.tobytes())
    with open(os.path.join(a.out_dir, 'keepout_mask.yaml'), 'w') as f:
        f.write('image: keepout_mask.pgm\nmode: trinary\n'
                f'resolution: {res}\norigin: [{x0}, {y0}, 0.0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    print(f'允许区域 {allowed.sum() * res * res:.0f} m²，禁行 {(~allowed).sum() * res * res:.0f} m²；'
          f'原图已知格 {known.sum() * res * res:.0f} m²')


if __name__ == '__main__':
    main()
