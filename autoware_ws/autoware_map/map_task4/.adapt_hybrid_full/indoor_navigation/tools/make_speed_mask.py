#!/usr/bin/env python3
"""Build a Nav2 SpeedFilter mask aligned with the navigation map.

usage: make_speed_mask.py <nav_map.yaml> <speed_zones.yaml> <out_dir>
Mask encoding (map_server mode 'scale', occupied_thresh 1.0, free_thresh 0.0):
  cell value v in 0..100 ; 0 = no limit ; speed limit = base + v * multiplier
  with costmap_filter_info_server type 2 (absolute), base 0.0, multiplier 0.01 -> v = speed * 100
"""
import os
import sys

import numpy as np
import yaml


def main(nav_yaml, zones_yaml, out_dir):
    meta = yaml.safe_load(open(nav_yaml))
    img_path = os.path.join(os.path.dirname(nav_yaml), meta['image'])
    with open(img_path, 'rb') as f:
        data = f.read()
    hdr, pos = [], 0
    while len(hdr) < 4:
        end = data.index(b'\n', pos)
        line = data[pos:end]
        pos = end + 1
        if not line.startswith(b'#'):
            hdr += line.split()
    w, h = int(hdr[1]), int(hdr[2])
    res, (x0, y0, _) = meta['resolution'], meta['origin']
    gx = x0 + (np.arange(w) + 0.5) * res
    gy = y0 + (np.arange(h) + 0.5) * res
    GX, GY = np.meshgrid(gx, gy)
    v = np.zeros((h, w), np.int32)
    for z in yaml.safe_load(open(zones_yaml))['zones']:
        xa, xb, ya, yb = z['rect']
        m = (GX >= xa) & (GX <= xb) & (GY >= ya) & (GY <= yb)
        v[m] = int(round(z['speed'] * 100))
        print(f"zone {z['name']}: {z['speed']} m/s -> value {int(round(z['speed'] * 100))}, {m.sum()} cells")
    pix = np.clip(np.round(255.0 * (1.0 - v / 100.0)), 0, 255).astype(np.uint8)
    pix = np.flipud(pix)                       # PGM row 0 = top (max y)
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, 'speed_mask.pgm'), 'wb') as f:
        f.write(f'P5\n# speed mask, value = speed*100\n{w} {h}\n255\n'.encode())
        f.write(pix.tobytes())
    with open(os.path.join(out_dir, 'speed_mask.yaml'), 'w') as f:
        f.write('image: speed_mask.pgm\nmode: scale\n'
                f'resolution: {res}\norigin: [{x0:.3f}, {y0:.3f}, 0.0]\n'
                'negate: 0\noccupied_thresh: 1.0\nfree_thresh: 0.0\n')
    # self-check: decode like nav2_map_server (scale mode)
    occ = 1.0 - np.flipud(pix) / 255.0
    dec = np.rint(occ * 100).astype(int)
    assert (dec == v).all(), 'mask round-trip mismatch'
    print('round-trip check ok')


if __name__ == '__main__':
    main(*sys.argv[1:4])
