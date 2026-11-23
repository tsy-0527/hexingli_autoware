"""Offline test of the room global localizer on the real map with simulated scans.

usage: python3 offline_sim_test.py <map_dir> [n_trials]
Simulates a 360-deg 2D scan (ray casting on the localization grid), converts it to a
3D cloud in base_link (z inside the height band), and runs the localizer.
"""
import math
import os
import sys
import time

import numpy as np
from scipy import ndimage

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from room_global_localizer.core import GridMap, Params, RoomGlobalLocalizer, _points_in_poly  # noqa


def raycast(occ, res, x0, y0, x, y, yaw, rng, n=1440, rmax=20.0, drop=0.3, noise=0.02, occl=None):
    ang = np.arange(n) * 2 * np.pi / n
    rs = np.arange(0.2, rmax, res * 0.5)
    wx = x + np.cos(ang + yaw)[:, None] * rs
    wy = y + np.sin(ang + yaw)[:, None] * rs
    ix = ((wx - x0) / res).astype(int)
    iy = ((wy - y0) / res).astype(int)
    ok = (ix >= 0) & (ix < occ.shape[1]) & (iy >= 0) & (iy < occ.shape[0])
    hit = np.zeros_like(ok)
    hit[ok] = occ[iy[ok], ix[ok]]
    has = hit.any(1)
    first = hit.argmax(1)
    keep = has.copy()
    if occl is not None:
        deg = np.degrees(ang)
        keep &= ~((deg > occl[0]) & (deg < occl[1]))
    r = rs[first[keep]] + rng.normal(0, noise, keep.sum())
    a = ang[keep]
    pts = np.c_[r * np.cos(a), r * np.sin(a), rng.uniform(0.7, 2.7, keep.sum())]
    return pts[rng.random(len(pts)) > drop]


def main():
    map_dir = sys.argv[1]
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 30
    loc = GridMap.load(os.path.join(map_dir, 'indoor_loc_2d.yaml'))
    nav = GridMap.load(os.path.join(map_dir, 'indoor_nav_2d.yaml'))
    p = Params()
    t0 = time.time()
    L = RoomGlobalLocalizer(loc, nav, p)
    print(f'init {time.time() - t0:.2f}s, allowed positions {L.allowed.sum()}')
    rng = np.random.default_rng(7)
    clear = ndimage.distance_transform_edt(~nav.occ) * nav.res
    poly = np.asarray(p.room_polygon)

    def sample(inside_room):
        while True:
            if inside_room:
                x, y = rng.uniform(poly[:, 0].min(), poly[:, 0].max()), rng.uniform(poly[:, 1].min(), poly[:, 1].max())
            else:  # corridor / hall poses -> must be rejected
                if rng.random() < 0.5:
                    x, y = 16.4 + rng.uniform(-0.6, 0.6), rng.uniform(-38, -15)
                else:
                    x, y = rng.uniform(-6, 22), -46.5 + rng.uniform(-1.5, 1.5)
            ix, iy = int((x - nav.x0) / nav.res), int((y - nav.y0) / nav.res)
            if nav.free[iy, ix] and clear[iy, ix] > 0.4:
                if not inside_room or _points_in_poly(np.array([x]), np.array([y]), poly)[0]:
                    return x, y, rng.uniform(-180, 180)

    suites = [('room, nominal', True, dict()),
              ('room, 60deg occlusion + 50% drop', True, dict(occl=True, drop=0.5)),
              ('OUTSIDE room (must reject)', False, dict())]
    for name, inside, kw in suites:
        ok = acc = wrong_acc = 0
        ratios, times = [], []
        for _ in range(n):
            x, y, yaw = sample(inside)
            occl = None
            if kw.get('occl'):
                a0 = rng.uniform(0, 360)
                occl = (a0, a0 + 60)
            pts = raycast(loc.occ, loc.res, loc.x0, loc.y0, x, y, math.radians(yaw), rng,
                          drop=kw.get('drop', 0.3), occl=occl)
            t = time.time()
            r = L.localize(pts)
            times.append(time.time() - t)
            if 'ratio' in r:
                ratios.append(r['ratio'])
            correct = 'x' in r and math.hypot(r['x'] - x, r['y'] - y) < 0.15 and abs((r['yaw_deg'] - yaw + 180) % 360 - 180) < 2.0
            ok += correct
            acc += r['accepted']
            wrong_acc += r['accepted'] and not correct
        print(f'{name}: top1-correct {ok}/{n}, accepted {acc}/{n}, WRONG-accepted {wrong_acc}, '
              f'ratio mean {np.mean(ratios):.2f} max {np.max(ratios):.2f}, time mean {np.mean(times):.2f}s max {np.max(times):.2f}s')


if __name__ == '__main__':
    main()
