"""Room-constrained 2D global localization (no ROS dependency).

Pipeline:
  1. scan (N x 3, in base_link) -> height band / range crop -> 2D points
  2. coarse search: for every yaw, FFT cross-correlation of the scan with a
     likelihood field over the room window (res 0.1 m, yaw step 2 deg)
  3. top-K distinct candidates -> local refinement (0.05 m, 0.5 deg)
  4. decision: best score high enough AND clearly better than the best
     distinct alternative (typically the 180-deg flipped pose)
"""
from dataclasses import dataclass, field
import math
import os

import numpy as np
from scipy import ndimage, signal


# ----------------------------------------------------------------------------
# map loading
# ----------------------------------------------------------------------------
def _read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    # P5 header: magic, (comments), width height, maxval
    tokens, pos = [], 0
    while len(tokens) < 4:
        line_end = data.index(b'\n', pos)
        line = data[pos:line_end]
        pos = line_end + 1
        if line.startswith(b'#'):
            continue
        tokens += line.split()
    assert tokens[0] == b'P5', 'only binary PGM (P5) supported'
    w, h = int(tokens[1]), int(tokens[2])
    img = np.frombuffer(data[pos:pos + w * h], np.uint8).reshape(h, w)
    return np.flipud(img)  # row 0 = lowest y (map origin)


def _read_yaml_simple(path):
    out = {}
    with open(path) as f:
        for line in f:
            line = line.split('#')[0].strip()
            if not line or ':' not in line:
                continue
            k, v = line.split(':', 1)
            v = v.strip()
            if v.startswith('['):
                out[k.strip()] = [float(t) for t in v.strip('[]').split(',')]
            else:
                try:
                    out[k.strip()] = float(v)
                except ValueError:
                    out[k.strip()] = v
    return out


@dataclass
class GridMap:
    occ: np.ndarray        # bool, [iy, ix]
    free: np.ndarray       # bool
    res: float
    x0: float
    y0: float

    @staticmethod
    def load(yaml_path):
        meta = _read_yaml_simple(yaml_path)
        img = _read_pgm(os.path.join(os.path.dirname(yaml_path), meta['image']))
        p = img.astype(float) / 255.0
        if int(meta.get('negate', 0)) == 0:
            p = 1.0 - p
        occ = p > meta.get('occupied_thresh', 0.65)
        free = p < meta.get('free_thresh', 0.25)
        return GridMap(occ, free, float(meta['resolution']),
                       float(meta['origin'][0]), float(meta['origin'][1]))


# ----------------------------------------------------------------------------
# parameters
# ----------------------------------------------------------------------------
@dataclass
class Params:
    room_polygon: list = field(default_factory=lambda: [
        [18.6, -13.1], [27.05, -13.1], [27.05, -7.7], [18.6, -7.7]])
    min_clearance: float = 0.30     # base_link must be this far from obstacles (nav map)
    # scan preprocessing (base_link frame, base_link z = floor)
    z_min: float = 0.60
    z_max: float = 2.85
    r_min: float = 0.30
    r_max: float = 20.0
    self_box: list = field(default_factory=lambda: [-0.30, 0.80, -0.40, 0.40])  # xmin xmax ymin ymax
    voxel: float = 0.05
    max_points: int = 1500
    # search
    sigma: float = 0.10
    coarse_res: float = 0.10
    coarse_yaw_step_deg: float = 2.0
    top_k: int = 6
    nms_dist: float = 0.6
    nms_yaw_deg: float = 15.0
    refine_res: float = 0.025
    refine_window: float = 0.15
    refine_yaw_step_deg: float = 0.5
    refine_yaw_window_deg: float = 3.0
    # decision
    distinct_dist: float = 1.0
    distinct_yaw_deg: float = 20.0
    min_score: float = 0.70         # mean likelihood of scan points, 0..1
    max_ratio: float = 0.85         # second_best / best must be below this
    min_points: int = 200


# ----------------------------------------------------------------------------
# localizer
# ----------------------------------------------------------------------------
class RoomGlobalLocalizer:
    def __init__(self, loc_map: GridMap, nav_map: GridMap, params: Params):
        self.p = params
        self.loc = loc_map
        self.nav = nav_map
        dist = ndimage.distance_transform_edt(~loc_map.occ) * loc_map.res
        self.field_fine = np.exp(-dist ** 2 / (2 * params.sigma ** 2))
        k = max(1, int(round(params.coarse_res / loc_map.res)))
        f = self.field_fine
        f = f[:f.shape[0] // k * k, :f.shape[1] // k * k]
        self.field_coarse = f.reshape(f.shape[0] // k, k, f.shape[1] // k, k).max((1, 3))
        self.coarse_res = loc_map.res * k
        self._build_allowed()

    # positions allowed for base_link: inside room polygon, free, clearance
    def _build_allowed(self):
        p, nav = self.p, self.nav
        poly = np.asarray(p.room_polygon, float)
        self.win = (poly[:, 0].min(), poly[:, 0].max(), poly[:, 1].min(), poly[:, 1].max())
        cx = np.arange(self.win[0], self.win[1] + 1e-9, self.coarse_res)
        cy = np.arange(self.win[2], self.win[3] + 1e-9, self.coarse_res)
        self.cx, self.cy = cx, cy
        GX, GY = np.meshgrid(cx, cy)
        inside = _points_in_poly(GX.ravel(), GY.ravel(), poly).reshape(GY.shape)
        clear = ndimage.distance_transform_edt(~nav.occ) * nav.res
        ix = ((GX - nav.x0) / nav.res).astype(int).clip(0, nav.occ.shape[1] - 1)
        iy = ((GY - nav.y0) / nav.res).astype(int).clip(0, nav.occ.shape[0] - 1)
        ok = nav.free[iy, ix] & (clear[iy, ix] >= p.min_clearance)
        self.allowed = inside & ok

    def preprocess(self, pts_base, rng=None):
        p = self.p
        pts = np.asarray(pts_base, float)
        pts = pts[np.isfinite(pts).all(1)]
        r = np.hypot(pts[:, 0], pts[:, 1])
        sb = p.self_box
        in_self = (pts[:, 0] > sb[0]) & (pts[:, 0] < sb[1]) & (pts[:, 1] > sb[2]) & (pts[:, 1] < sb[3])
        m = (pts[:, 2] > p.z_min) & (pts[:, 2] < p.z_max) & (r > p.r_min) & (r < p.r_max) & ~in_self
        xy = pts[m, :2]
        if len(xy) == 0:
            return xy
        key = np.floor(xy / p.voxel).astype(np.int64)
        _, idx = np.unique(key, axis=0, return_index=True)
        xy = xy[idx]
        if len(xy) > p.max_points:
            rng = rng or np.random.default_rng(0)
            xy = xy[rng.choice(len(xy), p.max_points, replace=False)]
        return xy

    # ---- scoring helpers ----
    def _score_fine(self, xy, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        wx = c * xy[:, 0] - s * xy[:, 1] + x
        wy = s * xy[:, 0] + c * xy[:, 1] + y
        ix = ((wx - self.loc.x0) / self.loc.res).astype(int)
        iy = ((wy - self.loc.y0) / self.loc.res).astype(int)
        H, W = self.field_fine.shape
        ok = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
        v = np.zeros(len(xy))
        v[ok] = self.field_fine[iy[ok], ix[ok]]
        return v.mean()

    def _coarse(self, xy):
        res = self.coarse_res
        pad = int(math.ceil(self.p.r_max / res)) + 2
        f = np.pad(self.field_coarse, pad)
        fx0, fy0 = self.loc.x0 - pad * res, self.loc.y0 - pad * res
        nyaw = int(round(360.0 / self.p.coarse_yaw_step_deg))
        S = np.full((nyaw, len(self.cy), len(self.cx)), -1.0)
        jx0 = int(round((self.win[0] - fx0) / res))
        jy0 = int(round((self.win[2] - fy0) / res))
        for k in range(nyaw):
            t = math.radians(k * self.p.coarse_yaw_step_deg)
            c, s = math.cos(t), math.sin(t)
            px = c * xy[:, 0] - s * xy[:, 1]
            py = s * xy[:, 0] + c * xy[:, 1]
            ox = np.round(px / res).astype(int)
            oy = np.round(py / res).astype(int)
            kx0, ky0 = ox.min(), oy.min()
            K = np.zeros((oy.max() - ky0 + 1, ox.max() - kx0 + 1))
            np.add.at(K, (oy - ky0, ox - kx0), 1.0)
            sx0, sy0 = jx0 + kx0, jy0 + ky0
            sub = f[sy0:sy0 + len(self.cy) + K.shape[0] - 1, sx0:sx0 + len(self.cx) + K.shape[1] - 1]
            C = signal.correlate(sub, K, mode='valid', method='fft') / len(xy)
            C = C[:len(self.cy), :len(self.cx)]
            S[k] = np.where(self.allowed, C, -1.0)
        return S

    def _topk(self, S):
        p = self.p
        yaws = np.arange(S.shape[0]) * p.coarse_yaw_step_deg
        flat = np.argsort(S.ravel())[::-1]
        cands = []
        for f in flat:
            k, iy, ix = np.unravel_index(f, S.shape)
            sc = S[k, iy, ix]
            if sc <= 0:
                break
            x, y, yaw = self.cx[ix], self.cy[iy], yaws[k]
            if all(math.hypot(x - c[0], y - c[1]) > p.nms_dist or _ang(yaw - c[2]) > p.nms_yaw_deg for c in cands):
                cands.append((x, y, yaw, sc))
                if len(cands) >= p.top_k:
                    break
        return cands

    def _refine(self, xy, x, y, yaw_deg):
        p = self.p
        best = (-1, x, y, yaw_deg)
        # yaw first at coarse position, then xy, then joint small grid
        for dyaw in np.arange(-p.refine_yaw_window_deg, p.refine_yaw_window_deg + 1e-9, p.refine_yaw_step_deg):
            for dx in np.arange(-p.refine_window, p.refine_window + 1e-9, p.refine_res * 2):
                for dy in np.arange(-p.refine_window, p.refine_window + 1e-9, p.refine_res * 2):
                    s = self._score_fine(xy, x + dx, y + dy, math.radians(yaw_deg + dyaw))
                    if s > best[0]:
                        best = (s, x + dx, y + dy, yaw_deg + dyaw)
        s0, x1, y1, w1 = best
        for dyaw in np.arange(-p.refine_yaw_step_deg, p.refine_yaw_step_deg + 1e-9, p.refine_yaw_step_deg / 2):
            for dx in np.arange(-p.refine_res, p.refine_res + 1e-9, p.refine_res):
                for dy in np.arange(-p.refine_res, p.refine_res + 1e-9, p.refine_res):
                    s = self._score_fine(xy, x1 + dx, y1 + dy, math.radians(w1 + dyaw))
                    if s > best[0]:
                        best = (s, x1 + dx, y1 + dy, w1 + dyaw)
        return best

    def localize(self, pts_base):
        p = self.p
        xy = self.preprocess(pts_base)
        res = dict(accepted=False, reason='', n_points=int(len(xy)))
        if len(xy) < p.min_points:
            res['reason'] = f'too few points after preprocessing ({len(xy)} < {p.min_points})'
            return res
        S = self._coarse(xy)
        cands = self._topk(S)
        if not cands:
            res['reason'] = 'no candidate inside room'
            return res
        refined = sorted([self._refine(xy, c[0], c[1], c[2]) for c in cands], reverse=True)
        best = refined[0]
        alts = [r for r in refined[1:] if math.hypot(r[1] - best[1], r[2] - best[2]) > p.distinct_dist
                or _ang(r[3] - best[3]) > p.distinct_yaw_deg]
        # always evaluate the 180-deg flipped pose explicitly (room symmetry)
        poly = np.asarray(p.room_polygon, float)
        c = poly.mean(0)
        fx, fy = 2 * c[0] - best[1], 2 * c[1] - best[2]
        flip = self._refine(xy, fx, fy, (best[3] + 180.0) % 360.0)
        alts.append(flip)
        second = max(alts, key=lambda r: r[0])
        ratio = second[0] / best[0] if best[0] > 0 else 1.0
        res.update(
            x=float(best[1]), y=float(best[2]), yaw_deg=float(_wrap(best[3])),
            score=float(best[0]), second_score=float(second[0]), ratio=float(ratio),
            second_pose=[float(second[1]), float(second[2]), float(_wrap(second[3]))],
            candidates=[[float(r[1]), float(r[2]), float(_wrap(r[3])), float(r[0])] for r in refined])
        if best[0] < p.min_score:
            res['reason'] = f'score {best[0]:.3f} < min_score {p.min_score}'
        elif ratio > p.max_ratio:
            res['reason'] = f'ambiguous: second/best {ratio:.3f} > max_ratio {p.max_ratio}'
        else:
            res['accepted'] = True
            res['reason'] = 'ok'
        return res


def _ang(d):
    return abs((d + 180.0) % 360.0 - 180.0)


def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def choose_ndt_winner(reliable, candidates, pos_tol, yaw_tol):
    """Select an NDT result without treating correlated seeds as votes.

    Candidate 0 is the highest-scoring 2D hypothesis.  If it is NDT-reliable,
    its refined pose wins.  Lower-ranked candidates are diagnostic only. They
    cannot safely replace the primary in a symmetric room because the align
    service does not expose a comparable numeric NDT score.
    """
    clusters = []
    for item in reliable:
        for cluster in clusters:
            anchor = cluster[0]
            distance = math.hypot(item['x'] - anchor['x'], item['y'] - anchor['y'])
            yaw_distance = abs(
                (item['yaw'] - anchor['yaw'] + math.pi) % (2.0 * math.pi) - math.pi)
            if distance <= pos_tol and yaw_distance <= yaw_tol:
                cluster.append(item)
                break
        else:
            clusters.append([item])

    primary = next((item for item in reliable if item['seed_index'] == 0), None)
    if primary is not None:
        cluster = next(group for group in clusters if primary in group)
        return primary, 'primary_reliable', len(cluster), len(clusters)
    return None, 'primary_not_reliable', 0, len(clusters)


def _points_in_poly(x, y, poly):
    inside = np.zeros(len(x), bool)
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        cond = ((yi > y) != (yj > y)) & (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi)
        inside ^= cond
        j = i
    return inside
