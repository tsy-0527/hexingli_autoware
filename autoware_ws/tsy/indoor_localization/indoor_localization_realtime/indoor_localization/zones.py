"""Degeneracy zones: inflate the NDT pose covariance along a corridor axis.

A zone is a straight corridor segment from `start` to `end` (map frame, metres),
with a lateral half width. Inside the zone the longitudinal standard deviation
reported to the EKF is raised to at least `sigma_long`, so the EKF relies on
wheel/IMU odometry along the corridor while NDT still constrains lateral
position and yaw. `ramp` metres at both ends blend the inflation in and out.
"""
from dataclasses import dataclass
import math

import numpy as np


@dataclass
class Zone:
    name: str
    start: tuple
    end: tuple
    half_width: float
    sigma_long: float
    ramp: float

    def weight_and_axis(self, x, y):
        sx, sy = self.start
        ex, ey = self.end
        L = math.hypot(ex - sx, ey - sy)
        ux, uy = (ex - sx) / L, (ey - sy) / L
        s = (x - sx) * ux + (y - sy) * uy          # along axis
        d = -(x - sx) * uy + (y - sy) * ux         # lateral
        if s < 0 or s > L or abs(d) > self.half_width:
            return 0.0, math.atan2(uy, ux)
        w = min(s, L - s) / self.ramp if self.ramp > 0 else 1.0
        return float(min(1.0, max(0.0, w))), math.atan2(uy, ux)


def inflate_covariance(cov36, axis_yaw, sigma_long, weight):
    """Return a new 6x6 covariance (flat list) with longitudinal xy variance raised.

    Effective longitudinal sigma = max(original, weight * sigma_long). Lateral variance,
    z, roll, pitch and yaw entries are left unchanged.
    """
    C = np.array(cov36, dtype=float).reshape(6, 6).copy()
    if weight <= 0.0:
        return C.ravel().tolist(), float(math.sqrt(max(C[0, 0], 0.0)))
    c, s = math.cos(axis_yaw), math.sin(axis_yaw)
    R = np.array([[c, -s], [s, c]])               # axis frame -> map
    Ca = R.T @ C[:2, :2] @ R                      # in axis frame: [0]=long, [1]=lat
    long0 = max(Ca[0, 0], 1e-12)
    target = weight * sigma_long
    if target ** 2 <= long0:
        return C.ravel().tolist(), float(math.sqrt(long0))
    # congruence C' = M C M^T with M stretching only the longitudinal direction:
    # keeps the matrix positive (semi)definite and scales all cross terms consistently
    k = math.sqrt(target ** 2 / long0)
    M = np.eye(6)
    M[:2, :2] = R @ np.diag([k, 1.0]) @ R.T
    Cn = M @ C @ M.T
    return Cn.ravel().tolist(), float(target)


def zones_from_params(names, getter):
    zones = []
    for n in names:
        p = lambda k: getter(f'zones.{n}.{k}')  # noqa
        st, en = p('start'), p('end')
        zones.append(Zone(n, (st[0], st[1]), (en[0], en[1]), float(p('half_width')),
                          float(p('sigma_long')), float(p('ramp'))))
    return zones
