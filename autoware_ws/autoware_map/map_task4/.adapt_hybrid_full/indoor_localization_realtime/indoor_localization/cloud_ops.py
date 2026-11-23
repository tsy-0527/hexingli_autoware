"""Point cloud operations for MID-360 preprocessing (pure numpy, no ROS).

Conventions
-----------
- Points are N x 3 float64 arrays.
- base_link: x forward, y left, z up, origin on the floor under the rear axle.
- Deskew target time t_ref: every point is moved to where it would have been
  measured if the whole scan had been taken at t_ref (the scan end time).
"""
import math

import numpy as np


def point_times_to_sec(t_field, header_stamp_sec):
    """Convert a per-point time field to absolute seconds.

    Handles the common encodings automatically:
      - absolute ns (livox_ros_driver2 'timestamp', float64, ~1.7e18)
      - absolute s
      - relative to header stamp, in s, ms, us or ns
    """
    t = np.asarray(t_field, dtype=np.float64)
    if t.size == 0:
        return t
    m = float(np.nanmedian(np.abs(t)))
    if m > 1e17:                       # absolute ns
        return t * 1e-9
    if m > 1e8:                        # absolute s
        return t
    # relative offsets: guess unit from the spread (one scan lasts ~0.1 s)
    spread = float(np.nanmax(t) - np.nanmin(t)) if t.size > 1 else 0.0
    if spread > 1e6:                   # ns
        scale = 1e-9
    elif spread > 1e3:                 # us
        scale = 1e-6
    elif spread > 1.0:                 # ms
        scale = 1e-3
    else:                              # s
        scale = 1.0
    return header_stamp_sec + t * scale


def transform(points, R, t):
    return points @ R.T + t


def crop(points, *, r_min, r_max, z_min, z_max, self_box, extra=None):
    """Keep points by horizontal range, height (base_link z) and outside the vehicle box.

    self_box = [xmin, xmax, ymin, ymax, zmin, zmax] in base_link.
    Returns (points, mask) so other per-point arrays can be filtered the same way.
    """
    p = points
    r = np.hypot(p[:, 0], p[:, 1])
    b = self_box
    in_self = ((p[:, 0] > b[0]) & (p[:, 0] < b[1]) & (p[:, 1] > b[2]) & (p[:, 1] < b[3])
               & (p[:, 2] > b[4]) & (p[:, 2] < b[5]))
    m = np.isfinite(p).all(1) & (r > r_min) & (r < r_max) & (p[:, 2] > z_min) & (p[:, 2] < z_max) & ~in_self
    return p[m], m


def deskew_planar(points, t_points, t_ref, vx, wz):
    """Constant-velocity planar deskew in base_link.

    A point measured at time t_i (dt = t_i - t_ref <= 0) is expressed in the base_link
    frame at t_i. The base_link pose at t_i relative to base_link at t_ref is obtained by
    integrating (vx, wz) over dt; the point is mapped with that pose.
    """
    dt = t_points - t_ref
    th = wz * dt
    if abs(wz) > 1e-6:
        tx = vx / wz * np.sin(th)
        ty = vx / wz * (1.0 - np.cos(th))
    else:
        tx = vx * dt
        ty = np.zeros_like(dt)
    c, s = np.cos(th), np.sin(th)
    x = c * points[:, 0] - s * points[:, 1] + tx
    y = s * points[:, 0] + c * points[:, 1] + ty
    return np.column_stack([x, y, points[:, 2]])


def voxel_downsample(points, leaf, intensity=None):
    """Centroid voxel filter. Returns (points, intensity)."""
    if len(points) == 0:
        return points, intensity
    key = np.floor(points / leaf).astype(np.int64)
    _, inv, counts = np.unique(key, axis=0, return_inverse=True, return_counts=True)
    inv = inv.ravel()
    out = np.zeros((len(counts), 3))
    np.add.at(out, inv, points)
    out /= counts[:, None]
    oi = None
    if intensity is not None:
        oi = np.zeros(len(counts))
        np.add.at(oi, inv, intensity)
        oi /= counts
    return out, oi


def quat_to_mat(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_from_quat(x, y, z, w):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
