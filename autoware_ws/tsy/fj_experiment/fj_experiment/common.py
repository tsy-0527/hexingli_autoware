"""Pure-Python helpers shared by the nodes and the offline scripts (no ROS imports here).

- GilbertElliott: burst (two-state Markov) dropout generator used by the fault injector.
- ExistenceCusum: improvement 1, CUSUM on per-frame detect/miss observations.
- Geometry helpers for collision / clearance checks.
"""
import math
import random


# ---------------------------------------------------------------------------
# Dropout models
# ---------------------------------------------------------------------------
class GilbertElliott:
    """Two-state Markov chain: GOOD (always detected) / BAD (always missed).

    p_enter: P(GOOD -> BAD) per frame;  p_exit: P(BAD -> GOOD) per frame.
    Stationary miss rate = p_enter / (p_enter + p_exit); mean burst length = 1 / p_exit.
    """

    def __init__(self, p_enter, p_exit, rng):
        self.p_enter, self.p_exit, self.rng = p_enter, p_exit, rng
        self.bad = False

    def step(self):
        """Advance one frame; return True if the object is DETECTED this frame."""
        if self.bad:
            if self.rng.random() < self.p_exit:
                self.bad = False
        else:
            if self.rng.random() < self.p_enter:
                self.bad = True
        return not self.bad


class IidDropout:
    def __init__(self, p_miss, rng):
        self.p_miss, self.rng = p_miss, rng

    def step(self):
        return self.rng.random() >= self.p_miss


class Ar1:
    """Zero-mean AR(1) noise with stationary std sigma and lag-1 correlation rho."""

    def __init__(self, sigma, rho, rng):
        self.sigma, self.rho, self.rng = sigma, rho, rng
        self.e = rng.gauss(0.0, sigma) if sigma > 0 else 0.0

    def step(self):
        if self.sigma <= 0:
            return 0.0
        self.e = self.rho * self.e + math.sqrt(max(0.0, 1 - self.rho ** 2)) * self.rng.gauss(0.0, self.sigma)
        return self.e


# ---------------------------------------------------------------------------
# Improvement 1: existence CUSUM
# ---------------------------------------------------------------------------
class ExistenceCusum:
    """CUSUM for the change "object exists" (H0, pre-change) -> "object is gone" (H1, post-change).

    Observation per frame: detected (True) / missed (False).
      H1 (gone):   P(detected) = p_fa   (false alarm at that spot)
      H0 (exists): two-state Markov detection model (GOOD/BAD); P(detected | GOOD) = pd_good,
                   P(detected | BAD) = pd_bad. With p_enter = 0 this reduces to iid Bernoulli(pd_good).
    The pre-change predictive probability uses an HMM forward filter, so a miss right after a
    detection is strong evidence (bursts are rare to start), while further misses inside a burst
    are weak evidence (bursts tend to continue).

    S_n = max(0, S_{n-1} + ln f1(x_n)/f0(x_n | past));  alarm (delete) when S_n >= h.
    """

    def __init__(self, p_fa=0.05, pd_good=0.99, pd_bad=0.0, p_enter=0.066, p_exit=0.154, h=6.0):
        self.p_fa, self.pd_good, self.pd_bad = p_fa, pd_good, pd_bad
        self.p_enter, self.p_exit, self.h = p_enter, p_exit, h
        self.reset()

    def reset(self):
        self.S = 0.0
        self.b = 0.0  # P(BAD state); a freshly confirmed track has just been seen -> start in GOOD

    def llr(self, detected):
        """Log-likelihood ratio ln f1/f0 for this frame (does not change state)."""
        b_pred = self.b * (1 - self.p_exit) + (1 - self.b) * self.p_enter
        p0_det = (1 - b_pred) * self.pd_good + b_pred * self.pd_bad
        p0_det = min(max(p0_det, 1e-6), 1 - 1e-6)
        if detected:
            return math.log(self.p_fa / p0_det)
        return math.log((1 - self.p_fa) / (1 - p0_det))

    def update(self, detected):
        b_pred = self.b * (1 - self.p_exit) + (1 - self.b) * self.p_enter
        l = self.llr(detected)
        # posterior of BAD given the observation (H0 filter)
        lik_bad = self.pd_bad if detected else 1 - self.pd_bad
        lik_good = self.pd_good if detected else 1 - self.pd_good
        num = b_pred * lik_bad
        den = num + (1 - b_pred) * lik_good
        self.b = num / den if den > 0 else b_pred
        self.S = max(0.0, self.S + l)
        return self.S

    @property
    def alarmed(self):
        return self.S >= self.h


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------
def yaw_from_quat(q):
    """q: object with x,y,z,w or dict."""
    if isinstance(q, dict):
        x, y, z, w = q['x'], q['y'], q['z'], q['w']
    else:
        x, y, z, w = q.x, q.y, q.z, q.w
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def rect_corners(cx, cy, yaw, front, rear, half_w):
    """Rectangle around a reference point: `front` m ahead, `rear` m behind, +-half_w sideways."""
    c, s = math.cos(yaw), math.sin(yaw)
    pts = []
    for dx, dy in ((front, half_w), (front, -half_w), (-rear, -half_w), (-rear, half_w)):
        pts.append((cx + c * dx - s * dy, cy + s * dx + c * dy))
    return pts


def _proj(poly, ax):
    vals = [p[0] * ax[0] + p[1] * ax[1] for p in poly]
    return min(vals), max(vals)


def polys_intersect(a, b):
    """Separating-axis test for two convex polygons."""
    for poly in (a, b):
        n = len(poly)
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]
            ax = (y1 - y2, x2 - x1)
            amin, amax = _proj(a, ax)
            bmin, bmax = _proj(b, ax)
            if amax < bmin or bmax < amin:
                return False
    return True


def _pt_seg(p, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L = dx * dx + dy * dy
    t = 0.0 if L == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def poly_distance(a, b):
    if polys_intersect(a, b):
        return 0.0
    d = float('inf')
    for P, Q in ((a, b), (b, a)):
        for p in P:
            for i in range(len(Q)):
                d = min(d, _pt_seg(p, Q[i], Q[(i + 1) % len(Q)]))
    return d


def seeded_rng(seed, salt):
    return random.Random(f'{seed}-{salt}')
