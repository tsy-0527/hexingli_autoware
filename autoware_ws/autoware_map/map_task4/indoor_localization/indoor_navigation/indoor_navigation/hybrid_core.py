"""ROS-independent state machine used by the indoor/outdoor command arbiter."""
from dataclasses import dataclass


OUTDOOR = 'outdoor'
INDOOR = 'indoor'
STOPPING = 'stopping'
ACTIVE = 'active'


@dataclass
class ArbiterParams:
    stop_speed: float = 0.03
    stop_hold_time: float = 1.0
    source_timeout: float = 0.5


class ModeArbiter:
    """Require a stationary hold and a fresh target command before handover."""

    def __init__(self, initial_mode=OUTDOOR, params=None):
        self.p = params or ArbiterParams()
        self.mode = initial_mode
        self.requested_mode = initial_mode
        self.phase = ACTIVE
        self.stopped_since = None
        self.last_source_time = {OUTDOOR: None, INDOOR: None}

    def note_source(self, mode, now):
        self.last_source_time[mode] = now

    def request(self, mode):
        if mode not in (OUTDOOR, INDOOR):
            raise ValueError(f'unknown mode: {mode}')
        self.requested_mode = mode
        if mode != self.mode:
            self.phase = STOPPING
            self.stopped_since = None

    def source_fresh(self, mode, now):
        stamp = self.last_source_time[mode]
        return stamp is not None and now - stamp <= self.p.source_timeout

    def step(self, now, speed, target_healthy=True):
        if self.phase == ACTIVE:
            return self.mode, False

        if abs(speed) > self.p.stop_speed:
            self.stopped_since = None
            return self.mode, True

        if self.stopped_since is None:
            self.stopped_since = now
            return self.mode, True

        held = now - self.stopped_since >= self.p.stop_hold_time
        if held and target_healthy and self.source_fresh(self.requested_mode, now):
            self.mode = self.requested_mode
            self.phase = ACTIVE
            self.stopped_since = None
            return self.mode, False
        return self.mode, True


def point_in_polygon(x, y, polygon):
    """Ray-casting inclusion test; boundary points are treated as inside."""
    if len(polygon) < 3:
        return False
    inside = False
    j = len(polygon) - 1
    for i, (xi, yi) in enumerate(polygon):
        xj, yj = polygon[j]
        cross = (yi > y) != (yj > y)
        if cross:
            edge_x = (xj - xi) * (y - yi) / (yj - yi) + xi
            if x <= edge_x:
                inside = not inside
        j = i
    return inside

