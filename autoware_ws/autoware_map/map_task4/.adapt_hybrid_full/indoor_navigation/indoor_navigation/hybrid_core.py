"""ROS-independent state machine used by the indoor/outdoor command arbiter.

交权规则（全部满足才把底盘交给目标控制器）：
  1. 先输出停车指令；
  2. 车速数据新鲜，且持续低于 stop_speed 达 stop_hold_time；
  3. 目标控制器的命令新鲜（节点活着）；
  4. 目标控制器此刻输出的是"停车"（|v| ≤ idle_speed）——先交权、再下发任务，
     避免交权瞬间车被一条旧路线/旧目标直接带走；
  5. 定位健康（两个方向都要求，NDT 是全程唯一定位源）。
"""
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
    idle_speed: float = 0.05


class ModeArbiter:
    """Require a stationary hold and a fresh, idle target command before handover."""

    def __init__(self, initial_mode=OUTDOOR, params=None):
        self.p = params or ArbiterParams()
        self.mode = initial_mode
        self.requested_mode = initial_mode
        self.phase = ACTIVE
        self.stopped_since = None
        self.last_source_time = {OUTDOOR: None, INDOOR: None}
        self.last_source_speed = {OUTDOOR: None, INDOOR: None}
        self.block_reason = ''

    def note_source(self, mode, now, cmd_speed=None):
        self.last_source_time[mode] = now
        self.last_source_speed[mode] = cmd_speed

    def request(self, mode):
        if mode not in (OUTDOOR, INDOOR):
            raise ValueError(f'unknown mode: {mode}')
        self.requested_mode = mode
        if mode != self.mode:
            self.phase = STOPPING
            self.stopped_since = None
        elif self.phase == STOPPING:
            # 切换途中撤回请求：仍然先停稳，再把权限还给原控制器
            self.stopped_since = None

    def source_fresh(self, mode, now):
        stamp = self.last_source_time[mode]
        return stamp is not None and now - stamp <= self.p.source_timeout

    def source_idle(self, mode):
        v = self.last_source_speed[mode]
        return v is None or abs(v) <= self.p.idle_speed

    def step(self, now, speed, target_healthy=True, speed_valid=True):
        """speed_valid=False（车速数据断流）时一律视为"可能在动"，不交权。"""
        if self.phase == ACTIVE:
            self.block_reason = ''
            return self.mode, False

        if not speed_valid:
            self.stopped_since = None
            self.block_reason = 'vehicle speed unavailable'
            return self.mode, True

        if abs(speed) > self.p.stop_speed:
            self.stopped_since = None
            self.block_reason = f'vehicle moving ({speed:.2f} m/s)'
            return self.mode, True

        if self.stopped_since is None:
            self.stopped_since = now
            self.block_reason = 'stop hold started'
            return self.mode, True

        target = self.requested_mode
        if now - self.stopped_since < self.p.stop_hold_time:
            self.block_reason = 'stop hold'
        elif not target_healthy:
            self.block_reason = 'localization not healthy'
        elif not self.source_fresh(target, now):
            self.block_reason = f'{target} command stale'
        elif not self.source_idle(target):
            self.block_reason = (f'{target} controller is commanding motion '
                                 f'({self.last_source_speed[target]:.2f} m/s); '
                                 'switch first, then send the route/goal')
        else:
            self.mode = target
            self.phase = ACTIVE
            self.stopped_since = None
            self.block_reason = ''
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


def check_manual_request(target, position, enter_polygon, exit_polygon):
    """手动请求的位置检查。返回 (ok, reason)。

    enter_polygon：室内区域（小）；exit_polygon：包住它的大区域。两者之间的带状区域就是门口交接带。
      - 请求进室内：车必须在 exit_polygon 内（门口或室内）；
      - 请求出室外：车必须在 enter_polygon 外（门口或室外）。
    围栏没配置时放行（返回 ok=True，reason 提示未检查）。
    """
    if len(enter_polygon) < 3 or len(exit_polygon) < 3:
        return True, 'switch zones not configured; position not checked'
    if position is None:
        return False, 'no vehicle position'
    x, y = position
    if target == INDOOR and not point_in_polygon(x, y, exit_polygon):
        return False, 'vehicle is not at the doorway/indoor area (outside exit polygon)'
    if target == OUTDOOR and point_in_polygon(x, y, enter_polygon):
        return False, 'vehicle is still inside the indoor area; drive to the doorway first'
    return True, 'position ok'


def health_ok(data):
    """indoor_localization/health JSON → 是否允许交权。WARN 允许（限速 0.5），ERROR/STALE 不允许。"""
    try:
        return data.get('level') not in ('ERROR', 'STALE') and float(data.get('speed_factor', 0.0)) > 0.0
    except (AttributeError, TypeError, ValueError):
        return False
