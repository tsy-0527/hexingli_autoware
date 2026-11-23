"""Turn Nav2 (v, wz) commands into safe Ackermann commands (speed, front-wheel angle).

Pure Python, no ROS. One `step()` per control tick.

What it does, in order:
  1. command timeout          -> brake to zero, keep wheel angle
  2. curvature                -> delta = atan(L * wz / v); at |v|~0 keep the previous curvature
  3. limits                   -> |delta| <= max_steer, forward / reverse speed limits, health speed factor
  4. direction change         -> must come to a full stop and stay stopped for `stop_hold_time`
  5. steering rate limit      -> the wheels need time to turn; while they lag, speed is reduced
                                 so the vehicle does not leave the planned arc (and waits at standstill
                                 until the wheels are close to the target before starting to move)
  6. acceleration limits      -> on the speed, curvature is preserved
"""
from dataclasses import dataclass
import math


@dataclass
class AckermannParams:
    wheelbase: float = 0.475
    max_steer: float = math.radians(21.0)
    max_steer_rate: float = math.radians(30.0)     # TODO(chassis): measure 0 -> 21 deg time
    max_speed_fwd: float = 0.5
    max_speed_rev: float = 0.2
    max_accel: float = 0.3
    max_decel: float = 0.5
    v_eps: float = 0.02                            # below this |v| counts as "stopped"
    stop_hold_time: float = 0.5                    # standstill time before changing direction
    start_steer_tolerance: float = math.radians(4.0)   # at standstill, wait until wheels are this close
    steer_slowdown_err: float = math.radians(8.0)  # wheel-angle error at which speed is scaled to min factor
    steer_slowdown_min_factor: float = 0.3
    cmd_timeout: float = 0.5


class AckermannShaper:
    def __init__(self, p: AckermannParams):
        self.p = p
        self.v = 0.0            # last output speed
        self.delta = 0.0        # last output wheel angle
        self.kappa = 0.0        # last commanded curvature
        self.direction = 0      # +1 forward, -1 reverse, 0 none yet
        self.stopped_since = None
        self.last_cmd_t = None

    def on_cmd(self, t):
        self.last_cmd_t = t

    def step(self, t, dt, v_cmd, wz_cmd, speed_factor=1.0, v_meas=None):
        p = self.p
        timed_out = self.last_cmd_t is None or (t - self.last_cmd_t) > p.cmd_timeout
        if timed_out:
            v_cmd, wz_cmd = 0.0, 0.0

        # 2) curvature
        if abs(v_cmd) > p.v_eps:
            self.kappa = wz_cmd / v_cmd
        delta_des = math.atan(p.wheelbase * self.kappa)
        delta_des = max(-p.max_steer, min(p.max_steer, delta_des))

        # 3) speed limits
        v_des = max(-p.max_speed_rev, min(p.max_speed_fwd, v_cmd)) * max(0.0, min(1.0, speed_factor))
        want_dir = 0 if abs(v_des) <= p.v_eps else (1 if v_des > 0 else -1)

        # 4) direction change: stop, hold, then go
        v_now = self.v if v_meas is None else v_meas
        is_stopped = abs(self.v) <= p.v_eps and abs(v_now) <= p.v_eps
        if is_stopped:
            self.stopped_since = self.stopped_since if self.stopped_since is not None else t
        else:
            self.stopped_since = None
        if want_dir != 0 and self.direction != 0 and want_dir != self.direction:
            if not is_stopped or (t - self.stopped_since) < p.stop_hold_time:
                v_des = 0.0                       # brake / hold before reversing direction
                want_dir = 0
            else:
                self.direction = want_dir
        elif want_dir != 0 and self.direction == 0:
            self.direction = want_dir

        # 5) steering rate limit + speed reduction while the wheels lag
        step = p.max_steer_rate * dt
        self.delta += max(-step, min(step, delta_des - self.delta))
        err = abs(delta_des - self.delta)
        if is_stopped and err > p.start_steer_tolerance:
            v_des = 0.0                           # pre-steer at standstill
        elif err > 0:
            f = 1.0 - (1.0 - p.steer_slowdown_min_factor) * min(1.0, err / p.steer_slowdown_err)
            v_des *= f

        # 6) acceleration limit (speed only; wheel angle already set -> curvature preserved)
        dv = v_des - self.v
        speeding_up = abs(v_des) > abs(self.v) and (v_des * self.v >= 0)
        lim = (p.max_accel if speeding_up else p.max_decel) * dt
        self.v += max(-lim, min(lim, dv))
        if abs(self.v) < 1e-4:
            self.v = 0.0
        accel = (max(-lim, min(lim, dv))) / dt if dt > 0 else 0.0
        return dict(speed=self.v, steer=self.delta, accel=accel, steer_target=delta_des,
                    timed_out=timed_out, direction=self.direction)
