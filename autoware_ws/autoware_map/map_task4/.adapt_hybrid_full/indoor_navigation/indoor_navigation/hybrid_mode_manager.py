"""Fail-safe control ownership switch for outdoor Autoware and indoor Nav2.

The two controllers publish to private source topics.  This node is the only
publisher to the Yunle bridge topics.  Every ownership change is stop-first:
the current source is replaced by a zero command, the measured vehicle speed
must remain below the threshold for a hold period, and the target source must
have a fresh *idle* command before it receives authority.

修正要点（相对原版）：
  - 车速优先取底盘 VelocityReport（不依赖定位），数据断流视为"可能在动"，不交权；
  - 定位健康数据带超时；两个方向交权都要求定位健康（NDT 是全程唯一定位源）；
  - 目标控制器交权瞬间必须在输出停车：先交权，再给 Nav2 / Autoware 下任务；
  - 配置了围栏时，手动请求要在门口交接带才接受，不满足就返回 success=false；
  - 状态里给出当前卡在哪一步（block_reason）。
"""
import json
import os

import yaml

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool
from autoware_vehicle_msgs.msg import GearCommand

from .hybrid_core import ACTIVE, INDOOR, OUTDOOR, ArbiterParams, ModeArbiter
from .hybrid_core import check_manual_request, health_ok, point_in_polygon


def _load_control_type():
    try:
        from autoware_control_msgs.msg import Control
        return 'control', Control
    except ImportError:
        from autoware_auto_control_msgs.msg import AckermannControlCommand
        return 'legacy', AckermannControlCommand


def _load_velocity_type():
    try:
        from autoware_vehicle_msgs.msg import VelocityReport
        return VelocityReport
    except ImportError:
        try:
            from autoware_auto_vehicle_msgs.msg import VelocityReport
            return VelocityReport
        except ImportError:
            return None


class HybridModeManager(Node):
    def __init__(self):
        super().__init__('hybrid_mode_manager')
        d = self.declare_parameter
        d('initial_mode', OUTDOOR)
        d('outdoor_control_topic', '/hybrid/outdoor/control_cmd')
        d('outdoor_gear_topic', '/hybrid/outdoor/gear_cmd')
        d('indoor_control_topic', '/hybrid/indoor/control_cmd')
        d('indoor_gear_topic', '/hybrid/indoor/gear_cmd')
        d('output_control_topic', '/control/command/control_cmd')
        d('output_gear_topic', '/control/command/gear_cmd')
        d('odom_topic', '/localization/kinematic_state')
        d('velocity_topic', '/vehicle/status/velocity_status')   # 底盘实测车速（优先）
        d('indoor_health_topic', '/indoor_localization/health')
        d('rate_hz', 30.0)
        d('stop_speed', 0.03)
        d('stop_hold_time', 1.0)
        d('source_timeout', 0.5)
        d('idle_speed', 0.05)              # 目标控制器交权时输出速度上限
        d('speed_timeout', 0.5)            # 车速数据超过这个时间没更新 → 视为无效
        d('health_timeout', 1.0)           # 健康数据超过这个时间没更新 → 视为不健康
        d('require_health_for_outdoor', True)
        d('manual_requires_zone', True)    # 配了围栏时，手动请求必须在门口交接带
        d('automatic_switching', False)
        d('switch_zones_file', '')
        g = lambda k: self.get_parameter(k).value  # noqa: E731

        initial = str(g('initial_mode')).lower()
        if initial not in (OUTDOOR, INDOOR):
            raise ValueError('initial_mode must be outdoor or indoor')
        self.arbiter = ModeArbiter(initial, ArbiterParams(
            stop_speed=float(g('stop_speed')),
            stop_hold_time=float(g('stop_hold_time')),
            source_timeout=float(g('source_timeout')),
            idle_speed=float(g('idle_speed'))))
        self.kind, self.Control = _load_control_type()
        self.controls = {OUTDOOR: None, INDOOR: None}
        self.gears = {OUTDOOR: None, INDOOR: None}
        self.speed_timeout = float(g('speed_timeout'))
        self.health_timeout = float(g('health_timeout'))
        self.require_health_outdoor = bool(g('require_health_for_outdoor'))
        self.manual_requires_zone = bool(g('manual_requires_zone'))
        self.v_report = None          # (t, v)
        self.v_odom = None            # (t, v)
        self.position = None
        self.health = None            # (t, dict)
        self.automatic_switching = bool(g('automatic_switching'))
        self.enter_polygon = []
        self.exit_polygon = []
        zones_file = str(g('switch_zones_file'))
        if zones_file:
            if not os.path.isfile(zones_file):
                raise FileNotFoundError(f'switch_zones_file not found: {zones_file}')
            with open(zones_file, encoding='utf-8') as f:
                zones = yaml.safe_load(f) or {}
            self.enter_polygon = zones.get('indoor_enter_polygon') or []
            self.exit_polygon = zones.get('indoor_exit_polygon') or self.enter_polygon
        if self.automatic_switching and (
                len(self.enter_polygon) < 3 or len(self.exit_polygon) < 3):
            raise ValueError('automatic switching needs valid enter and exit polygons')
        if len(self.enter_polygon) < 3:
            self.get_logger().warn('switch zones not configured: manual requests are not position-checked')

        self.control_pub = self.create_publisher(self.Control, g('output_control_topic'), 10)
        self.gear_pub = self.create_publisher(GearCommand, g('output_gear_topic'), 10)
        self.state_pub = self.create_publisher(String, '~/state', 10)
        self.create_subscription(self.Control, g('outdoor_control_topic'),
                                 lambda m: self.on_control(OUTDOOR, m), 10)
        self.create_subscription(self.Control, g('indoor_control_topic'),
                                 lambda m: self.on_control(INDOOR, m), 10)
        self.create_subscription(GearCommand, g('outdoor_gear_topic'),
                                 lambda m: self.on_gear(OUTDOOR, m), 10)
        self.create_subscription(GearCommand, g('indoor_gear_topic'),
                                 lambda m: self.on_gear(INDOOR, m), 10)
        self.create_subscription(Odometry, g('odom_topic'), self.on_odom, 10)
        VelocityReport = _load_velocity_type()
        if VelocityReport is not None and g('velocity_topic'):
            self.create_subscription(VelocityReport, g('velocity_topic'), self.on_velocity, 10)
        else:
            self.get_logger().warn('VelocityReport not available; using localization speed only')
        self.create_subscription(String, g('indoor_health_topic'), self.on_indoor_health, 10)
        self.create_service(SetBool, '~/set_indoor', self.on_set_indoor)
        self.create_timer(1.0 / float(g('rate_hz')), self.tick)
        self.get_logger().info(
            f'hybrid control authority starts in {initial}; use ~/set_indoor')

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ inputs
    def cmd_speed(self, msg):
        try:
            if self.kind == 'control':
                return float(msg.longitudinal.velocity)
            return float(msg.longitudinal.speed)
        except AttributeError:
            return None

    def on_control(self, mode, msg):
        self.controls[mode] = msg
        self.arbiter.note_source(mode, self.now_sec(), self.cmd_speed(msg))

    def on_gear(self, mode, msg):
        self.gears[mode] = msg

    def on_odom(self, msg):
        self.v_odom = (self.now_sec(), float(msg.twist.twist.linear.x))
        self.position = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))

    def on_velocity(self, msg):
        self.v_report = (self.now_sec(), float(msg.longitudinal_velocity))

    def on_indoor_health(self, msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            data = {}
        self.health = (self.now_sec(), data)

    def speed(self, now):
        """(speed, valid, source)：底盘车速优先，其次定位速度。"""
        for src, v in (('chassis', self.v_report), ('localization', self.v_odom)):
            if v is not None and now - v[0] <= self.speed_timeout:
                return v[1], True, src
        return 0.0, False, 'none'

    def healthy(self, now):
        if self.health is None or now - self.health[0] > self.health_timeout:
            return False
        return health_ok(self.health[1])

    def target_healthy(self, now):
        if self.arbiter.requested_mode == OUTDOOR and not self.require_health_outdoor:
            return True
        return self.healthy(now)

    # ------------------------------------------------------------------ service
    def on_set_indoor(self, request, response):
        target = INDOOR if request.data else OUTDOOR
        if self.arbiter.phase == ACTIVE and self.arbiter.mode == target:
            response.success = True
            response.message = f'already {target}'
            return response
        if self.manual_requires_zone:
            ok, why = check_manual_request(target, self.position, self.enter_polygon, self.exit_polygon)
            if not ok:
                response.success = False
                response.message = f'{target} request rejected: {why}'
                self.get_logger().error(response.message)
                return response
            if 'not configured' in why:
                self.get_logger().warn(why)
        self.arbiter.request(target)
        response.success = True
        response.message = (f'{target} requested; stop-first handover started. '
                            f'Send the {"Nav2 station goal" if target == INDOOR else "Autoware route"} '
                            'AFTER /hybrid_mode_manager/state shows mode=' + target + ', phase=active')
        self.get_logger().warn(response.message)
        return response

    # ------------------------------------------------------------------ output
    def make_stop(self):
        msg = self.Control()
        stamp = self.get_clock().now().to_msg()
        if self.kind == 'control':
            msg.stamp = stamp
            msg.control_time = stamp
            msg.lateral.stamp = stamp
            msg.lateral.control_time = stamp
            msg.longitudinal.stamp = stamp
            msg.longitudinal.control_time = stamp
            msg.longitudinal.velocity = 0.0
            msg.longitudinal.acceleration = -0.5
            if hasattr(msg.longitudinal, 'is_defined_acceleration'):
                msg.longitudinal.is_defined_acceleration = True
        else:
            msg.stamp = stamp
            msg.lateral.stamp = stamp
            msg.longitudinal.stamp = stamp
            msg.longitudinal.speed = 0.0
            msg.longitudinal.acceleration = -0.5
        return msg

    def tick(self):
        now = self.now_sec()
        speed, speed_valid, speed_src = self.speed(now)
        if self.automatic_switching and self.position is not None and \
                self.arbiter.phase == ACTIVE:
            x, y = self.position
            if self.arbiter.mode == OUTDOOR and point_in_polygon(
                    x, y, self.enter_polygon):
                self.arbiter.request(INDOOR)
                self.get_logger().warn('entered indoor geofence; stop-first handover requested')
            elif self.arbiter.mode == INDOOR and not point_in_polygon(
                    x, y, self.exit_polygon):
                self.arbiter.request(OUTDOOR)
                self.get_logger().warn('left indoor geofence; stop-first handover requested')
        mode, force_stop = self.arbiter.step(
            now, speed, target_healthy=self.target_healthy(now), speed_valid=speed_valid)
        source = self.controls[mode]
        if force_stop or source is None or not self.arbiter.source_fresh(mode, now):
            self.control_pub.publish(self.make_stop())
        else:
            self.control_pub.publish(source)
            if self.gears[mode] is not None:
                self.gear_pub.publish(self.gears[mode])
        h = self.health[1] if self.health else {}
        self.state_pub.publish(String(data=json.dumps({
            'mode': self.arbiter.mode,
            'requested_mode': self.arbiter.requested_mode,
            'phase': self.arbiter.phase,
            'force_stop': force_stop,
            'block_reason': self.arbiter.block_reason,
            'speed': round(speed, 3),
            'speed_source': speed_src,
            'localization_healthy': self.healthy(now),
            'localization_level': h.get('level'),
            'indoor_healthy': self.healthy(now),          # 兼容原字段名
            'outdoor_fresh': self.arbiter.source_fresh(OUTDOOR, now),
            'indoor_fresh': self.arbiter.source_fresh(INDOOR, now),
            'automatic_switching': self.automatic_switching,
            'zones_configured': len(self.enter_polygon) >= 3,
        }, ensure_ascii=False)))


def main():
    rclpy.init()
    node = HybridModeManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
