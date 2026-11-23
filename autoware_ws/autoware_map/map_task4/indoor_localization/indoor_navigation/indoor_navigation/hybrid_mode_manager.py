"""Fail-safe control ownership switch for outdoor Autoware and indoor Nav2.

The two controllers publish to private source topics.  This node is the only
publisher to the Yunle bridge topics.  Every ownership change is stop-first:
the current source is replaced by a zero command, the measured vehicle speed
must remain below the threshold for a hold period, and the target source must
have a fresh command before it receives authority.
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
from .hybrid_core import point_in_polygon


def _load_control_type():
    try:
        from autoware_control_msgs.msg import Control
        return 'control', Control
    except ImportError:
        from autoware_auto_control_msgs.msg import AckermannControlCommand
        return 'legacy', AckermannControlCommand


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
        d('indoor_health_topic', '/indoor_localization/health')
        d('rate_hz', 30.0)
        d('stop_speed', 0.03)
        d('stop_hold_time', 1.0)
        d('source_timeout', 0.5)
        d('automatic_switching', False)
        d('switch_zones_file', '')
        g = lambda k: self.get_parameter(k).value  # noqa: E731

        initial = str(g('initial_mode')).lower()
        if initial not in (OUTDOOR, INDOOR):
            raise ValueError('initial_mode must be outdoor or indoor')
        self.arbiter = ModeArbiter(initial, ArbiterParams(
            stop_speed=float(g('stop_speed')),
            stop_hold_time=float(g('stop_hold_time')),
            source_timeout=float(g('source_timeout'))))
        self.kind, self.Control = _load_control_type()
        self.controls = {OUTDOOR: None, INDOOR: None}
        self.gears = {OUTDOOR: None, INDOOR: None}
        self.speed = 0.0
        self.position = None
        self.indoor_healthy = False
        self.automatic_switching = bool(g('automatic_switching'))
        self.enter_polygon = []
        self.exit_polygon = []
        zones_file = str(g('switch_zones_file'))
        if zones_file:
            if not os.path.isfile(zones_file):
                raise FileNotFoundError(f'switch_zones_file not found: {zones_file}')
            with open(zones_file, encoding='utf-8') as f:
                zones = yaml.safe_load(f) or {}
            self.enter_polygon = zones.get('indoor_enter_polygon', [])
            self.exit_polygon = zones.get('indoor_exit_polygon', self.enter_polygon)
        if self.automatic_switching and (
                len(self.enter_polygon) < 3 or len(self.exit_polygon) < 3):
            raise ValueError('automatic switching needs valid enter and exit polygons')

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
        self.create_subscription(String, g('indoor_health_topic'), self.on_indoor_health, 10)
        self.create_service(SetBool, '~/set_indoor', self.on_set_indoor)
        self.create_timer(1.0 / float(g('rate_hz')), self.tick)
        self.get_logger().info(
            f'hybrid control authority starts in {initial}; use ~/set_indoor')

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_control(self, mode, msg):
        self.controls[mode] = msg
        self.arbiter.note_source(mode, self.now_sec())

    def on_gear(self, mode, msg):
        self.gears[mode] = msg

    def on_odom(self, msg):
        self.speed = float(msg.twist.twist.linear.x)
        self.position = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))

    def on_indoor_health(self, msg):
        try:
            data = json.loads(msg.data)
            self.indoor_healthy = data.get('level') not in ('ERROR', 'STALE') and \
                float(data.get('speed_factor', 0.0)) > 0.0
        except (ValueError, TypeError):
            self.indoor_healthy = False

    def on_set_indoor(self, request, response):
        target = INDOOR if request.data else OUTDOOR
        self.arbiter.request(target)
        response.success = True
        response.message = f'{target} requested; stop-first handover started'
        self.get_logger().warn(response.message)
        return response

    def target_healthy(self):
        return self.arbiter.requested_mode != INDOOR or self.indoor_healthy

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
            now, self.speed, target_healthy=self.target_healthy())
        source = self.controls[mode]
        if force_stop or source is None or not self.arbiter.source_fresh(mode, now):
            self.control_pub.publish(self.make_stop())
        else:
            self.control_pub.publish(source)
            if self.gears[mode] is not None:
                self.gear_pub.publish(self.gears[mode])
        self.state_pub.publish(String(data=json.dumps({
            'mode': self.arbiter.mode,
            'requested_mode': self.arbiter.requested_mode,
            'phase': self.arbiter.phase,
            'force_stop': force_stop,
            'speed': round(self.speed, 3),
            'indoor_healthy': self.indoor_healthy,
            'outdoor_fresh': self.arbiter.source_fresh(OUTDOOR, now),
            'indoor_fresh': self.arbiter.source_fresh(INDOOR, now),
            'automatic_switching': self.automatic_switching,
        }, ensure_ascii=False)))


def main():
    rclpy.init()
    node = HybridModeManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

