"""cmd_vel (after Collision Monitor) -> Ackermann command for the chassis.

Subscribes : cmd_vel (geometry_msgs/Twist), odometry (speed feedback),
             localization health JSON (speed factor)
Publishes  : vehicle command on `output_topic`            <- TODO(chassis interface)
             ~/debug (std_msgs/String JSON)

=====================================================================
TODO(chassis interface): the message type / topic consumed by your
chassis_driver (and the command mux in front of it) is not decided yet.
Set `output_type` + `output_topic` in config/indoor_navigation_nodes.param.yaml:
  - "autoware_control"   : autoware_control_msgs/Control
                           (falls back to autoware_auto_control_msgs/AckermannControlCommand)
  - "ackermann_drive"    : ackermann_msgs/AckermannDriveStamped
  - "debug_only"         : publish nothing but ~/debug
The CAN mapping (0x121) stays inside chassis_driver.
=====================================================================
"""
import json
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from autoware_vehicle_msgs.msg import GearCommand

from .ackermann import AckermannParams, AckermannShaper


def _load_output(kind):
    if kind == 'autoware_control':
        try:
            from autoware_control_msgs.msg import Control
            return 'control', Control
        except ImportError:
            from autoware_auto_control_msgs.msg import AckermannControlCommand
            return 'ackermann_control_command', AckermannControlCommand
    if kind == 'ackermann_drive':
        from ackermann_msgs.msg import AckermannDriveStamped
        return 'ackermann_drive', AckermannDriveStamped
    return 'debug_only', None


class AckermannCmdConverter(Node):
    def __init__(self):
        super().__init__('ackermann_cmd_converter')
        d = self.declare_parameter
        d('input_topic', '/cmd_vel')
        d('odom_topic', '/localization/kinematic_state')
        d('health_topic', '/indoor_localization/health')
        d('use_health_speed_factor', True)
        d('output_type', 'autoware_control')                       # TODO(chassis interface)
        d('output_topic', '/control/command/control_cmd')
        d('gear_topic', '/control/command/gear_cmd')
        d('rate_hz', 30.0)
        defaults = AckermannParams()
        for k, v in vars(defaults).items():
            d(k, float(v))
        g = lambda k: self.get_parameter(k).value  # noqa
        self.p = AckermannParams(**{k: float(g(k)) for k in vars(defaults)})
        self.shaper = AckermannShaper(self.p)
        self.cmd = (0.0, 0.0)
        self.v_meas = None
        self.speed_factor = 1.0
        self.health_level = None
        self.kind, self.MsgT = _load_output(g('output_type'))
        self.pub = self.create_publisher(self.MsgT, g('output_topic'), 10) if self.MsgT else None
        self.gear_pub = self.create_publisher(GearCommand, g('gear_topic'), 10)
        self.last_gear = GearCommand.NEUTRAL
        self.dbg = self.create_publisher(String, '~/debug', 10)
        self.create_subscription(Twist, g('input_topic'), self.on_cmd, 10)
        self.create_subscription(Odometry, g('odom_topic'), self.on_odom, 10)
        if g('use_health_speed_factor'):
            self.create_subscription(String, g('health_topic'), self.on_health, 10)
        self.dt = 1.0 / float(g('rate_hz'))
        self.create_timer(self.dt, self.tick)
        self.get_logger().info(f'output: {self.kind} on {g("output_topic")} '
                               f'(wheelbase {self.p.wheelbase}, max steer {math.degrees(self.p.max_steer):.1f} deg)')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, m):
        self.cmd = (m.linear.x, m.angular.z)
        self.shaper.on_cmd(self.now())

    def on_odom(self, m):
        self.v_meas = m.twist.twist.linear.x

    def on_health(self, m):
        try:
            h = json.loads(m.data)
            self.speed_factor = float(h.get('speed_factor', 1.0))
            self.health_level = h.get('level')
        except (ValueError, TypeError):
            pass

    def tick(self):
        t = self.now()
        o = self.shaper.step(t, self.dt, self.cmd[0], self.cmd[1], self.speed_factor, self.v_meas)
        if self.pub is not None:
            self.publish_gear(o['speed'])
            self.pub.publish(self.make_msg(o))
        self.dbg.publish(String(data=json.dumps(dict(
            speed=round(o['speed'], 3), steer_deg=round(math.degrees(o['steer']), 2),
            steer_target_deg=round(math.degrees(o['steer_target']), 2), accel=round(o['accel'], 3),
            cmd_v=round(self.cmd[0], 3), cmd_wz=round(self.cmd[1], 3), timed_out=o['timed_out'],
            speed_factor=self.speed_factor, health=self.health_level))))

    def publish_gear(self, shaped_speed):
        """Feed the existing Yunle Autoware bridge with D/R/N selection.

        During a direction change the shaper first holds zero speed. Change
        gear only after feedback says the vehicle is stopped, then the next
        non-zero control command is accepted with the correct direction.
        """
        stopped = self.v_meas is None or abs(self.v_meas) <= self.p.v_eps
        if shaped_speed > self.p.v_eps:
            self.last_gear = GearCommand.DRIVE
        elif shaped_speed < -self.p.v_eps:
            self.last_gear = GearCommand.REVERSE
        elif stopped and self.cmd[0] > self.p.v_eps:
            self.last_gear = GearCommand.DRIVE
        elif stopped and self.cmd[0] < -self.p.v_eps:
            self.last_gear = GearCommand.REVERSE
        msg = GearCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.command = self.last_gear
        self.gear_pub.publish(msg)

    def make_msg(self, o):
        stamp = self.get_clock().now().to_msg()
        msg = self.MsgT()
        if self.kind == 'control':
            msg.stamp = stamp
            msg.control_time = stamp
            msg.lateral.stamp = stamp
            msg.lateral.control_time = stamp
            msg.lateral.steering_tire_angle = float(o['steer'])
            msg.lateral.steering_tire_rotation_rate = 0.0
            msg.longitudinal.stamp = stamp
            msg.longitudinal.control_time = stamp
            msg.longitudinal.velocity = float(o['speed'])
            msg.longitudinal.acceleration = float(o['accel'])
            if hasattr(msg.longitudinal, 'is_defined_acceleration'):
                msg.longitudinal.is_defined_acceleration = True
        elif self.kind == 'ackermann_control_command':
            msg.stamp = stamp
            msg.lateral.stamp = stamp
            msg.lateral.steering_tire_angle = float(o['steer'])
            msg.longitudinal.stamp = stamp
            msg.longitudinal.speed = float(o['speed'])
            msg.longitudinal.acceleration = float(o['accel'])
        elif self.kind == 'ackermann_drive':
            msg.header.stamp = stamp
            msg.header.frame_id = 'base_link'
            msg.drive.steering_angle = float(o['steer'])
            msg.drive.speed = float(o['speed'])
            msg.drive.acceleration = float(o['accel'])
        return msg


def main():
    rclpy.init()
    node = AckermannCmdConverter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
