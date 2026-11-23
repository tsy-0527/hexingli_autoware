#!/usr/bin/env python3
"""Adapt Nav2 Twist commands to the existing safety-gated Autoware bridge."""

import math
import time

from autoware_control_msgs.msg import Control

from autoware_vehicle_msgs.msg import GearCommand, VelocityReport

from geometry_msgs.msg import Twist

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .kinematics import clamp, steering_from_twist


class TwistToControl(Node):
    """Convert Nav2 yaw-rate commands into speed and front tire angle."""

    def __init__(self) -> None:
        """Initialize command conversion, watchdog and gear interlock."""
        super().__init__("yunle_twist_to_control")

        self.wheel_base = float(self.declare_parameter("wheel_base", 0.475).value)
        self.max_steering_angle = float(
            self.declare_parameter("max_steering_angle", 0.366519).value
        )
        self.max_forward_speed = float(
            self.declare_parameter("max_forward_speed", 0.8).value
        )
        self.max_reverse_speed = float(
            self.declare_parameter("max_reverse_speed", 0.2).value
        )
        self.stopped_velocity = float(
            self.declare_parameter("stopped_velocity", 0.01).value
        )
        self.gear_change_speed = float(
            self.declare_parameter("gear_change_speed", 0.05).value
        )
        self.gear_change_pause = float(
            self.declare_parameter("gear_change_pause", 0.5).value
        )
        self.command_timeout = float(
            self.declare_parameter("command_timeout", 0.2).value
        )
        self.stop_publish_duration = float(
            self.declare_parameter("stop_publish_duration", 0.5).value
        )
        publish_rate = float(self.declare_parameter("publish_rate", 20.0).value)

        cmd_vel_topic = str(self.declare_parameter("cmd_vel_topic", "/cmd_vel").value)
        curvature_cmd_vel_topic = str(
            self.declare_parameter(
                "curvature_cmd_vel_topic", "/cmd_vel_nav"
            ).value
        )
        control_topic = str(
            self.declare_parameter(
                "control_topic", "/control/command/control_cmd"
            ).value
        )
        gear_topic = str(
            self.declare_parameter("gear_topic", "/control/command/gear_cmd").value
        )
        velocity_topic = str(
            self.declare_parameter(
                "velocity_topic", "/vehicle/status/velocity_status"
            ).value
        )

        self.control_pub = self.create_publisher(Control, control_topic, 10)
        self.gear_pub = self.create_publisher(GearCommand, gear_topic, 10)
        self.create_subscription(Twist, cmd_vel_topic, self._on_twist, 10)
        self.create_subscription(
            Twist, curvature_cmd_vel_topic, self._on_curvature_twist, 10
        )
        self.create_subscription(VelocityReport, velocity_topic, self._on_velocity, 10)

        self.latest_twist = Twist()
        self.last_command_monotonic = None
        self.latest_curvature_twist = Twist()
        self.last_curvature_command_monotonic = None
        self.timeout_started_monotonic = None
        self.measured_speed = 0.0
        self.active_gear = GearCommand.NEUTRAL
        self.pending_gear = None
        self.gear_change_started = None

        self.timer = self.create_timer(1.0 / max(1.0, publish_rate), self._on_timer)
        self.get_logger().info(
            "Nav2 adapter ready: wheel_base=%.3f m, max_steer=%.2f deg, "
            "speed limits=(%.2f, %.2f) m/s, curvature source=%s"
            % (
                self.wheel_base,
                math.degrees(self.max_steering_angle),
                self.max_forward_speed,
                self.max_reverse_speed,
                curvature_cmd_vel_topic,
            )
        )

    def _on_twist(self, msg: Twist) -> None:
        """Store the velocity-smoother output used for longitudinal speed."""
        self.latest_twist = msg
        self.last_command_monotonic = time.monotonic()
        self.timeout_started_monotonic = None

    def _on_curvature_twist(self, msg: Twist) -> None:
        """Store the controller output whose angular/linear ratio is curvature."""
        self.latest_curvature_twist = msg
        self.last_curvature_command_monotonic = time.monotonic()

    def _on_velocity(self, msg: VelocityReport) -> None:
        self.measured_speed = float(msg.longitudinal_velocity)

    def _publish_gear(self, command: int) -> None:
        msg = GearCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.command = command
        self.gear_pub.publish(msg)

    def _publish_control(self, velocity: float, steering: float) -> None:
        now = self.get_clock().now().to_msg()
        msg = Control()
        msg.stamp = now
        msg.lateral.stamp = now
        msg.lateral.steering_tire_angle = float(steering)
        msg.lateral.steering_tire_rotation_rate = 0.0
        msg.lateral.is_defined_steering_tire_rotation_rate = False
        msg.longitudinal.stamp = now
        msg.longitudinal.velocity = float(velocity)
        msg.longitudinal.acceleration = 0.0
        msg.longitudinal.jerk = 0.0
        msg.longitudinal.is_defined_acceleration = False
        msg.longitudinal.is_defined_jerk = False
        self.control_pub.publish(msg)

    def _publish_stop(self) -> None:
        if self.active_gear == GearCommand.NONE:
            self.active_gear = GearCommand.NEUTRAL
        self._publish_gear(self.active_gear)
        self._publish_control(0.0, 0.0)

    def _handle_gear_change(self, desired_gear: int, now: float) -> bool:
        """Return True while motion must remain inhibited for a gear change."""
        if desired_gear == self.active_gear and self.pending_gear is None:
            return False

        if abs(self.measured_speed) > self.gear_change_speed:
            self._publish_stop()
            return True

        if self.pending_gear != desired_gear:
            self.pending_gear = desired_gear
            self.gear_change_started = now
            self._publish_gear(desired_gear)
            self._publish_control(0.0, 0.0)
            return True

        self._publish_gear(desired_gear)
        self._publish_control(0.0, 0.0)
        if now - self.gear_change_started < self.gear_change_pause:
            return True

        self.active_gear = desired_gear
        self.pending_gear = None
        self.gear_change_started = None
        return False

    def _on_timer(self) -> None:
        now = time.monotonic()
        if self.last_command_monotonic is None:
            return

        command_age = now - self.last_command_monotonic
        if command_age > self.command_timeout:
            if self.timeout_started_monotonic is None:
                self.timeout_started_monotonic = now
                self.get_logger().warn("Nav2 velocity command timed out; stopping")
            if now - self.timeout_started_monotonic <= self.stop_publish_duration:
                self._publish_stop()
            return

        velocity = clamp(
            float(self.latest_twist.linear.x),
            -self.max_reverse_speed,
            self.max_forward_speed,
        )

        if abs(velocity) <= self.stopped_velocity:
            # Reject Nav2's in-place rotation commands on the Ackermann chassis.
            self._publish_stop()
            return

        desired_gear = GearCommand.DRIVE if velocity > 0.0 else GearCommand.REVERSE
        if self._handle_gear_change(desired_gear, now):
            return

        # The generic Nav2 velocity smoother limits linear.x and angular.z
        # independently while accelerating.  Feeding that transient ratio to
        # Ackermann kinematics produced a false curvature spike and saturated
        # the Yunle steering at 21 degrees.  Use its smoothed linear speed for
        # longitudinal control, but preserve the RPP controller's original
        # angular/linear ratio for steering.
        curvature_command_fresh = (
            self.last_curvature_command_monotonic is not None
            and now - self.last_curvature_command_monotonic <= self.command_timeout
        )
        if curvature_command_fresh:
            steering = steering_from_twist(
                float(self.latest_curvature_twist.linear.x),
                float(self.latest_curvature_twist.angular.z),
                self.wheel_base,
                self.max_steering_angle,
                self.stopped_velocity,
            )
        else:
            steering = 0.0
        self._publish_gear(self.active_gear)
        self._publish_control(velocity, steering)


def main(args=None) -> None:
    """Run the Twist-to-Control adapter node."""
    rclpy.init(args=args)
    node = TwistToControl()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
