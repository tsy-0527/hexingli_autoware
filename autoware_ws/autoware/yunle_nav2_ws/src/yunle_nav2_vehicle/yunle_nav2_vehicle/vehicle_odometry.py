#!/usr/bin/env python3
"""Generate raw Ackermann odometry from normalized Yunle vehicle feedback."""

import math

from autoware_vehicle_msgs.msg import SteeringReport, VelocityReport

from geometry_msgs.msg import TransformStamped

from nav_msgs.msg import Odometry

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from .kinematics import normalize_angle, yaw_rate_from_steering


class VehicleOdometry(Node):
    """Integrate actual speed and tire angle using a bicycle model."""

    def __init__(self) -> None:
        """Initialize feedback subscriptions and raw odometry output."""
        super().__init__("yunle_vehicle_odometry")
        self.wheel_base = float(self.declare_parameter("wheel_base", 0.475).value)
        self.odom_frame = str(self.declare_parameter("odom_frame", "odom").value)
        self.base_frame = str(self.declare_parameter("base_frame", "base_link").value)
        self.publish_tf = bool(self.declare_parameter("publish_tf", False).value)
        velocity_topic = str(
            self.declare_parameter(
                "velocity_topic", "/vehicle/status/velocity_status"
            ).value
        )
        steering_topic = str(
            self.declare_parameter(
                "steering_topic", "/vehicle/status/steering_status"
            ).value
        )
        odom_topic = str(
            self.declare_parameter("odom_topic", "/odometry/wheel").value
        )

        self.odom_pub = self.create_publisher(Odometry, odom_topic, 20)
        self.create_subscription(SteeringReport, steering_topic, self._on_steering, 20)
        self.create_subscription(VelocityReport, velocity_topic, self._on_velocity, 20)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.steering = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_stamp = None

    def _on_steering(self, msg: SteeringReport) -> None:
        self.steering = float(msg.steering_tire_angle)

    @staticmethod
    def _quaternion_from_yaw(yaw: float):
        half = yaw * 0.5
        return 0.0, 0.0, math.sin(half), math.cos(half)

    def _on_velocity(self, msg: VelocityReport) -> None:
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0:
            stamp = self.get_clock().now()

        velocity = float(msg.longitudinal_velocity)
        yaw_rate = yaw_rate_from_steering(velocity, self.steering, self.wheel_base)

        if self.last_stamp is not None:
            dt = (stamp - self.last_stamp).nanoseconds * 1.0e-9
            if 0.0 < dt <= 0.5:
                mid_yaw = self.yaw + 0.5 * yaw_rate * dt
                self.x += velocity * math.cos(mid_yaw) * dt
                self.y += velocity * math.sin(mid_yaw) * dt
                self.yaw = normalize_angle(self.yaw + yaw_rate * dt)
        self.last_stamp = stamp

        qx, qy, qz, qw = self._quaternion_from_yaw(self.yaw)
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = velocity
        # An Ackermann chassis has no lateral wheel velocity in normal
        # operation.  Publishing vy=0 with finite covariance lets the EKF
        # enforce the nonholonomic constraint instead of leaving y unbounded.
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = yaw_rate
        odom.pose.covariance[0] = 0.10
        odom.pose.covariance[7] = 0.10
        odom.pose.covariance[35] = 0.15
        odom.twist.covariance[0] = 0.04
        odom.twist.covariance[7] = 0.01
        odom.twist.covariance[35] = 0.05
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    """Run the raw vehicle odometry node."""
    rclpy.init(args=args)
    node = VehicleOdometry()
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
