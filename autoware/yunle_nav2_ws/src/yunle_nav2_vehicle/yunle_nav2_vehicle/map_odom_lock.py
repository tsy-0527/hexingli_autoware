#!/usr/bin/env python3
"""Lock map->odom from an RViz initial pose for short-range relative navigation."""

import math
import time

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from .kinematics import (
    locked_map_to_odom,
    map_correction_is_acceptable,
    soft_correct_map_to_odom,
)


def _yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


class MapOdomLock(Node):
    """Capture map->odom once per /initialpose update and keep it constant."""

    def __init__(self) -> None:
        super().__init__("map_odom_lock")
        self.map_frame = str(self.declare_parameter("map_frame", "map").value)
        self.odom_frame = str(self.declare_parameter("odom_frame", "odom").value)
        self.base_frame = str(self.declare_parameter("base_frame", "base_link").value)
        publish_rate = float(self.declare_parameter("publish_rate", 20.0).value)
        self.soft_correction_enabled = bool(
            self.declare_parameter("soft_correction_enabled", False).value
        )
        self.correction_gain = float(
            self.declare_parameter("correction_gain", 0.10).value
        )
        self.max_translation_step = float(
            self.declare_parameter("max_translation_step", 0.01).value
        )
        self.max_yaw_step = float(
            self.declare_parameter("max_yaw_step", math.radians(0.3)).value
        )
        self.max_translation_innovation = float(
            self.declare_parameter("max_translation_innovation", 0.50).value
        )
        self.max_yaw_innovation = float(
            self.declare_parameter("max_yaw_innovation", math.radians(10.0)).value
        )
        self.max_xy_covariance = float(
            self.declare_parameter("max_xy_covariance", 0.10).value
        )
        self.max_yaw_covariance = float(
            self.declare_parameter("max_yaw_covariance", 0.10).value
        )
        self.correction_grace_period = float(
            self.declare_parameter("correction_grace_period", 2.0).value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pending_pose = None
        self.locked_transform = None
        self.last_lock_monotonic = None
        self.rejected_corrections = 0

        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self._on_initial_pose, 10
        )
        if self.soft_correction_enabled:
            self.create_subscription(
                PoseWithCovarianceStamped,
                "/amcl_pose",
                self._on_amcl_pose,
                10,
            )
        self.create_timer(1.0 / max(1.0, publish_rate), self._on_timer)
        self.get_logger().info(
            "Map lock ready: set 2D Pose Estimate to initialize %s -> %s; "
            "soft correction=%s"
            % (self.map_frame, self.odom_frame, self.soft_correction_enabled)
        )

    def _on_initial_pose(self, msg: PoseWithCovarianceStamped) -> None:
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.get_logger().error(
                "Ignoring initial pose in frame '%s'; expected '%s'"
                % (msg.header.frame_id, self.map_frame)
            )
            return
        self.pending_pose = msg.pose.pose
        self._try_lock()

    def _try_lock(self) -> None:
        if self.pending_pose is None:
            return
        try:
            odom_base = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, Time()
            )
        except TransformException:
            return

        requested = self.pending_pose
        current = odom_base.transform
        map_odom_x, map_odom_y, map_odom_yaw = locked_map_to_odom(
            requested.position.x,
            requested.position.y,
            _yaw_from_quaternion(requested.orientation),
            current.translation.x,
            current.translation.y,
            _yaw_from_quaternion(current.rotation),
        )

        transform = TransformStamped()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = map_odom_x
        transform.transform.translation.y = map_odom_y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = math.sin(map_odom_yaw / 2.0)
        transform.transform.rotation.w = math.cos(map_odom_yaw / 2.0)
        self.locked_transform = transform
        self.pending_pose = None
        self.last_lock_monotonic = time.monotonic()
        self.get_logger().info(
            "Locked map->odom at x=%.3f m, y=%.3f m, yaw=%.2f deg"
            % (map_odom_x, map_odom_y, math.degrees(map_odom_yaw))
        )

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        if not self.soft_correction_enabled or self.locked_transform is None:
            return
        if (
            self.last_lock_monotonic is not None
            and time.monotonic() - self.last_lock_monotonic
            < self.correction_grace_period
        ):
            return
        try:
            odom_base = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, Time.from_msg(msg.header.stamp)
            )
        except TransformException:
            return

        pose = msg.pose.pose
        current_odom_base = odom_base.transform
        desired_x, desired_y, desired_yaw = locked_map_to_odom(
            pose.position.x,
            pose.position.y,
            _yaw_from_quaternion(pose.orientation),
            current_odom_base.translation.x,
            current_odom_base.translation.y,
            _yaw_from_quaternion(current_odom_base.rotation),
        )
        transform = self.locked_transform.transform
        current_x = transform.translation.x
        current_y = transform.translation.y
        current_yaw = _yaw_from_quaternion(transform.rotation)
        covariance = msg.pose.covariance
        if not map_correction_is_acceptable(
            current_x,
            current_y,
            current_yaw,
            desired_x,
            desired_y,
            desired_yaw,
            covariance[0],
            covariance[7],
            covariance[35],
            self.max_translation_innovation,
            self.max_yaw_innovation,
            self.max_xy_covariance,
            self.max_yaw_covariance,
        ):
            self.rejected_corrections += 1
            if self.rejected_corrections % 20 == 1:
                self.get_logger().warn(
                    "Rejected AMCL map correction: innovation or covariance too large"
                )
            return

        corrected_x, corrected_y, corrected_yaw = soft_correct_map_to_odom(
            current_x,
            current_y,
            current_yaw,
            desired_x,
            desired_y,
            desired_yaw,
            self.correction_gain,
            self.max_translation_step,
            self.max_yaw_step,
        )
        transform.translation.x = corrected_x
        transform.translation.y = corrected_y
        transform.rotation.x = 0.0
        transform.rotation.y = 0.0
        transform.rotation.z = math.sin(corrected_yaw / 2.0)
        transform.rotation.w = math.cos(corrected_yaw / 2.0)

    def _on_timer(self) -> None:
        self._try_lock()
        if self.locked_transform is None:
            return
        self.locked_transform.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.locked_transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapOdomLock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
