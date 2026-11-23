#!/usr/bin/env python3
"""External cost-aware path proxy for Autoware.

This node is deliberately independent from behavior_path_planner.  It consumes the
already generated PathWithLaneId and PredictedObjects messages, creates a small set
of local lateral-avoidance candidates, scores them, and publishes the selected path
on a separate topic.

The default mode is ``shadow``: the selected path is published only on the proxy
output topic and cannot affect Autoware unless a downstream launch remaps its input
to that topic.  ``override`` is intentionally opt-in.

This is a validation prototype, not a replacement for Lanelet2 traffic-rule logic.
It generates same-lane/local lateral detours and a stop fallback.  Lane changes,
opposite-lane borrowing, turn-signal decisions, and RTC approval remain owned by
Autoware until the idea has been validated.
"""

from __future__ import annotations

import copy
import json
import math
import time
from dataclasses import dataclass
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from autoware_internal_planning_msgs.msg import PathWithLaneId
except ImportError:  # pragma: no cover - compatibility with older Autoware layouts
    from autoware_planning_msgs.msg import PathWithLaneId

try:
    from autoware_perception_msgs.msg import PredictedObjects
except ImportError:  # pragma: no cover - compatibility with older Autoware layouts
    from autoware_auto_perception_msgs.msg import PredictedObjects


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def yaw_from_quaternion(quaternion: Any) -> float:
    sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


def set_pose_yaw(pose: Any, yaw: float) -> None:
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = math.sin(0.5 * yaw)
    pose.orientation.w = math.cos(0.5 * yaw)


def rotate_xy(x: float, y: float, quaternion: Any) -> tuple[float, float]:
    """Rotate a 2D vector by a quaternion."""
    qx = float(quaternion.x)
    qy = float(quaternion.y)
    qz = float(quaternion.z)
    qw = float(quaternion.w)

    # q * [x, y, 0] * q^-1, expanded for the 2D input.
    tx = 2.0 * (qy * 0.0 - qz * y)
    ty = 2.0 * (qz * x - qx * 0.0)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + qy * tz - qz * ty,
        y + qw * ty + qz * tx - qx * tz,
    )


@dataclass
class PathGeometry:
    points: list[tuple[float, float]]
    yaws: list[float]
    arc_lengths: list[float]
    left_widths: list[Optional[float]]
    right_widths: list[Optional[float]]


@dataclass
class Obstacle:
    object_id: str
    s: float
    l: float
    half_s: float
    half_l: float
    speed: float
    existence: float


@dataclass
class Candidate:
    name: str
    path: PathWithLaneId
    profile: list[float]
    valid: bool
    total_cost: float
    components: dict[str, float]
    reason: str = ""


def build_path_geometry(path: PathWithLaneId) -> Optional[PathGeometry]:
    if len(path.points) < 2:
        return None

    points = [
        (float(item.point.pose.position.x), float(item.point.pose.position.y))
        for item in path.points
    ]

    yaws: list[float] = []
    for index in range(len(points)):
        if index == 0:
            dx = points[1][0] - points[0][0]
            dy = points[1][1] - points[0][1]
        elif index == len(points) - 1:
            dx = points[-1][0] - points[-2][0]
            dy = points[-1][1] - points[-2][1]
        else:
            dx = points[index + 1][0] - points[index - 1][0]
            dy = points[index + 1][1] - points[index - 1][1]

        if math.hypot(dx, dy) < 1.0e-6:
            yaw = yaw_from_quaternion(path.points[index].point.pose.orientation)
        else:
            yaw = math.atan2(dy, dx)
        yaws.append(yaw)

    arc_lengths = [0.0]
    for index in range(1, len(points)):
        arc_lengths.append(
            arc_lengths[-1]
            + math.hypot(
                points[index][0] - points[index - 1][0],
                points[index][1] - points[index - 1][1],
            )
        )

    left_widths: list[Optional[float]] = [None] * len(points)
    right_widths: list[Optional[float]] = [None] * len(points)
    if len(path.left_bound) == len(points) and len(path.right_bound) == len(points):
        for index, (x, y) in enumerate(points):
            normal_x = -math.sin(yaws[index])
            normal_y = math.cos(yaws[index])
            left_point = path.left_bound[index]
            right_point = path.right_bound[index]
            left_widths[index] = max(
                0.0,
                (float(left_point.x) - x) * normal_x
                + (float(left_point.y) - y) * normal_y,
            )
            right_widths[index] = max(
                0.0,
                (x - float(right_point.x)) * normal_x
                + (y - float(right_point.y)) * normal_y,
            )

    return PathGeometry(points, yaws, arc_lengths, left_widths, right_widths)


def project_xy(x: float, y: float, geometry: PathGeometry) -> tuple[float, float, int]:
    """Return Frenet-like s/l and the nearest path point index."""
    best_distance = float("inf")
    best_s = 0.0
    best_l = 0.0
    best_index = 0

    for index in range(len(geometry.points) - 1):
        start_x, start_y = geometry.points[index]
        end_x, end_y = geometry.points[index + 1]
        segment_x = end_x - start_x
        segment_y = end_y - start_y
        segment_length_squared = segment_x * segment_x + segment_y * segment_y
        if segment_length_squared < 1.0e-9:
            ratio = 0.0
        else:
            ratio = clamp(
                ((x - start_x) * segment_x + (y - start_y) * segment_y)
                / segment_length_squared,
                0.0,
                1.0,
            )

        projected_x = start_x + ratio * segment_x
        projected_y = start_y + ratio * segment_y
        dx = x - projected_x
        dy = y - projected_y
        segment_length = math.sqrt(segment_length_squared)
        if segment_length < 1.0e-6:
            tangent_x = math.cos(geometry.yaws[index])
            tangent_y = math.sin(geometry.yaws[index])
        else:
            tangent_x = segment_x / segment_length
            tangent_y = segment_y / segment_length
        normal_x = -tangent_y
        normal_y = tangent_x
        distance = dx * dx + dy * dy
        if distance < best_distance:
            best_distance = distance
            best_s = geometry.arc_lengths[index] + ratio * segment_length
            best_l = (x - projected_x) * normal_x + (y - projected_y) * normal_y
            best_index = index if ratio < 0.5 else index + 1

    return best_s, best_l, best_index


def offset_at_s(arc_lengths: list[float], profile: list[float], s: float) -> float:
    if not profile:
        return 0.0
    if s <= arc_lengths[0]:
        return profile[0]
    if s >= arc_lengths[-1]:
        return profile[-1]

    for index in range(1, len(arc_lengths)):
        if arc_lengths[index] >= s:
            denominator = arc_lengths[index] - arc_lengths[index - 1]
            if denominator < 1.0e-6:
                return profile[index]
            ratio = (s - arc_lengths[index - 1]) / denominator
            return profile[index - 1] + ratio * (profile[index] - profile[index - 1])
    return profile[-1]


def smoothstep(ratio: float) -> float:
    ratio = clamp(ratio, 0.0, 1.0)
    return ratio * ratio * (3.0 - 2.0 * ratio)


class CostAwarePathProxy(Node):
    def __init__(self) -> None:
        super().__init__("cost_aware_path_proxy")

        self.declare_parameter("mode", "shadow")
        self.declare_parameter(
            "input_path_topic",
            "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id",
        )
        self.declare_parameter(
            "output_path_topic",
            "/planning/scenario_planning/lane_driving/behavior_planning/cost_aware/path_with_lane_id",
        )
        self.declare_parameter(
            "objects_topic", "/perception/object_recognition/objects"
        )
        self.declare_parameter(
            "debug_topic",
            "/planning/scenario_planning/lane_driving/behavior_planning/cost_aware/debug",
        )
        self.declare_parameter(
            "candidate_topic_prefix",
            "/planning/scenario_planning/lane_driving/behavior_planning/cost_aware/candidate",
        )
        self.declare_parameter(
            "rviz_path_topic",
            "/planning/scenario_planning/lane_driving/behavior_planning/cost_aware/rviz_path",
        )

        self.declare_parameter("planning_horizon_m", 60.0)
        self.declare_parameter("max_lateral_distance_m", 6.0)
        self.declare_parameter("vehicle_width_m", 1.0)
        self.declare_parameter("vehicle_length_m", 4.0)
        self.declare_parameter("hard_clearance_m", 0.35)
        self.declare_parameter("candidate_clearance_buffer_m", 0.05)
        self.declare_parameter("object_margin_m", 0.20)
        self.declare_parameter("min_shift_m", 0.80)
        self.declare_parameter("max_shift_m", 2.50)
        self.declare_parameter("shift_prepare_distance_m", 6.0)
        self.declare_parameter("shift_length_m", 8.0)
        self.declare_parameter("return_length_m", 8.0)
        self.declare_parameter("return_margin_m", 3.0)
        self.declare_parameter("risk_clearance_m", 1.0)
        self.declare_parameter("dynamic_speed_threshold_mps", 0.5)
        self.declare_parameter("min_object_existence_probability", 0.25)
        self.declare_parameter("objects_timeout_sec", 0.75)
        self.declare_parameter("stop_margin_m", 3.0)
        self.declare_parameter("stop_cost", 1000.0)
        self.declare_parameter("switch_cost", 3.0)
        self.declare_parameter("publish_candidates", True)
        self.declare_parameter("crop_path_to_ego", True)
        self.declare_parameter("ego_frame", "base_link")
        self.declare_parameter("max_ego_to_path_distance_m", 5.0)
        self.declare_parameter("ego_tf_timeout_sec", 0.05)

        self.declare_parameter("weights.lateral", 2.0)
        self.declare_parameter("weights.smoothness", 8.0)
        self.declare_parameter("weights.clearance", 8.0)
        self.declare_parameter("weights.risk", 20.0)
        self.declare_parameter("weights.boundary", 8.0)
        self.declare_parameter("weights.speed_loss", 1.0)

        mode = str(self.get_parameter("mode").value).lower()
        if mode not in {"shadow", "override"}:
            raise ValueError("mode must be 'shadow' or 'override'")
        self.mode = mode

        self.latest_objects: Optional[PredictedObjects] = None
        self.latest_objects_monotonic = 0.0
        self.last_selected = "KEEP_LANE"
        self.last_tf_warning_monotonic = 0.0
        self.last_summary_monotonic = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = 10
        input_path_topic = str(self.get_parameter("input_path_topic").value)
        objects_topic = str(self.get_parameter("objects_topic").value)
        output_path_topic = str(self.get_parameter("output_path_topic").value)
        debug_topic = str(self.get_parameter("debug_topic").value)
        rviz_path_topic = str(self.get_parameter("rviz_path_topic").value)
        if output_path_topic == input_path_topic:
            raise ValueError(
                "output_path_topic must be isolated from input_path_topic; "
                "two publishers on the original Autoware path are unsafe"
            )

        self.path_sub = self.create_subscription(
            PathWithLaneId, input_path_topic, self._on_path, qos
        )
        self.objects_sub = self.create_subscription(
            PredictedObjects, objects_topic, self._on_objects, qos
        )
        self.path_pub = self.create_publisher(PathWithLaneId, output_path_topic, qos)
        self.rviz_path_pub = self.create_publisher(NavPath, rviz_path_topic, qos)
        self.debug_pub = self.create_publisher(String, debug_topic, qos)

        candidate_prefix = str(self.get_parameter("candidate_topic_prefix").value)
        self.candidate_pubs = {
            name: self.create_publisher(
                PathWithLaneId, f"{candidate_prefix}/{name.lower()}", qos
            )
            for name in ("KEEP_LANE", "LEFT_SHIFT", "RIGHT_SHIFT", "STOP_WAIT")
        }

        self.get_logger().info(
            f"cost-aware path proxy started in {self.mode} mode; "
            f"input={input_path_topic}, output={output_path_topic}, "
            f"rviz_output={rviz_path_topic}, objects={objects_topic}"
        )

    def _on_objects(self, message: PredictedObjects) -> None:
        self.latest_objects = message
        self.latest_objects_monotonic = time.monotonic()

    def _crop_path_to_ego(
        self, path: PathWithLaneId
    ) -> tuple[PathWithLaneId, dict[str, Any]]:
        """Make the ego pose the first point of the local planning path."""
        if not bool(self.get_parameter("crop_path_to_ego").value):
            return path, {"mode": "original", "reason": "disabled"}

        target_frame = str(path.header.frame_id)
        ego_frame = str(self.get_parameter("ego_frame").value)
        if not target_frame or not ego_frame:
            return path, {"mode": "original", "reason": "missing_frame_id"}

        try:
            ego_transform = self.tf_buffer.lookup_transform(
                target_frame,
                ego_frame,
                Time(),
                timeout=Duration(
                    seconds=float(self.get_parameter("ego_tf_timeout_sec").value)
                ),
            )
        except TransformException as exception:
            now = time.monotonic()
            if now - self.last_tf_warning_monotonic > 2.0:
                self.get_logger().warning(
                    f"cannot transform ego {ego_frame}->{target_frame}: {exception}"
                )
                self.last_tf_warning_monotonic = now
            return path, {"mode": "original", "reason": "ego_tf_unavailable"}

        geometry = build_path_geometry(path)
        if geometry is None:
            return path, {"mode": "original", "reason": "path_has_less_than_two_points"}

        ego_x = float(ego_transform.transform.translation.x)
        ego_y = float(ego_transform.transform.translation.y)
        _, ego_l, nearest_index = project_xy(ego_x, ego_y, geometry)
        nearest_x, nearest_y = geometry.points[nearest_index]
        ego_to_path_distance = math.hypot(ego_x - nearest_x, ego_y - nearest_y)
        max_distance = float(
            self.get_parameter("max_ego_to_path_distance_m").value
        )
        if ego_to_path_distance > max_distance:
            return path, {
                "mode": "original",
                "reason": "ego_too_far_from_path",
                "distance_m": round(ego_to_path_distance, 3),
                "max_distance_m": max_distance,
            }

        start_index = min(nearest_index, len(path.points) - 2)
        result = copy.deepcopy(path)
        result.points = list(path.points[start_index:])

        # Preserve PathWithLaneId's usual boundary layouts while cropping:
        # either one boundary per path point or one per path segment.
        if len(path.left_bound) == len(path.points):
            result.left_bound = list(path.left_bound[start_index:])
        elif len(path.left_bound) == len(path.points) - 1:
            result.left_bound = list(path.left_bound[start_index:])
        else:
            result.left_bound = []
        if len(path.right_bound) == len(path.points):
            result.right_bound = list(path.right_bound[start_index:])
        elif len(path.right_bound) == len(path.points) - 1:
            result.right_bound = list(path.right_bound[start_index:])
        else:
            result.right_bound = []

        # Anchor the first path point to the current vehicle pose.  The remaining
        # points keep Autoware's reference-path geometry and lane IDs.
        first_point = result.points[0].point
        first_point.pose.position.x = ego_x
        first_point.pose.position.y = ego_y
        first_point.pose.position.z = float(ego_transform.transform.translation.z)
        first_point.pose.orientation = copy.deepcopy(ego_transform.transform.rotation)

        return result, {
            "mode": "ego",
            "ego_frame": ego_frame,
            "original_nearest_index": nearest_index,
            "cropped_points": len(result.points),
            "ego_to_path_distance_m": round(ego_to_path_distance, 3),
            "ego_lateral_error_m": round(ego_l, 3),
        }

    def _on_path(self, path: PathWithLaneId) -> None:
        planning_path, path_start = self._crop_path_to_ego(path)
        geometry = build_path_geometry(planning_path)
        if geometry is None:
            self.path_pub.publish(planning_path)
            self.rviz_path_pub.publish(self._to_rviz_path(planning_path))
            self._publish_debug(
                {
                    "selected": "KEEP_LANE",
                    "reason": "path_has_less_than_two_points",
                    "path_start": path_start,
                }
            )
            return

        objects_are_stale = (
            self.latest_objects is None
            or time.monotonic() - self.latest_objects_monotonic
            > float(self.get_parameter("objects_timeout_sec").value)
        )
        obstacles = (
            []
            if objects_are_stale
            else self._extract_obstacles(planning_path, geometry)
        )
        candidates = self._make_candidates(planning_path, geometry, obstacles)
        selected = min(
            (candidate for candidate in candidates if candidate.valid),
            key=lambda candidate: candidate.total_cost,
            default=None,
        )

        if selected is None:
            # STOP_WAIT is always expected to be valid.  Keep a conservative fallback
            # in case a future change invalidates every candidate.
            selected = self._make_stop_candidate(planning_path, geometry, obstacles)

        self.last_selected = selected.name
        self.path_pub.publish(selected.path)
        self.rviz_path_pub.publish(self._to_rviz_path(selected.path))

        if bool(self.get_parameter("publish_candidates").value):
            for candidate in candidates:
                publisher = self.candidate_pubs.get(candidate.name)
                if publisher is not None:
                    publisher.publish(candidate.path)

        self._publish_debug(
            {
                "mode": self.mode,
                "frame_id": planning_path.header.frame_id,
                "path_start": path_start,
                "path_stamp": {
                    "sec": int(planning_path.header.stamp.sec),
                    "nanosec": int(planning_path.header.stamp.nanosec),
                },
                "objects_stale": objects_are_stale,
                "obstacle_count": len(obstacles),
                "selected": selected.name,
                "candidates": [self._candidate_debug(candidate) for candidate in candidates],
            }
        )

        now = time.monotonic()
        if now - self.last_summary_monotonic > 1.0:
            self.get_logger().info(
                f"selected={selected.name}, obstacles={len(obstacles)}, "
                f"cost={selected.total_cost:.3f}, mode={self.mode}"
            )
            self.last_summary_monotonic = now

    def _extract_obstacles(
        self, path: PathWithLaneId, geometry: PathGeometry
    ) -> list[Obstacle]:
        if self.latest_objects is None:
            return []

        source_frame = str(self.latest_objects.header.frame_id)
        target_frame = str(path.header.frame_id)
        transform = None
        if source_frame and target_frame and source_frame != target_frame:
            stamp = (
                Time.from_msg(self.latest_objects.header.stamp)
                if self.latest_objects.header.stamp.sec
                or self.latest_objects.header.stamp.nanosec
                else Time()
            )
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    stamp,
                    timeout=Duration(seconds=0.05),
                )
            except TransformException as exception:
                now = time.monotonic()
                if now - self.last_tf_warning_monotonic > 2.0:
                    self.get_logger().warning(
                        f"cannot transform objects {source_frame}->{target_frame}: {exception}"
                    )
                    self.last_tf_warning_monotonic = now
                return []

        min_probability = float(
            self.get_parameter("min_object_existence_probability").value
        )
        horizon = float(self.get_parameter("planning_horizon_m").value)
        max_lateral = float(self.get_parameter("max_lateral_distance_m").value)
        object_margin = float(self.get_parameter("object_margin_m").value)
        result: list[Obstacle] = []

        for object_message in self.latest_objects.objects:
            existence = float(getattr(object_message, "existence_probability", 1.0))
            if existence < min_probability:
                continue

            pose = getattr(
                getattr(object_message, "kinematics", None),
                "initial_pose_with_covariance",
                None,
            )
            if pose is None:
                continue

            x = float(pose.pose.position.x)
            y = float(pose.pose.position.y)
            if transform is not None:
                rotated_x, rotated_y = rotate_xy(
                    x,
                    y,
                    transform.transform.rotation,
                )
                x = rotated_x + float(transform.transform.translation.x)
                y = rotated_y + float(transform.transform.translation.y)

            object_s, object_l, _ = project_xy(x, y, geometry)
            if object_s < -5.0 or object_s > horizon:
                continue
            if abs(object_l) > max_lateral:
                continue

            half_length, half_width = self._object_half_dimensions(object_message)
            speed = self._object_speed(object_message)
            object_id = bytes(object_message.object_id.uuid).hex()
            result.append(
                Obstacle(
                    object_id=object_id or "unknown",
                    s=object_s,
                    l=object_l,
                    half_s=half_length + object_margin,
                    half_l=half_width + object_margin,
                    speed=speed,
                    existence=existence,
                )
            )

        result.sort(key=lambda obstacle: obstacle.s)
        return result

    @staticmethod
    def _to_rviz_path(path: PathWithLaneId) -> NavPath:
        """Convert PathWithLaneId for the standard RViz Path display."""
        result = NavPath()
        result.header = copy.deepcopy(path.header)
        for path_point in path.points:
            pose_stamped = PoseStamped()
            pose_stamped.header = copy.deepcopy(path.header)
            pose_stamped.pose = copy.deepcopy(path_point.point.pose)
            result.poses.append(pose_stamped)
        return result

    @staticmethod
    def _object_half_dimensions(object_message: Any) -> tuple[float, float]:
        dimensions = getattr(getattr(object_message, "shape", None), "dimensions", None)
        if dimensions is None:
            return 1.5, 0.6
        length = max(abs(float(dimensions.x)), 0.8)
        width = max(abs(float(dimensions.y)), 0.6)
        # A conservative circle-like inflation avoids depending on object yaw.
        radius = 0.5 * math.hypot(length, width)
        return radius, max(0.5 * width, 0.4)

    @staticmethod
    def _object_speed(object_message: Any) -> float:
        kinematics = getattr(object_message, "kinematics", None)
        twist = getattr(kinematics, "initial_twist_with_covariance", None)
        if twist is None:
            return 0.0
        linear = twist.twist.linear
        return math.hypot(float(linear.x), float(linear.y))

    def _make_candidates(
        self,
        path: PathWithLaneId,
        geometry: PathGeometry,
        obstacles: list[Obstacle],
    ) -> list[Candidate]:
        keep_profile = [0.0] * len(path.points)
        candidates = [
            self._evaluate_candidate(
                "KEEP_LANE", path, geometry, keep_profile, obstacles
            )
        ]

        blocked = [
            obstacle
            for obstacle in obstacles
            if self._profile_collides_with_obstacle(
                geometry, keep_profile, obstacle
            )
        ]

        if blocked:
            candidates.append(
                self._make_shift_candidate(
                    "LEFT_SHIFT", 1.0, path, geometry, blocked, obstacles
                )
            )
            candidates.append(
                self._make_shift_candidate(
                    "RIGHT_SHIFT", -1.0, path, geometry, blocked, obstacles
                )
            )
            candidates.append(self._make_stop_candidate(path, geometry, blocked))

        return candidates

    def _make_shift_candidate(
        self,
        name: str,
        direction: float,
        path: PathWithLaneId,
        geometry: PathGeometry,
        blocked: list[Obstacle],
        all_obstacles: list[Obstacle],
    ) -> Candidate:
        vehicle_half_width = 0.5 * float(
            self.get_parameter("vehicle_width_m").value
        )
        hard_clearance = float(self.get_parameter("hard_clearance_m").value)
        clearance_buffer = float(
            self.get_parameter("candidate_clearance_buffer_m").value
        )
        min_shift = float(self.get_parameter("min_shift_m").value)
        max_shift = float(self.get_parameter("max_shift_m").value)

        if direction > 0.0:
            target = max(
                obstacle.l
                + obstacle.half_l
                + vehicle_half_width
                + hard_clearance
                + clearance_buffer
                for obstacle in blocked
            )
            target = max(target, min_shift)
        else:
            target = min(
                obstacle.l
                - obstacle.half_l
                - vehicle_half_width
                - hard_clearance
                - clearance_buffer
                for obstacle in blocked
            )
            target = min(target, -min_shift)

        if abs(target) > max_shift:
            return Candidate(
                name=name,
                path=copy.deepcopy(path),
                profile=[0.0] * len(path.points),
                valid=False,
                total_cost=1.0e9,
                components={},
                reason=f"required_shift_{target:.2f}m_exceeds_max_shift",
            )

        profile = self._make_shift_profile(geometry, target, blocked)
        shifted_path = self._apply_profile(path, geometry, profile)
        return self._evaluate_candidate(
            name, shifted_path, geometry, profile, all_obstacles
        )

    def _make_shift_profile(
        self, geometry: PathGeometry, target: float, blocked: list[Obstacle]
    ) -> list[float]:
        first = min(obstacle.s - obstacle.half_s for obstacle in blocked)
        last = max(obstacle.s + obstacle.half_s for obstacle in blocked)
        prepare = float(self.get_parameter("shift_prepare_distance_m").value)
        shift_length = max(float(self.get_parameter("shift_length_m").value), 1.0)
        return_length = max(float(self.get_parameter("return_length_m").value), 1.0)
        return_margin = float(self.get_parameter("return_margin_m").value)

        shift_finish = max(0.0, first - prepare)
        if shift_finish <= 0.0:
            # The obstacle is too close to complete the requested preparation
            # distance.  Start the maneuver at the ego point, but do not jump to
            # the lateral target there; the ramp will be checked for collision and
            # will normally make STOP_WAIT win when there is not enough room.
            shift_start = 0.0
            shift_finish = shift_length
        else:
            shift_start = max(0.0, shift_finish - shift_length)
        return_start = max(shift_finish, last + return_margin)
        return_finish = return_start + return_length

        profile: list[float] = []
        for s in geometry.arc_lengths:
            if s <= 0.0:
                # The published local path must pass exactly through the current
                # vehicle position.
                offset = 0.0
            elif s < shift_start:
                offset = 0.0
            elif s < shift_finish and shift_finish > shift_start:
                ratio = (s - shift_start) / (shift_finish - shift_start)
                offset = target * smoothstep(ratio)
            elif s < return_start:
                offset = target
            elif s < return_finish:
                ratio = (s - return_start) / (return_finish - return_start)
                offset = target * (1.0 - smoothstep(ratio))
            else:
                offset = 0.0
            profile.append(offset)
        return profile

    @staticmethod
    def _apply_profile(
        path: PathWithLaneId, geometry: PathGeometry, profile: list[float]
    ) -> PathWithLaneId:
        result = copy.deepcopy(path)
        shifted_points: list[tuple[float, float]] = []
        for index, ((x, y), yaw, offset) in enumerate(
            zip(geometry.points, geometry.yaws, profile)
        ):
            normal_x = -math.sin(yaw)
            normal_y = math.cos(yaw)
            shifted_points.append((x + offset * normal_x, y + offset * normal_y))
            result.points[index].point.pose.position.x = shifted_points[-1][0]
            result.points[index].point.pose.position.y = shifted_points[-1][1]

        for index, point in enumerate(result.points):
            if index == 0:
                dx = shifted_points[1][0] - shifted_points[0][0]
                dy = shifted_points[1][1] - shifted_points[0][1]
            elif index == len(shifted_points) - 1:
                dx = shifted_points[-1][0] - shifted_points[-2][0]
                dy = shifted_points[-1][1] - shifted_points[-2][1]
            else:
                dx = shifted_points[index + 1][0] - shifted_points[index - 1][0]
                dy = shifted_points[index + 1][1] - shifted_points[index - 1][1]
            if math.hypot(dx, dy) > 1.0e-6:
                set_pose_yaw(point.point.pose, math.atan2(dy, dx))
        return result

    def _make_stop_candidate(
        self,
        path: PathWithLaneId,
        geometry: PathGeometry,
        obstacles: list[Obstacle],
    ) -> Candidate:
        result = copy.deepcopy(path)
        profile = [0.0] * len(path.points)
        if obstacles:
            vehicle_half_length = 0.5 * float(
                self.get_parameter("vehicle_length_m").value
            )
            stop_margin = float(self.get_parameter("stop_margin_m").value)
            stop_s = max(
                0.0,
                min(
                    obstacle.s
                    - obstacle.half_s
                    - vehicle_half_length
                    - stop_margin
                    for obstacle in obstacles
                ),
            )
            for index, s in enumerate(geometry.arc_lengths):
                if s >= stop_s:
                    result.points[index].point.longitudinal_velocity_mps = 0.0

        stop_cost = float(self.get_parameter("stop_cost").value)
        if obstacles:
            nearest = max(0.0, min(obstacle.s for obstacle in obstacles))
            stop_cost += max(0.0, 20.0 - nearest) * 0.1
        return Candidate(
            name="STOP_WAIT",
            path=result,
            profile=profile,
            valid=True,
            total_cost=stop_cost,
            components={"stop": stop_cost},
        )

    def _evaluate_candidate(
        self,
        name: str,
        path: PathWithLaneId,
        geometry: PathGeometry,
        profile: list[float],
        obstacles: list[Obstacle],
    ) -> Candidate:
        valid = True
        reasons: list[str] = []
        vehicle_half_width = 0.5 * float(
            self.get_parameter("vehicle_width_m").value
        )
        vehicle_half_length = 0.5 * float(
            self.get_parameter("vehicle_length_m").value
        )
        hard_clearance = float(self.get_parameter("hard_clearance_m").value)

        boundary_cost = 0.0
        boundary_samples = 0
        for index, offset in enumerate(profile):
            left_width = geometry.left_widths[index]
            right_width = geometry.right_widths[index]
            if left_width is None or right_width is None:
                continue
            boundary_samples += 1
            left_margin = left_width - vehicle_half_width - hard_clearance - offset
            right_margin = right_width - vehicle_half_width - hard_clearance + offset
            if left_margin < 0.0 or right_margin < 0.0:
                valid = False
                reasons.append("outside_drivable_bound")
            boundary_cost += max(
                0.0,
                1.0 - min(left_margin, right_margin)
                / max(float(self.get_parameter("risk_clearance_m").value), 1.0e-3),
            )
        if boundary_samples:
            boundary_cost /= boundary_samples

        collision_count = 0
        clearance_cost = 0.0
        risk_cost = 0.0
        for obstacle in obstacles:
            min_lateral_clearance = float("inf")
            for index, s in enumerate(geometry.arc_lengths):
                if abs(s - obstacle.s) > obstacle.half_s + vehicle_half_length + 2.0:
                    continue
                lateral_clearance = abs(profile[index] - obstacle.l) - (
                    obstacle.half_l + vehicle_half_width
                )
                min_lateral_clearance = min(min_lateral_clearance, lateral_clearance)
                if (
                    abs(s - obstacle.s) <= obstacle.half_s + vehicle_half_length
                    and lateral_clearance < hard_clearance
                ):
                    collision_count += 1
                    valid = False
                    reasons.append(f"collision_with_{obstacle.object_id[:8]}")

            if min_lateral_clearance == float("inf"):
                continue
            risk_clearance = float(self.get_parameter("risk_clearance_m").value)
            clearance_cost += (
                max(0.0, risk_clearance - min_lateral_clearance)
                / max(risk_clearance, 1.0e-3)
            ) ** 2
            if obstacle.speed >= float(
                self.get_parameter("dynamic_speed_threshold_mps").value
            ):
                risk_cost += obstacle.existence * (1.0 + obstacle.speed) * math.exp(
                    -max(min_lateral_clearance, 0.0) / max(risk_clearance, 1.0e-3)
                )

        horizon = max(
            min(
                float(self.get_parameter("planning_horizon_m").value),
                geometry.arc_lengths[-1],
            ),
            1.0,
        )
        lateral_cost = sum(
            abs(profile[index])
            * max(geometry.arc_lengths[index] - geometry.arc_lengths[max(index - 1, 0)], 0.0)
            for index in range(len(profile))
        ) / horizon

        smoothness_cost = 0.0
        for index in range(1, len(profile) - 1):
            ds = max(
                geometry.arc_lengths[index + 1] - geometry.arc_lengths[index - 1],
                1.0e-3,
            )
            smoothness_cost += abs(
                profile[index + 1] - 2.0 * profile[index] + profile[index - 1]
            ) / ds

        switch_cost = 0.0
        if name != self.last_selected:
            switch_cost = float(self.get_parameter("switch_cost").value)

        components = {
            "lateral": lateral_cost,
            "smoothness": smoothness_cost,
            "clearance": clearance_cost,
            "risk": risk_cost,
            "boundary": boundary_cost,
            "speed_loss": 0.0,
            "switch": switch_cost,
            "collision_count": float(collision_count),
        }
        total_cost = (
            float(self.get_parameter("weights.lateral").value) * lateral_cost
            + float(self.get_parameter("weights.smoothness").value) * smoothness_cost
            + float(self.get_parameter("weights.clearance").value) * clearance_cost
            + float(self.get_parameter("weights.risk").value) * risk_cost
            + float(self.get_parameter("weights.boundary").value) * boundary_cost
            + float(self.get_parameter("weights.speed_loss").value) * 0.0
            + switch_cost
        )
        if not valid:
            total_cost = 1.0e9 + total_cost

        return Candidate(
            name=name,
            path=path,
            profile=profile,
            valid=valid,
            total_cost=total_cost,
            components=components,
            reason=";".join(sorted(set(reasons))),
        )

    def _profile_collides_with_obstacle(
        self, geometry: PathGeometry, profile: list[float], obstacle: Obstacle
    ) -> bool:
        vehicle_half_width = 0.5 * float(
            self.get_parameter("vehicle_width_m").value
        )
        vehicle_half_length = 0.5 * float(
            self.get_parameter("vehicle_length_m").value
        )
        hard_clearance = float(self.get_parameter("hard_clearance_m").value)
        for index, s in enumerate(geometry.arc_lengths):
            if abs(s - obstacle.s) > obstacle.half_s + vehicle_half_length:
                continue
            lateral_clearance = abs(profile[index] - obstacle.l) - (
                obstacle.half_l + vehicle_half_width
            )
            if lateral_clearance < hard_clearance:
                return True
        return False

    def _candidate_debug(self, candidate: Candidate) -> dict[str, Any]:
        return {
            "name": candidate.name,
            "valid": candidate.valid,
            "total_cost": round(float(candidate.total_cost), 4),
            "reason": candidate.reason,
            "components": {
                key: round(float(value), 4)
                for key, value in candidate.components.items()
            },
            "max_abs_lateral_shift": round(
                max((abs(value) for value in candidate.profile), default=0.0), 4
            ),
        }

    def _publish_debug(self, payload: dict[str, Any]) -> None:
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.debug_pub.publish(message)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = CostAwarePathProxy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
