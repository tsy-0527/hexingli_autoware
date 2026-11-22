#!/usr/bin/env python3
"""把 Autoware 定位和障碍物 bounding box 压缩成小型 UDP 数据报。"""

import json
import math
import socket
import time
import zlib

import rclpy
from autoware_perception_msgs.msg import PredictedObjects
from autoware_planning_msgs.msg import Path, Trajectory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def quaternion_yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def rounded(value: float) -> float:
    return round(float(value), 3)


class RosUdpTelemetryRelay(Node):
    def __init__(self):
        super().__init__("ros_udp_telemetry_relay")
        # 默认直接发送给三台 WireGuard 上位机。target_ip 保留为单目标兼容参数：
        # 只要显式传入 target_ip，就会覆盖 target_ips。
        self.declare_parameter("target_ip", "")
        self.declare_parameter("target_ips", ["10.66.0.3", "10.66.0.4", "10.66.0.5"])
        self.declare_parameter("target_port", 15000)
        self.declare_parameter("topic", "/localization/kinematic_state")
        self.declare_parameter("max_rate_hz", 10.0)
        self.declare_parameter("objects_enabled", True)
        self.declare_parameter("objects_topic", "/perception/object_recognition/objects")
        self.declare_parameter("objects_rate_hz", 5.0)
        self.declare_parameter("trajectory_enabled", True)
        self.declare_parameter("trajectory_topic", "/planning/scenario_planning/trajectory")
        self.declare_parameter("trajectory_rate_hz", 3.5)
        self.declare_parameter("trajectory_max_points", 100)
        self.declare_parameter("path_enabled", True)
        self.declare_parameter("path_topic", "/planning/scenario_planning/lane_driving/behavior_planning/path")
        self.declare_parameter("path_rate_hz", 1.0)
        self.declare_parameter("path_max_points", 300)
        self.declare_parameter("max_datagram_bytes", 1200)

        target_port = int(self.get_parameter("target_port").value)
        legacy_target_ip = str(self.get_parameter("target_ip").value).strip()
        configured_target_ips = [
            str(value).strip()
            for value in self.get_parameter("target_ips").value
            if str(value).strip()
        ]
        target_ips = [legacy_target_ip] if legacy_target_ip else configured_target_ips
        # 去重并保持配置顺序，避免同一上位机收到重复报文。
        target_ips = list(dict.fromkeys(target_ips))
        if not target_ips:
            raise ValueError("target_ip/target_ips 至少要配置一个 UDP 目标地址")
        self.targets = [(target_ip, target_port) for target_ip in target_ips]
        pose_topic = str(self.get_parameter("topic").value)
        pose_rate_hz = float(self.get_parameter("max_rate_hz").value)
        self.pose_min_period = 0.0 if pose_rate_hz <= 0 else 1.0 / pose_rate_hz
        self.pose_last_sent_at = 0.0
        self.pose_seq = 0

        self.objects_enabled = bool(self.get_parameter("objects_enabled").value)
        objects_topic = str(self.get_parameter("objects_topic").value)
        objects_rate_hz = float(self.get_parameter("objects_rate_hz").value)
        self.objects_min_period = 0.0 if objects_rate_hz <= 0 else 1.0 / objects_rate_hz
        self.objects_last_sent_at = 0.0
        self.objects_seq = 0
        self.max_datagram_bytes = max(512, min(1400, int(self.get_parameter("max_datagram_bytes").value)))

        self.trajectory_enabled = bool(self.get_parameter("trajectory_enabled").value)
        trajectory_topic = str(self.get_parameter("trajectory_topic").value)
        trajectory_rate_hz = float(self.get_parameter("trajectory_rate_hz").value)
        self.trajectory_min_period = 0.0 if trajectory_rate_hz <= 0 else 1.0 / trajectory_rate_hz
        self.trajectory_last_sent_at = 0.0
        self.trajectory_seq = 0
        self.trajectory_max_points = max(2, int(self.get_parameter("trajectory_max_points").value))

        self.path_enabled = bool(self.get_parameter("path_enabled").value)
        path_topic = str(self.get_parameter("path_topic").value)
        path_rate_hz = float(self.get_parameter("path_rate_hz").value)
        self.path_min_period = 0.0 if path_rate_hz <= 0 else 1.0 / path_rate_hz
        self.path_last_sent_at = 0.0
        self.path_seq = 0
        self.path_max_points = max(2, int(self.get_parameter("path_max_points").value))

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pose_subscription = self.create_subscription(
            Odometry,
            pose_topic,
            self.on_pose,
            qos_profile_sensor_data,
        )
        self.objects_subscription = None
        if self.objects_enabled:
            self.objects_subscription = self.create_subscription(
                PredictedObjects,
                objects_topic,
                self.on_objects,
                qos_profile_sensor_data,
            )
        self.trajectory_subscription = None
        if self.trajectory_enabled:
            self.trajectory_subscription = self.create_subscription(
                Trajectory,
                trajectory_topic,
                self.on_trajectory,
                qos_profile_sensor_data,
            )
        self.path_subscription = None
        if self.path_enabled:
            self.path_subscription = self.create_subscription(
                Path,
                path_topic,
                self.on_path,
                qos_profile_sensor_data,
            )

        self.get_logger().info(
            f"UDP telemetry -> {', '.join(f'{host}:{port}' for host, port in self.targets)}; "
            f"pose={pose_topic}@{pose_rate_hz:g}Hz; "
            f"objects={objects_topic}@{objects_rate_hz:g}Hz enabled={self.objects_enabled}; "
            f"trajectory={trajectory_topic}@{trajectory_rate_hz:g}Hz enabled={self.trajectory_enabled}; "
            f"path={path_topic}@{path_rate_hz:g}Hz enabled={self.path_enabled}"
        )

    def send_packet(self, packet: dict, label: str) -> None:
        payload = json.dumps(packet, separators=(",", ":"), allow_nan=False).encode("utf-8")
        if len(payload) > self.max_datagram_bytes:
            self.get_logger().warning(
                f"UDP {label} packet is {len(payload)} bytes (limit {self.max_datagram_bytes})",
                throttle_duration_sec=5.0,
            )
        for target in self.targets:
            try:
                self.socket.sendto(payload, target)
            except OSError as error:
                self.get_logger().error(
                    f"UDP send to {target[0]}:{target[1]} failed: {error}",
                    throttle_duration_sec=5.0,
                )

    def on_pose(self, msg: Odometry) -> None:
        now = time.monotonic()
        if self.pose_min_period > 0 and now - self.pose_last_sent_at < self.pose_min_period:
            return
        self.pose_last_sent_at = now
        self.pose_seq = (self.pose_seq + 1) & 0xFFFFFFFF

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        linear = msg.twist.twist.linear
        stamp = msg.header.stamp
        self.send_packet({
            "v": 1,
            "type": "pose",
            "seq": self.pose_seq,
            "stamp": [int(stamp.sec), int(stamp.nanosec)],
            "frame": msg.header.frame_id or "map",
            "p": [position.x, position.y, position.z],
            "q": [orientation.x, orientation.y, orientation.z, orientation.w],
            "linear": [linear.x, linear.y, linear.z],
        }, "pose")

    def object_box(self, obj, fallback_index: int):
        pose = obj.kinematics.initial_pose_with_covariance.pose
        shape = obj.shape
        dimensions = shape.dimensions
        yaw = quaternion_yaw(pose.orientation)
        center_x = float(pose.position.x)
        center_y = float(pose.position.y)
        center_z = float(pose.position.z)
        length = abs(float(dimensions.x))
        width = abs(float(dimensions.y))
        height = abs(float(dimensions.z))

        # POLYGON 的 dimensions.x/y 常为 0；取局部 footprint 的轴向包围框，并修正中心偏移。
        points = list(shape.footprint.points)
        if int(shape.type) == 2 and len(points) >= 3:
            min_x = min(float(point.x) for point in points)
            max_x = max(float(point.x) for point in points)
            min_y = min(float(point.y) for point in points)
            max_y = max(float(point.y) for point in points)
            local_x = (min_x + max_x) * 0.5
            local_y = (min_y + max_y) * 0.5
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            center_x += cos_yaw * local_x - sin_yaw * local_y
            center_y += sin_yaw * local_x + cos_yaw * local_y
            length = max_x - min_x
            width = max_y - min_y

        values = [center_x, center_y, center_z, yaw, length, width, height]
        if not all(math.isfinite(value) for value in values):
            return None
        if length <= 0 or width <= 0 or height <= 0:
            return None

        uuid_bytes = bytes(obj.object_id.uuid)
        short_id = f"{zlib.crc32(uuid_bytes) & 0xFFFFFFFF:08x}" if uuid_bytes else f"obj{fallback_index}"
        return [short_id, *(rounded(value) for value in values)]

    def chunked_packets(self, packet_type: str, seq: int, stamp, field: str, items: list) -> list:
        chunks = [[]]
        # 先按实际 JSON 字节数装箱，保证每个 UDP 数据报尽量不超过配置上限。
        for item in items:
            candidate = chunks[-1] + [item]
            probe = {
                "v": 1,
                "type": packet_type,
                "seq": seq,
                "stamp": stamp,
                "part": 63,
                "parts": 64,
                field: candidate,
            }
            probe_size = len(json.dumps(probe, separators=(",", ":"), allow_nan=False).encode("utf-8"))
            if chunks[-1] and probe_size > self.max_datagram_bytes:
                chunks.append([item])
            else:
                chunks[-1] = candidate

        parts = len(chunks)
        return [{
            "v": 1,
            "type": packet_type,
            "seq": seq,
            "stamp": stamp,
            "part": part,
            "parts": parts,
            field: chunk,
        } for part, chunk in enumerate(chunks)]

    @staticmethod
    def sampled(items, limit: int):
        count = len(items)
        if count <= limit:
            return items
        # 均匀抽样并保留首尾点，避免只截取路径前半段。
        indexes = [round(index * (count - 1) / (limit - 1)) for index in range(limit)]
        return [items[index] for index in indexes]

    def on_objects(self, msg: PredictedObjects) -> None:
        now = time.monotonic()
        if self.objects_min_period > 0 and now - self.objects_last_sent_at < self.objects_min_period:
            return
        self.objects_last_sent_at = now
        self.objects_seq = (self.objects_seq + 1) & 0xFFFFFFFF
        boxes = []
        for index, obj in enumerate(msg.objects):
            box = self.object_box(obj, index)
            if box is not None:
                boxes.append(box)
        stamp = [int(msg.header.stamp.sec), int(msg.header.stamp.nanosec)]
        for packet in self.chunked_packets("objects", self.objects_seq, stamp, "o", boxes):
            self.send_packet(packet, "objects")

    def on_trajectory(self, msg: Trajectory) -> None:
        now = time.monotonic()
        if self.trajectory_min_period > 0 and now - self.trajectory_last_sent_at < self.trajectory_min_period:
            return
        self.trajectory_last_sent_at = now
        self.trajectory_seq = (self.trajectory_seq + 1) & 0xFFFFFFFF
        points = self.sampled(list(msg.points), self.trajectory_max_points)
        compact = [[rounded(point.pose.position.x), rounded(point.pose.position.y)] for point in points]
        stamp = [int(msg.header.stamp.sec), int(msg.header.stamp.nanosec)]
        for packet in self.chunked_packets("trajectory", self.trajectory_seq, stamp, "p", compact):
            self.send_packet(packet, "trajectory")

    def on_path(self, msg: Path) -> None:
        now = time.monotonic()
        if self.path_min_period > 0 and now - self.path_last_sent_at < self.path_min_period:
            return
        self.path_last_sent_at = now
        self.path_seq = (self.path_seq + 1) & 0xFFFFFFFF
        points = self.sampled(list(msg.points), self.path_max_points)
        compact = [[
            rounded(point.pose.position.x),
            rounded(point.pose.position.y),
            rounded(point.pose.position.z),
        ] for point in points]
        stamp = [int(msg.header.stamp.sec), int(msg.header.stamp.nanosec)]
        for packet in self.chunked_packets("path", self.path_seq, stamp, "p", compact):
            self.send_packet(packet, "path")

    def destroy_node(self):
        self.socket.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RosUdpTelemetryRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
