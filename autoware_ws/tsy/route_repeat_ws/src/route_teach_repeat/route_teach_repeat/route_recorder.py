#!/usr/bin/env python3
"""Record Lanelet2 route segments while driving.

The recorder can consume either the final Autoware odometry or the raw
LiDAR/NDT pose.  It matches the vehicle position and heading to lanelets,
then saves the visited lanelet id sequence and goal pose to YAML on shutdown.
"""
import math
import os
import yaml
from datetime import datetime

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

import lanelet2
from lanelet2.projection import LocalCartesianProjector, UtmProjector
from lanelet2.core import BasicPoint2d, Lanelet
from lanelet2 import matching, traffic_rules, routing


class RouteRecorder(Node):
    def __init__(self):
        super().__init__('route_recorder')

        # ---- Parameters ----
        self.declare_parameter('map_path', '')
        self.declare_parameter(
            'output_path',
            '/home/tang/tsy/route_repeat_ws/recorded_route.yaml')
        self.declare_parameter('projector_type', 'mgrs')          # 'mgrs' or 'utm'
        self.declare_parameter('map_origin_lat', 0.0)             # utm only
        self.declare_parameter('map_origin_lon', 0.0)             # utm only
        self.declare_parameter('localization_mode', 'lidar_ndt')
        # The running Autoware stack in this vehicle publishes the direct NDT
        # LiDAR localization result as PoseStamped.  Keep both the topic and
        # message type configurable so simulation/fused-odometry can still be
        # selected without changing source code.
        self.declare_parameter('pose_topic', '/localization/pose_estimator/pose')
        self.declare_parameter('pose_msg_type', 'pose_stamped')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('allow_lane_change', False)
        self.declare_parameter('match_distance', 3.0)
        self.declare_parameter('vehicle_length', 4.0)
        self.declare_parameter('vehicle_width', 1.8)
        self.declare_parameter('min_stable_hits', 3)              # 需连续 N 帧匹配同一 id 才切换
        self.declare_parameter('save_centerline_for_viz', True)   # 存中心线便于回放可视化

        map_path = self.get_parameter('map_path').value
        if not map_path:
            self.get_logger().error('map_path is required')
            raise RuntimeError('map_path is empty')
        self.output_path = self.get_parameter('output_path').value
        self.localization_mode = str(
            self.get_parameter('localization_mode').value).lower()
        if self.localization_mode not in ('lidar_ndt', 'gnss'):
            raise ValueError("localization_mode must be 'lidar_ndt' or 'gnss'")
        pose_topic = self.get_parameter('pose_topic').value
        pose_msg_type = str(self.get_parameter('pose_msg_type').value).lower()
        pose_msg_type = {
            'pose': 'pose_stamped',
            'posewithcovariancestamped': 'pose_with_covariance',
            'pose_with_covariance_stamped': 'pose_with_covariance',
        }.get(pose_msg_type, pose_msg_type)
        self.pose_msg_type = pose_msg_type
        self.map_frame = self.get_parameter('map_frame').value
        pose_msg_classes = {
            'odometry': Odometry,
            'pose_stamped': PoseStamped,
            'pose_with_covariance': PoseWithCovarianceStamped,
        }
        if pose_msg_type not in pose_msg_classes:
            raise ValueError(
                "pose_msg_type must be one of: odometry, pose_stamped, "
                "pose_with_covariance")

        self.pose_topic = pose_topic
        self.allow_lc = self.get_parameter('allow_lane_change').value
        self.match_dist = self.get_parameter('match_distance').value
        self.veh_len = self.get_parameter('vehicle_length').value
        self.veh_wid = self.get_parameter('vehicle_width').value
        self.min_stable = self.get_parameter('min_stable_hits').value
        self.save_cl = self.get_parameter('save_centerline_for_viz').value

        # ---- Load map ----
        proj_type = self.get_parameter('projector_type').value
        lat = self.get_parameter('map_origin_lat').value
        lon = self.get_parameter('map_origin_lon').value
        if proj_type == 'local_cartesian':
            projector = LocalCartesianProjector(lanelet2.io.Origin(lat, lon))
        else:
            projector = UtmProjector(lanelet2.io.Origin(lat, lon))

        self.lanelet_map = lanelet2.io.load(map_path, projector)
        self.rules = traffic_rules.create(
            traffic_rules.Locations.Germany,
            traffic_rules.Participants.Vehicle)
        self.routing_graph = routing.RoutingGraph(self.lanelet_map, self.rules)
        self.get_logger().info(
            f'Loaded {len(self.lanelet_map.laneletLayer)} lanelets from {map_path}')

        # ---- State ----
        self.segments = []          # list of dict: {preferred, primitives, centerline}
        self.last_pose = None       # geometry_msgs Pose
        self.candidate_id = None    # id being considered
        self.candidate_hits = 0     # 连续命中次数
        self.pose_count = 0         # 收到的定位帧总数
        self.unmatched_pose_count = 0
        self.frame_mismatch_count = 0

        # ---- Subs / Pubs ----
        self.sub = self.create_subscription(
            pose_msg_classes[pose_msg_type], pose_topic, self.on_pose, 10)

        self.marker_pub = self.create_publisher(
            MarkerArray, '/route_recorder/markers', 10)
        self.path_pub = self.create_publisher(
            Path, '/route_recorder/vehicle_path', 10)

        self.vehicle_path = Path()
        self.vehicle_path.header.frame_id = self.map_frame

        self.viz_timer = self.create_timer(0.2, self.publish_viz)
        self.status_timer = self.create_timer(2.0, self.publish_status)

        self.get_logger().info(
            f'Recorder ready. Subscribing to {pose_topic} '
            f'({pose_msg_type}, frame={self.map_frame}, '
            f'mode={self.localization_mode}). Ctrl+C to save and quit.')

    # ------------- Callbacks -------------

    def on_pose(self, msg):
        self.pose_count += 1
        if self.pose_msg_type == 'odometry':
            pose = msg.pose.pose
        elif self.pose_msg_type == 'pose_stamped':
            pose = msg.pose
        else:
            pose = msg.pose.pose

        # Lanelet coordinates and the visualization are in map coordinates.
        # Do not silently match a base_link/odom pose against the map: that
        # would create a plausible-looking but invalid route.
        frame_id = msg.header.frame_id.strip()
        if frame_id and frame_id != self.map_frame:
            self.frame_mismatch_count += 1
            self.get_logger().warn(
                f'Ignoring pose in frame {frame_id!r}; expected {self.map_frame!r}',
                throttle_duration_sec=2.0)
            return

        x, y = pose.position.x, pose.position.y
        yaw = self._quat_to_yaw(pose.orientation)

        matched = self._match_lanelet(x, y, yaw)
        if matched is None:
            self.unmatched_pose_count += 1
            self.get_logger().warn(
                f'No lanelet matched at ({x:.2f}, {y:.2f})', throttle_duration_sec=2.0)
            self.last_pose = pose
            self._append_path(msg.header, pose)
            return

        # 稳定性过滤：避免边界抖动
        if self.candidate_id != matched.id:
            self.candidate_id = matched.id
            self.candidate_hits = 1
        else:
            self.candidate_hits += 1

        if self.candidate_hits >= self.min_stable:
            if not self.segments or self.segments[-1]['preferred'] != matched.id:
                self._add_segment(matched)

        self.last_pose = pose
        self._append_path(msg.header, pose)

    def publish_status(self):
        """定期报告录制状态，区分节点启动和实际收到定位。"""
        if self.pose_count == 0:
            self.get_logger().warn(
                f'Recording has not started: no pose received on {self.pose_topic}')
            return

        if self.last_pose is None:
            return

        p = self.last_pose.position
        self.get_logger().info(
            f'Recording active: poses={self.pose_count}, '
            f'unmatched={self.unmatched_pose_count}, '
            f'frame_mismatch={self.frame_mismatch_count}, '
            f'segments={len(self.segments)}, '
            f'last_pose=({p.x:.2f}, {p.y:.2f})')

    # ------------- Helpers -------------

    def _match_lanelet(self, x, y, yaw):
        """用位置 + 航向匹配 lanelet。不用 hull（本版 lanelet2 无 Hull2d）。"""
        import math as _m
        pt = BasicPoint2d(x, y)
        # 找搜索半径内所有 lanelet
        nearby = lanelet2.geometry.findWithin2d(
            self.lanelet_map.laneletLayer, pt, self.match_dist)
        if not nearby:
            return None

        best = None
        best_score = float('inf')
        for dist, ll in nearby:
            # 计算 lanelet 中心线在最近点处的切向方向
            cl = ll.centerline
            # 找最近中心线段
            min_d = float('inf')
            seg_yaw = 0.0
            for i in range(len(cl) - 1):
                x1, y1 = cl[i].x, cl[i].y
                x2, y2 = cl[i+1].x, cl[i+1].y
                # 点到线段的近似距离
                dx, dy = x2 - x1, y2 - y1
                seglen2 = dx*dx + dy*dy
                if seglen2 < 1e-9:
                    continue
                t = max(0.0, min(1.0, ((x - x1)*dx + (y - y1)*dy) / seglen2))
                px, py = x1 + t*dx, y1 + t*dy
                d = _m.hypot(x - px, y - py)
                if d < min_d:
                    min_d = d
                    seg_yaw = _m.atan2(dy, dx)
            # 航向差 (归一化到 [-pi, pi])
            dyaw = yaw - seg_yaw
            while dyaw > _m.pi:
                dyaw -= 2 * _m.pi
            while dyaw < -_m.pi:
                dyaw += 2 * _m.pi
            # 过滤反向车道
            if abs(dyaw) > _m.pi / 2:
                continue
            # 综合评分：距离 + 航向偏差权重
            score = min_d + 2.0 * abs(dyaw)
            if score < best_score:
                best_score = score
                best = ll
        return best

    def _add_segment(self, ll: Lanelet):
        primitives = [int(ll.id)]
        if self.allow_lc:
            left = self.routing_graph.left(ll)
            right = self.routing_graph.right(ll)
            if left is not None:
                primitives.append(int(left.id))
            if right is not None:
                primitives.append(int(right.id))

        seg = {
            'preferred': int(ll.id),
            'primitives': primitives,
        }
        if self.save_cl:
            seg['centerline'] = [
                [float(p.x), float(p.y), float(getattr(p, 'z', 0.0))]
                for p in ll.centerline
            ]
        self.segments.append(seg)
        self.get_logger().info(
            f'[seg {len(self.segments)}] lanelet={ll.id}, primitives={primitives}')

    def _append_path(self, header, pose):
        ps = PoseStamped()
        ps.header.stamp = header.stamp
        ps.header.frame_id = self.map_frame
        ps.pose = pose
        self.vehicle_path.poses.append(ps)
        if len(self.vehicle_path.poses) > 5000:
            self.vehicle_path.poses = self.vehicle_path.poses[-5000:]

    @staticmethod
    def _quat_to_yaw(q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    # ------------- Visualization -------------

    def publish_viz(self):
        self.vehicle_path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.vehicle_path)

        markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        clear = Marker()
        clear.header.frame_id = self.map_frame
        clear.header.stamp = now
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for i, seg in enumerate(self.segments):
            is_current = (i == len(self.segments) - 1)
            ll = self.lanelet_map.laneletLayer[seg['preferred']]

            line = Marker()
            line.header.frame_id = self.map_frame
            line.header.stamp = now
            line.ns = 'centerline'
            line.id = i
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.5 if is_current else 0.25
            if is_current:
                line.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)
            else:
                line.color = ColorRGBA(r=0.2, g=1.0, b=0.2, a=0.8)
            for pt in ll.centerline:
                p = Point()
                p.x = float(pt.x)
                p.y = float(pt.y)
                p.z = float(getattr(pt, 'z', 0.0))
                line.points.append(p)
            markers.markers.append(line)

            # id label
            mid = ll.centerline[len(ll.centerline) // 2]
            text = Marker()
            text.header.frame_id = self.map_frame
            text.header.stamp = now
            text.ns = 'id_label'
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(mid.x)
            text.pose.position.y = float(mid.y)
            text.pose.position.z = float(getattr(mid, 'z', 0.0)) + 2.0
            text.pose.orientation.w = 1.0
            text.scale.z = 1.5
            text.color = ColorRGBA(r=1.0, g=1.0, b=0.2, a=1.0)
            text.text = f'#{i}:{seg["preferred"]}'
            markers.markers.append(text)

        self.marker_pub.publish(markers)

    # ------------- Save -------------

    def save(self):
        if not self.segments:
            if self.pose_count > 0:
                self.get_logger().error(
                    f'Received {self.pose_count} poses, but no lanelet was matched; '
                    'route YAML was not written.')
            else:
                self.get_logger().warn(
                    'No pose received and no segments recorded; route YAML was not written.')
            return
        if self.last_pose is None:
            self.get_logger().warn('No last pose available, goal will be empty.')
            return

        data = {
            'header': {
                'map_frame': self.map_frame,
                'recorded_at': datetime.now().isoformat(),
                'localization_mode': self.localization_mode,
                'allow_lane_change': self.allow_lc,
                'num_segments': len(self.segments),
            },
            'segments': self.segments,
            'goal_pose': {
                'x': float(self.last_pose.position.x),
                'y': float(self.last_pose.position.y),
                'z': float(self.last_pose.position.z),
                'qx': float(self.last_pose.orientation.x),
                'qy': float(self.last_pose.orientation.y),
                'qz': float(self.last_pose.orientation.z),
                'qw': float(self.last_pose.orientation.w),
            },
        }
        output_dir = os.path.dirname(os.path.abspath(self.output_path))
        os.makedirs(output_dir, exist_ok=True)
        with open(self.output_path, 'w') as f:
            yaml.safe_dump(data, f, sort_keys=False)
        self.get_logger().info(
            f'Saved route successfully: poses={self.pose_count}, '
            f'unmatched={self.unmatched_pose_count}, '
            f'frame_mismatch={self.frame_mismatch_count}, '
            f'segments={len(self.segments)}, file={self.output_path}')


def main():
    rclpy.init()
    node = RouteRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
