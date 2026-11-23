"""ROS 2 node: room-constrained standalone global initialization.

Flow (state machine on a 10 Hz timer):
  IDLE --(~/trigger or auto_start)--> WAIT_STOP --> ACCUMULATE --> SEARCH
       --> publish PoseWithCovarianceStamped --> DONE / FAILED

Status is published as a JSON string on ~/status (transient local), so the
Electron UI / Agent can read it through rosbridge.
"""
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker
import tf2_ros

try:
    from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
    from autoware_adapi_v1_msgs.srv import InitializeLocalization
    from autoware_internal_localization_msgs.srv import PoseWithCovarianceStamped as AlignPose
except ImportError:
    # Keep the direct MID-360S/standalone mode usable without sourcing an
    # Autoware workspace. These interfaces are required only when the launch
    # file explicitly enables use_autoware_ndt.
    LocalizationInitializationState = None
    InitializeLocalization = None
    AlignPose = None

from .core import GridMap, Params, RoomGlobalLocalizer, choose_ndt_winner


def _quat_to_mat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _mat_to_quat(R):
    """Convert a 3x3 rotation matrix to geometry_msgs quaternion order x,y,z,w."""
    q = np.empty(4, float)
    tr = np.trace(R)
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        q[:] = [(R[2, 1] - R[1, 2]) / s,
                (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(R)))
        if i == 0:
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            q[:] = [0.25 * s, (R[0, 1] + R[1, 0]) / s,
                    (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s]
        elif i == 1:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            q[:] = [(R[0, 1] + R[1, 0]) / s, 0.25 * s,
                    (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s]
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            q[:] = [(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s,
                    0.25 * s, (R[1, 0] - R[0, 1]) / s]
    return q / np.linalg.norm(q)


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _angle_diff(a, b):
    return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)


class RoomGlobalLocalizerNode(Node):
    def __init__(self):
        super().__init__('room_global_localizer')
        d = self.declare_parameter
        d('loc_map_yaml', '')
        d('nav_map_yaml', '')
        d('pointcloud_topic', '/sensing/lidar/concatenated/pointcloud')
        d('velocity_topic', '/vehicle/status/velocity_status')
        d('base_frame', 'base_link')
        d('map_frame', 'map')
        d('pose_topic', '/room_global_localizer/pose')
        d('use_autoware_ndt', False)
        d('autoware_init_timeout_sec', 30.0)
        d('ndt_candidate_count', 3)
        d('ndt_min_seed_score', 0.60)
        d('ndt_cluster_position_tolerance', 0.75)
        d('ndt_cluster_yaw_tolerance_deg', 10.0)
        d('internal_to_output_transform', [1.0, 0.0, 0.0, 0.0,
                                           0.0, 1.0, 0.0, 0.0,
                                           0.0, 0.0, 1.0, 0.0,
                                           0.0, 0.0, 0.0, 1.0])
        d('auto_start', True)
        d('auto_start_delay', 5.0)
        d('use_velocity_check', True)
        d('stop_speed_threshold', 0.02)
        d('stop_hold_sec', 1.0)
        d('accumulate_sec', 1.0)
        d('init_timeout_sec', 15.0)
        d('max_attempts', 2)
        d('cov_xy', 0.05 ** 2)
        d('cov_yaw', math.radians(2.0) ** 2)
        defaults = Params()
        d('room_polygon', [float(v) for xy in defaults.room_polygon for v in xy])  # flat [x1,y1,x2,y2,...]
        for k in ['min_clearance', 'z_min', 'z_max', 'r_min', 'r_max', 'voxel', 'sigma',
                  'coarse_res', 'coarse_yaw_step_deg', 'min_score', 'max_ratio']:
            d(k, float(getattr(defaults, k)))
        d('self_box', [float(v) for v in defaults.self_box])
        d('min_points', int(defaults.min_points))

        g = lambda k: self.get_parameter(k).value  # noqa
        params = Params()
        poly = g('room_polygon')
        params.room_polygon = [[poly[i], poly[i + 1]] for i in range(0, len(poly), 2)]
        for k in ['min_clearance', 'z_min', 'z_max', 'r_min', 'r_max', 'voxel', 'sigma',
                  'coarse_res', 'coarse_yaw_step_deg', 'min_score', 'max_ratio']:
            setattr(params, k, float(g(k)))
        params.self_box = list(g('self_box'))
        params.min_points = int(g('min_points'))

        t = time.time()
        self.loc = RoomGlobalLocalizer(GridMap.load(g('loc_map_yaml')), GridMap.load(g('nav_map_yaml')), params)
        self.get_logger().info(f'maps loaded in {time.time() - t:.2f}s, allowed cells in room: {int(self.loc.allowed.sum())}')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(PointCloud2, g('pointcloud_topic'), self._on_cloud, qos_profile_sensor_data)
        self.speed = None
        self.stop_since = None
        if g('use_velocity_check'):
            self.get_logger().warn(
                'use_velocity_check requested, but standalone mode has no fixed velocity message type; disabled')
        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.status_pub = self.create_publisher(String, '~/status', state_qos)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, g('pose_topic'), state_qos)
        self.candidate_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '~/candidate_pose', state_qos)
        self.aligned_cloud_pub = self.create_publisher(PointCloud2, '~/aligned_cloud', state_qos)
        self.result_marker_pub = self.create_publisher(Marker, '~/result_marker', state_qos)
        self.use_autoware_ndt = bool(g('use_autoware_ndt'))
        self.init_state = None
        self.latest_kinematic = None
        self.init_cli = None
        self.ndt_align_cli = None
        if self.use_autoware_ndt:
            if any(interface is None for interface in (
                    LocalizationInitializationState, InitializeLocalization, AlignPose)):
                raise RuntimeError(
                    'use_autoware_ndt=true requires sourcing the Autoware workspace')
            self.init_cli = self.create_client(
                InitializeLocalization, '/api/localization/initialize')
            self.create_subscription(
                LocalizationInitializationState,
                '/api/localization/initialization_state',
                self._on_init_state, state_qos)
            self.create_subscription(
                Odometry, '/localization/kinematic_state',
                self._on_kinematic_state, 10)
            self.ndt_align_cli = self.create_client(
                AlignPose, '/localization/pose_estimator/ndt_align_srv')
            self.get_logger().info(
                'Autoware NDT refinement enabled: /api/localization/initialize')
        self.create_service(Trigger, '~/trigger', self._on_trigger)

        self.state = 'IDLE'
        self.attempt = 0
        self.buf = []
        self.t_state = time.time()
        self.result = {}
        self.future = None
        self.ndt_candidates = []
        self.ndt_results = []
        self.ndt_index = 0
        self.start_time = time.time()
        self.auto_pending = bool(g('auto_start'))
        self.create_timer(0.1, self._tick)
        self._publish('IDLE', 'waiting for trigger' if not self.auto_pending else 'auto start pending')

    # ---------------- callbacks ----------------
    def _on_init_state(self, msg):
        self.init_state = msg.state

    def _on_kinematic_state(self, msg):
        self.latest_kinematic = msg

    def _on_trigger(self, req, resp):
        if self.state in (
                'WAIT_STOP', 'ACCUMULATE', 'SEARCH', 'NDT_VALIDATE',
                'CALL_INIT', 'WAIT_INIT'):
            resp.success = False
            resp.message = f'busy ({self.state})'
            return resp
        self.attempt = 0
        self.result = {}
        self.ndt_candidates = []
        self.ndt_results = []
        self.ndt_index = 0
        self._goto('WAIT_STOP')
        resp.success = True
        resp.message = 'started'
        return resp

    def _on_cloud(self, msg):
        if self.state != 'ACCUMULATE':
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.get_parameter('base_frame').value, msg.header.frame_id,
                                                 rclpy.time.Time())
        except Exception as e:  # noqa
            self.get_logger().warn(f'TF {msg.header.frame_id}->base_link not available: {e}', throttle_duration_sec=2.0)
            return
        arr = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if getattr(arr, 'dtype', None) is not None and arr.dtype.names:
            pts = np.stack([arr['x'], arr['y'], arr['z']], axis=1).astype(float)
        else:
            pts = np.array(list(arr), float).reshape(-1, 3)
        R = _quat_to_mat(tf.transform.rotation)
        tr = tf.transform.translation
        self.buf.append(pts @ R.T + np.array([tr.x, tr.y, tr.z]))

    # ---------------- state machine ----------------
    def _goto(self, s, msg=''):
        self.state = s
        self.t_state = time.time()
        self._publish(s, msg)

    def _publish(self, state, msg, **extra):
        body = dict(state=state, message=msg, attempt=self.attempt, **extra)
        if self.result:
            body['result'] = self.result
        self.status_pub.publish(String(data=json.dumps(body, ensure_ascii=False)))
        self.get_logger().info(f'[{state}] {msg}')

    def _make_output_pose(self, result):
        """Convert an internal candidate to the configured output map frame.

        The production configuration uses an identity transform, so the output
        remains in the levelled internal map coordinates.
        """
        g = lambda k: self.get_parameter(k).value  # noqa
        yaw = math.radians(result['yaw_deg'])
        c, s = math.cos(yaw), math.sin(yaw)
        T_internal_base = np.array([
            [c, -s, 0.0, result['x']],
            [s, c, 0.0, result['y']],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]])
        T_output_internal = np.asarray(g('internal_to_output_transform'), float).reshape(4, 4)
        T_output_base = T_output_internal @ T_internal_base
        q = _mat_to_quat(T_output_base[:3, :3])

        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = g('map_frame')
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = float(T_output_base[0, 3])
        pose.pose.pose.position.y = float(T_output_base[1, 3])
        pose.pose.pose.position.z = float(T_output_base[2, 3])
        pose.pose.pose.orientation.x = float(q[0])
        pose.pose.pose.orientation.y = float(q[1])
        pose.pose.pose.orientation.z = float(q[2])
        pose.pose.pose.orientation.w = float(q[3])
        cov = [0.0] * 36
        cov[0] = cov[7] = g('cov_xy')
        cov[14] = 0.01
        cov[21] = cov[28] = math.radians(1.0) ** 2
        cov[35] = g('cov_yaw')
        pose.pose.covariance = cov
        return pose, T_output_base

    def _publish_candidate_debug(self, result, points_base, pose, T_output_base):
        """Latch the best candidate, aligned scan and a readable RViz status label."""
        self.candidate_pose_pub.publish(pose)

        pts = points_base[np.isfinite(points_base).all(axis=1)]
        ranges = np.hypot(pts[:, 0], pts[:, 1])
        pts = pts[(ranges > 0.3) & (ranges < 20.0)]
        if len(pts) > 30000:
            idx = np.linspace(0, len(pts) - 1, 30000, dtype=int)
            pts = pts[idx]
        aligned = pts @ T_output_base[:3, :3].T + T_output_base[:3, 3]
        header = Header(stamp=pose.header.stamp, frame_id=pose.header.frame_id)
        self.aligned_cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, aligned.astype(np.float32)))

        marker = Marker()
        marker.header = header
        marker.ns = 'room_global_localizer'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = pose.pose.pose.position.x
        marker.pose.position.y = pose.pose.pose.position.y
        marker.pose.position.z = pose.pose.pose.position.z + 1.5
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.45
        accepted = bool(result['accepted'])
        ndt_pending = (
            self.use_autoware_ndt and
            result.get('score', 0.0) >=
            float(self.get_parameter('ndt_min_seed_score').value))
        marker.color.r = 1.0 if ndt_pending else (0.1 if accepted else 1.0)
        marker.color.g = 0.8 if ndt_pending else (1.0 if accepted else 0.25)
        marker.color.b = 0.1
        marker.color.a = 1.0
        decision = ('NDT VALIDATING' if ndt_pending else
                    ('ACCEPTED' if accepted else 'REJECTED'))
        marker.text = decision + \
            f"  score={result['score']:.3f}  ratio={result['ratio']:.3f}"
        self.last_result_marker = marker
        self.result_marker_pub.publish(marker)

    def _publish_final_marker(self, text, accepted, pose=None):
        """Replace the latched search-stage label with the final NDT decision."""
        marker = getattr(self, 'last_result_marker', None)
        if marker is None:
            return
        marker.header.stamp = self.get_clock().now().to_msg()
        if pose is not None:
            marker.pose.position.x = pose.pose.pose.position.x
            marker.pose.position.y = pose.pose.pose.position.y
            marker.pose.position.z = pose.pose.pose.position.z + 1.5
        marker.color.r = 0.1 if accepted else 1.0
        marker.color.g = 1.0 if accepted else 0.2
        marker.color.b = 0.1
        marker.text = text
        self.result_marker_pub.publish(marker)

    def _build_ndt_candidates(self, result, raw_candidates):
        """Build distinct task5-frame NDT seeds from the 2D Top-K results."""
        specs = list(raw_candidates or [])
        if result.get('second_pose'):
            specs.append([*result['second_pose'], result.get('second_score', 0.0)])
        selected = []
        limit = int(self.get_parameter('ndt_candidate_count').value)
        for x, y, yaw_deg, score in specs:
            if any(math.hypot(x - s['x'], y - s['y']) < 0.75 and
                   abs((yaw_deg - s['yaw_deg'] + 180.0) % 360.0 - 180.0) < 15.0
                   for s in selected):
                continue
            seed = {'x': float(x), 'y': float(y), 'yaw_deg': float(yaw_deg),
                    'score': float(score)}
            pose, _ = self._make_output_pose(seed)
            selected.append({**seed, 'pose': pose})
            if len(selected) >= limit:
                break
        return selected

    def _call_next_ndt_candidate(self):
        if self.ndt_index >= len(self.ndt_candidates):
            self._finish_ndt_validation()
            return
        candidate = self.ndt_candidates[self.ndt_index]
        request = AlignPose.Request()
        request.pose_with_covariance = candidate['pose']
        self.future = self.ndt_align_cli.call_async(request)
        self.t_state = time.time()
        self._publish(
            'NDT_VALIDATE',
            f"aligning candidate {self.ndt_index + 1}/{len(self.ndt_candidates)}: "
            f"x={candidate['x']:.2f} y={candidate['y']:.2f} "
            f"yaw={candidate['yaw_deg']:.1f} score={candidate['score']:.3f}")

    def _finish_ndt_validation(self):
        reliable = [r for r in self.ndt_results if r['success'] and r['reliable']]
        summary = [{
            'seed_index': r['seed_index'], 'success': r['success'],
            'reliable': r['reliable'], 'x': r.get('x'), 'y': r.get('y'),
            'z': r.get('z'), 'yaw_deg': r.get('yaw_deg'),
            'seed_score': self.ndt_candidates[r['seed_index']]['score']}
            for r in self.ndt_results]
        self.result['ndt_candidates'] = summary
        if not reliable:
            self._retry_or_fail('NDT rejected every 2D candidate')
            return

        pos_tol = float(self.get_parameter('ndt_cluster_position_tolerance').value)
        yaw_tol = math.radians(
            float(self.get_parameter('ndt_cluster_yaw_tolerance_deg').value))
        # Candidate 0 is the strongest independent 2D observation. Multiple
        # nearby lower-ranked seeds are correlated samples, not independent
        # votes; allowing two of them to outvote candidate 0 can select the
        # wrong half of a symmetric room. If candidate 0 passes Autoware's NDT
        # reliability check, use its refined 6-DoF pose. Other candidates are
        # diagnostic only and may not replace an unreliable primary.
        winner, validation, cluster_support, cluster_count = choose_ndt_winner(
            reliable, self.ndt_candidates, pos_tol, yaw_tol)
        if winner is None:
            self._retry_or_fail(
                'primary NDT candidate is not reliable; '
                f'{len(reliable)} lower-ranked reliable results are diagnostic only')
            return

        refined = winner['pose']
        refined.header.stamp = self.get_clock().now().to_msg()
        self.candidate_pose_pub.publish(refined)
        self.result.update(
            ndt_validation=validation,
            ndt_cluster_support=cluster_support,
            ndt_x=winner['x'], ndt_y=winner['y'], ndt_z=winner['z'],
            ndt_yaw_deg=winner['yaw_deg'])

        request = InitializeLocalization.Request()
        request.pose = [refined]
        self.future = self.init_cli.call_async(request)
        self._goto(
            'CALL_INIT',
            f"NDT {validation} selected from {len(reliable)} reliable candidates; "
            'sent to Autoware initializer')

    def _tick(self):
        g = lambda k: self.get_parameter(k).value  # noqa
        now = time.time()
        if self.auto_pending and now - self.start_time > g('auto_start_delay'):
            self.auto_pending = False
            self.attempt = 0
            self._goto('WAIT_STOP', 'auto start')

        if self.state == 'WAIT_STOP':
            if not g('use_velocity_check'):
                self.buf = []
                self._goto('ACCUMULATE')
                return
            self.buf = []
            self._goto('ACCUMULATE', 'standalone mode: velocity check disabled')

        elif self.state == 'ACCUMULATE':
            if now - self.t_state > g('accumulate_sec') and self.buf:
                self._goto('SEARCH', f'{len(self.buf)} clouds accumulated')
            elif now - self.t_state > 10.0:
                self._fail('no point cloud received (check pointcloud_topic / TF)')

        elif self.state == 'SEARCH':
            pts = np.concatenate(self.buf, axis=0)
            self.buf = []
            t = time.time()
            r = self.loc.localize(pts)
            r['search_time'] = round(time.time() - t, 3)
            raw_candidates = r.pop('candidates', None)
            self.result = r
            candidate_pose = None
            if 'x' in r:
                candidate_pose, T_output_base = self._make_output_pose(r)
                self.result.update(output_x=candidate_pose.pose.pose.position.x,
                                   output_y=candidate_pose.pose.pose.position.y,
                                   output_z=candidate_pose.pose.pose.position.z)
                self._publish_candidate_debug(r, pts, candidate_pose, T_output_base)
            if self.use_autoware_ndt:
                min_seed = float(g('ndt_min_seed_score'))
                if 'x' not in r or r.get('score', 0.0) < min_seed:
                    self._retry_or_fail(
                        f"2D seed score {r.get('score', 0.0):.3f} < "
                        f"ndt_min_seed_score {min_seed:.3f}")
                    return
                if not self.ndt_align_cli.service_is_ready():
                    self._retry_or_fail(
                        '/localization/pose_estimator/ndt_align_srv not available')
                    return
                if not self.init_cli.service_is_ready():
                    self._retry_or_fail('/api/localization/initialize not available')
                    return
                self.ndt_candidates = self._build_ndt_candidates(r, raw_candidates)
                if not self.ndt_candidates:
                    self._retry_or_fail('no distinct candidates for NDT validation')
                    return
                self.ndt_results = []
                self.ndt_index = 0
                self.state = 'NDT_VALIDATE'
                self._call_next_ndt_candidate()
            else:
                if not r['accepted']:
                    self._retry_or_fail(r['reason'])
                    return
                self.pose_pub.publish(candidate_pose)
                self._goto('DONE', f"standalone pose published on {g('pose_topic')}")

        elif self.state == 'NDT_VALIDATE':
            if self.future.done():
                candidate = self.ndt_candidates[self.ndt_index]
                try:
                    response = self.future.result()
                except Exception as exc:  # noqa
                    response = None
                    self.get_logger().warn(
                        f'NDT candidate {self.ndt_index + 1} exception: {exc}')
                item = {
                    'seed_index': self.ndt_index,
                    'success': bool(response is not None and response.success),
                    'reliable': bool(response is not None and response.reliable),
                }
                if item['success']:
                    pose = response.pose_with_covariance
                    q = pose.pose.pose.orientation
                    pos = pose.pose.pose.position
                    item.update(
                        pose=pose, x=float(pos.x), y=float(pos.y), z=float(pos.z),
                        yaw=_yaw_from_quat(q),
                        yaw_deg=math.degrees(_yaw_from_quat(q)))
                self.ndt_results.append(item)
                self.get_logger().info(
                    f"NDT candidate {self.ndt_index + 1}: "
                    f"success={item['success']} reliable={item['reliable']}")
                self.ndt_index += 1
                self._call_next_ndt_candidate()
            elif now - self.t_state > g('autoware_init_timeout_sec'):
                self.ndt_results.append({
                    'seed_index': self.ndt_index,
                    'success': False, 'reliable': False})
                self.get_logger().warn(
                    f'NDT candidate {self.ndt_index + 1} timed out')
                self.ndt_index += 1
                self._call_next_ndt_candidate()

        elif self.state == 'CALL_INIT':
            if self.future.done():
                try:
                    response = self.future.result()
                except Exception as exc:  # noqa
                    self._retry_or_fail(f'Autoware initialize exception: {exc}')
                    return
                if response is not None and response.status.success:
                    self._goto('WAIT_INIT', 'Autoware accepted pose; waiting for NDT')
                else:
                    message = response.status.message if response is not None else 'no response'
                    self._retry_or_fail(f'Autoware initialize failed: {message}')
            elif now - self.t_state > g('autoware_init_timeout_sec'):
                self._retry_or_fail('Autoware initialize service timeout')

        elif self.state == 'WAIT_INIT':
            if self.init_state == LocalizationInitializationState.INITIALIZED:
                refined = None
                if self.latest_kinematic is not None:
                    refined = PoseWithCovarianceStamped()
                    refined.header = self.latest_kinematic.header
                    refined.pose = self.latest_kinematic.pose
                    self.pose_pub.publish(refined)
                    pos = refined.pose.pose.position
                    self.result.update(ndt_x=pos.x, ndt_y=pos.y, ndt_z=pos.z)
                self.result.update(
                    accepted=True,
                    reason='Autoware NDT validation and initialization succeeded')
                self._publish_final_marker('NDT INITIALIZED', True, refined)
                self._goto('DONE', 'Autoware NDT localization INITIALIZED')
            elif now - self.t_state > g('autoware_init_timeout_sec'):
                self._retry_or_fail('Autoware NDT did not become INITIALIZED')

    def _retry_or_fail(self, reason):
        self.ndt_candidates = []
        self.ndt_results = []
        self.ndt_index = 0
        self.attempt += 1
        if self.attempt < self.get_parameter('max_attempts').value:
            self.buf = []
            self._goto('WAIT_STOP', f'retry after: {reason}')
        else:
            self._fail(reason)

    def _fail(self, reason):
        self._publish_final_marker(f'NDT REJECTED: {reason}', False)
        self._goto('FAILED', f'{reason} -> no pose published; move to a less ambiguous view and retry')


def main():
    rclpy.init()
    node = RoomGlobalLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
