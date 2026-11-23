"""MID-360 preprocessing: base_link transform -> deskew -> NDT and obstacle clouds.

in : PointCloud2 from the MID-360 driver (any frame with TF to base_link,
     per-point time field, e.g. livox_ros_driver2 'timestamp' in absolute ns)
     TwistWithCovarianceStamped (vehicle twist, for deskew)
out: NDT cloud and a low-obstacle cloud for Nav2, both in base_link.
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import tf2_ros

from .cloud_ops import crop, deskew_planar, point_times_to_sec, quat_to_mat, transform, voxel_downsample
from .pc2 import make_xyzi, read_fields


def _stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def _sec_to_stamp(t):
    from builtin_interfaces.msg import Time
    sec = int(t)
    return Time(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1000000000)


class Mid360Preprocessor(Node):
    def __init__(self):
        super().__init__('mid360_preprocessor')
        d = self.declare_parameter
        d('input_topic', '/sensing/lidar/mid360s/pointcloud')
        d('output_topic', '/localization/util/downsample/pointcloud')
        d('twist_topic', '/localization/twist_estimator/twist_with_covariance')
        d('base_frame', 'base_link')
        d('time_field', 'timestamp')
        d('enable_deskew', True)
        d('max_twist_age_sec', 0.3)
        d('r_min', 0.4)
        d('r_max', 40.0)
        d('z_min', 0.15)            # base_link z = height above floor; map has almost no floor points
        d('z_max', 4.6)
        d('self_box', [-0.35, 0.85, -0.45, 0.45, -0.2, 1.2])
        d('voxel_leaf', 0.1)
        d('min_output_points', 300)
        d('obstacle_enabled', True)
        d('obstacle_output_topic', '/indoor_perception/mid360/obstacle_pointcloud')
        d('obstacle_r_min', 0.2)
        d('obstacle_r_max', 10.0)
        d('obstacle_z_min', 0.05)
        d('obstacle_z_max', 0.60)
        d('obstacle_self_box', [-0.22, 0.70, -0.32, 0.32, -0.2, 0.75])
        d('obstacle_voxel_leaf', 0.05)
        self.p = {k: self.get_parameter(k).value for k in [
            'base_frame', 'time_field', 'enable_deskew', 'max_twist_age_sec', 'r_min', 'r_max',
            'z_min', 'z_max', 'self_box', 'voxel_leaf', 'min_output_points',
            'obstacle_enabled', 'obstacle_r_min', 'obstacle_r_max', 'obstacle_z_min',
            'obstacle_z_max', 'obstacle_self_box', 'obstacle_voxel_leaf']}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.extrinsic = {}                      # frame_id -> (R, t), static
        self.twist = None                        # (stamp_sec, vx, wz)
        self.pub = self.create_publisher(PointCloud2, self.get_parameter('output_topic').value, 10)
        self.pub_obs = (self.create_publisher(
            PointCloud2, self.get_parameter('obstacle_output_topic').value, 10)
            if self.p['obstacle_enabled'] else None)
        self.create_subscription(PointCloud2, self.get_parameter('input_topic').value, self.on_cloud,
                                 qos_profile_sensor_data)
        self.create_subscription(TwistWithCovarianceStamped, self.get_parameter('twist_topic').value,
                                 self.on_twist, 10)
        self.n = 0
        self.t_proc = 0.0
        self.create_timer(5.0, self.report)

    def on_twist(self, msg):
        self.twist = (_stamp_sec(msg.header.stamp), msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def get_extrinsic(self, frame):
        if frame not in self.extrinsic:
            tf = self.tf_buffer.lookup_transform(self.p['base_frame'], frame, rclpy.time.Time())
            q, tr = tf.transform.rotation, tf.transform.translation
            self.extrinsic[frame] = (quat_to_mat(q.x, q.y, q.z, q.w), np.array([tr.x, tr.y, tr.z]))
            self.get_logger().info(f'extrinsic {self.p["base_frame"]} <- {frame}: t={self.extrinsic[frame][1]}')
        return self.extrinsic[frame]

    def on_cloud(self, msg):
        t0 = time.time()
        try:
            R, t = self.get_extrinsic(msg.header.frame_id)
        except Exception as e:  # noqa
            self.get_logger().warn(f'waiting for TF {self.p["base_frame"]} <- {msg.header.frame_id}: {e}',
                                   throttle_duration_sec=2.0)
            return
        f = read_fields(msg, ['x', 'y', 'z', 'intensity', self.p['time_field']])
        if 'x' not in f or len(f['x']) == 0:
            return
        hdr_t = _stamp_sec(msg.header.stamp)
        pts = transform(np.column_stack([f['x'], f['y'], f['z']]), R, t)
        inten = f.get('intensity')
        has_time = self.p['time_field'] in f
        tpts = point_times_to_sec(f[self.p['time_field']], hdr_t) if has_time else None

        p = self.p
        r_lo = min(p['r_min'], p['obstacle_r_min']) if p['obstacle_enabled'] else p['r_min']
        r_hi = max(p['r_max'], p['obstacle_r_max']) if p['obstacle_enabled'] else p['r_max']
        pts, m = crop(pts, r_min=r_lo, r_max=r_hi, z_min=-1e9, z_max=1e9,
                      self_box=[0, 0, 0, 0, 0, 0])
        if inten is not None:
            inten = inten[m]
        t_ref = hdr_t
        if has_time:
            tpts = tpts[m]
            if len(tpts):
                t_ref = float(np.max(tpts))
            if p['enable_deskew'] and self.twist is not None and len(pts):
                age = abs(t_ref - self.twist[0])
                if age < p['max_twist_age_sec']:
                    pts = deskew_planar(pts, tpts, t_ref, self.twist[1], self.twist[2])
                else:
                    self.get_logger().warn(f'twist too old ({age:.2f}s), skip deskew', throttle_duration_sec=2.0)
        header = Header(frame_id=self.p['base_frame'], stamp=_sec_to_stamp(t_ref))

        ndt, ndt_mask = crop(pts, r_min=p['r_min'], r_max=p['r_max'], z_min=p['z_min'],
                             z_max=p['z_max'], self_box=p['self_box'])
        ndt_intensity = inten[ndt_mask] if inten is not None else None
        ndt, ndt_intensity = voxel_downsample(ndt, p['voxel_leaf'], ndt_intensity)
        if len(ndt) < p['min_output_points']:
            self.get_logger().warn(
                f'only {len(ndt)} NDT points after preprocessing', throttle_duration_sec=2.0)
        else:
            self.pub.publish(make_xyzi(header, ndt, ndt_intensity))

        if self.pub_obs is not None:
            obs, obs_mask = crop(
                pts, r_min=p['obstacle_r_min'], r_max=p['obstacle_r_max'],
                z_min=p['obstacle_z_min'], z_max=p['obstacle_z_max'],
                self_box=p['obstacle_self_box'])
            obs_intensity = inten[obs_mask] if inten is not None else None
            obs, obs_intensity = voxel_downsample(
                obs, p['obstacle_voxel_leaf'], obs_intensity)
            self.pub_obs.publish(make_xyzi(header, obs, obs_intensity))
        self.n += 1
        self.t_proc += time.time() - t0

    def report(self):
        if self.n:
            self.get_logger().info(f'{self.n / 5.0:.1f} Hz, {1000 * self.t_proc / self.n:.1f} ms/frame')
        self.n, self.t_proc = 0, 0.0


def main():
    rclpy.init()
    node = Mid360Preprocessor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
