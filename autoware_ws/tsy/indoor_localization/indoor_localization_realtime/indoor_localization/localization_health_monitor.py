"""Summarise localization health for the UI / Agent as one JSON message.

level: OK / WARN / ERROR, plus a suggested speed factor (1.0 / 0.5 / 0.0).
Checks: initialization state, NDT output freshness, NDT score (NVTL),
EKF lateral / longitudinal standard deviation in the vehicle frame.
Inside a degeneracy zone the longitudinal check is relaxed and the NVTL
check is skipped (low scores there are expected, lateral is what matters).
"""
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32, String
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState as InitState

Float32Stamped = None
for _mod in ('autoware_internal_debug_msgs.msg', 'tier4_debug_msgs.msg'):
    try:
        Float32Stamped = __import__(_mod, fromlist=['Float32Stamped']).Float32Stamped
        break
    except ImportError:
        pass


class LocalizationHealthMonitor(Node):
    def __init__(self):
        super().__init__('localization_health_monitor')
        d = self.declare_parameter
        d('ekf_pose_topic', '/localization/pose_with_covariance')
        d('ndt_pose_topic', '/localization/pose_estimator/pose_with_covariance')
        d('nvtl_topic', '/localization/pose_estimator/nearest_voxel_transformation_likelihood')
        d('zone_weight_topic', '/localization/pose_estimator/degeneracy_covariance_relay/debug/zone_weight')
        d('state_topic', '/localization/initialization_state')
        d('output_topic', '/indoor_localization/health')
        d('ndt_timeout_warn_sec', 1.0)
        d('ndt_timeout_error_sec', 3.0)
        d('nvtl_warn', 2.3)
        d('nvtl_error', 1.8)
        d('nvtl_bad_count', 5)          # consecutive low scores before reacting
        d('lat_sigma_warn', 0.08)       # m (corridor ~2.4 m wide)
        d('lat_sigma_error', 0.15)
        d('long_sigma_warn', 0.5)
        d('long_sigma_error', 1.0)
        d('long_sigma_scale_in_zone', 3.0)
        g = lambda k: self.get_parameter(k).value  # noqa
        self.g = g
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(String, g('output_topic'), latched)
        self.create_subscription(PoseWithCovarianceStamped, g('ekf_pose_topic'), self.on_ekf, 10)
        self.create_subscription(PoseWithCovarianceStamped, g('ndt_pose_topic'), self.on_ndt, 10)
        self.create_subscription(Float32, g('zone_weight_topic'), self.on_zone, 10)
        self.create_subscription(InitState, g('state_topic'), self.on_state, latched)
        if Float32Stamped is not None:
            self.create_subscription(Float32Stamped, g('nvtl_topic'), self.on_nvtl, 10)
        else:
            self.get_logger().warn('Float32Stamped type not found, NVTL check disabled')
        self.state = None
        self.ekf = None
        self.t_ndt = None
        self.nvtl = None
        self.low_count = 0
        self.zone_w = 0.0
        self.create_timer(0.2, self.evaluate)

    def on_state(self, m):
        self.state = m.state

    def on_zone(self, m):
        self.zone_w = m.data

    def on_ndt(self, m):
        self.t_ndt = time.time()

    def on_nvtl(self, m):
        self.nvtl = m.data
        self.low_count = self.low_count + 1 if m.data < self.g('nvtl_warn') else 0

    def on_ekf(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        C = np.array(m.pose.covariance).reshape(6, 6)[:2, :2]
        c, s = math.cos(yaw), math.sin(yaw)
        fwd, left = np.array([c, s]), np.array([-s, c])
        self.ekf = dict(x=m.pose.pose.position.x, y=m.pose.pose.position.y, yaw_deg=math.degrees(yaw),
                        sigma_long=float(math.sqrt(max(fwd @ C @ fwd, 0))),
                        sigma_lat=float(math.sqrt(max(left @ C @ left, 0))))

    def evaluate(self):
        g = self.g
        reasons, level = [], 0
        def flag(lv, why):  # noqa
            nonlocal level
            level = max(level, lv)
            reasons.append(why)
        if self.state != InitState.INITIALIZED:
            flag(2, 'not initialized')
        if self.t_ndt is None:
            flag(2, 'no NDT pose yet')
        else:
            age = time.time() - self.t_ndt
            if age > g('ndt_timeout_error_sec'):
                flag(2, f'NDT pose stale {age:.1f}s')
            elif age > g('ndt_timeout_warn_sec'):
                flag(1, f'NDT pose late {age:.1f}s')
        in_zone = self.zone_w > 0.0
        if self.nvtl is not None and not in_zone and self.low_count >= g('nvtl_bad_count'):
            flag(2 if self.nvtl < g('nvtl_error') else 1, f'NVTL low {self.nvtl:.2f}')
        if self.ekf:
            lat, lon = self.ekf['sigma_lat'], self.ekf['sigma_long']
            k = g('long_sigma_scale_in_zone') if in_zone else 1.0
            if lat > g('lat_sigma_error'):
                flag(2, f'lateral sigma {lat:.3f} m')
            elif lat > g('lat_sigma_warn'):
                flag(1, f'lateral sigma {lat:.3f} m')
            if lon > k * g('long_sigma_error'):
                flag(2, f'longitudinal sigma {lon:.2f} m')
            elif lon > k * g('long_sigma_warn'):
                flag(1, f'longitudinal sigma {lon:.2f} m')
        body = dict(level=['OK', 'WARN', 'ERROR'][level], speed_factor=[1.0, 0.5, 0.0][level],
                    reasons=reasons, in_degeneracy_zone=in_zone, zone_weight=round(self.zone_w, 2),
                    nvtl=None if self.nvtl is None else round(self.nvtl, 3), ekf=self.ekf)
        self.pub.publish(String(data=json.dumps(body, ensure_ascii=False)))


def main():
    rclpy.init()
    node = LocalizationHealthMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
