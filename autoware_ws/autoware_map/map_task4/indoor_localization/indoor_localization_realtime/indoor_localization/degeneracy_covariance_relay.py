"""Sits between NDT and the EKF. Inside configured corridor zones, raises the
longitudinal variance of the NDT pose so the EKF follows odometry along the
corridor while NDT keeps constraining lateral position and yaw."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32

from .zones import inflate_covariance, zones_from_params


class DegeneracyCovarianceRelay(Node):
    def __init__(self):
        super().__init__('degeneracy_covariance_relay', automatically_declare_parameters_from_overrides=True)
        g = self._get
        self.enabled = bool(g('enabled', True))
        names = list(g('zone_names', []) or [])
        names = [n for n in names if n]
        self.zones = zones_from_params(names, lambda k: self.get_parameter(k).value)
        for z in self.zones:
            self.get_logger().info(f'zone {z.name}: {z.start} -> {z.end}, half_width {z.half_width}, '
                                   f'sigma_long {z.sigma_long}, ramp {z.ramp}')
        self.pub = self.create_publisher(PoseWithCovarianceStamped,
                                         g('output_topic', '/localization/pose_estimator/pose_with_covariance'), 10)
        self.pub_w = self.create_publisher(Float32, '~/debug/zone_weight', 10)
        self.pub_s = self.create_publisher(Float32, '~/debug/sigma_long', 10)
        self.create_subscription(PoseWithCovarianceStamped,
                                 g('input_topic', '/localization/pose_estimator/ndt_pose_with_covariance_raw'),
                                 self.on_pose, 10)

    def _get(self, name, default):
        if self.has_parameter(name):
            return self.get_parameter(name).value
        return default

    def on_pose(self, msg):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        best_w, best = 0.0, None
        for z in self.zones:
            w, yaw = z.weight_and_axis(x, y)
            if w > best_w:
                best_w, best = w, (z, yaw)
        sigma = None
        if self.enabled and best is not None:
            cov, sigma = inflate_covariance(list(msg.pose.covariance), best[1], best[0].sigma_long, best_w)
            msg.pose.covariance = cov
        self.pub.publish(msg)
        self.pub_w.publish(Float32(data=float(best_w)))
        if sigma is not None:
            self.pub_s.publish(Float32(data=float(sigma)))


def main():
    rclpy.init()
    node = DegeneracyCovarianceRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
