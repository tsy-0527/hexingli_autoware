"""VelocityReport (chassis 0x51) -> TwistWithCovarianceStamped for gyro_odometer.

`speed_scale_factor` is where the wheel-speed calibration goes (most important
parameter for the corridor stretch where NDT has no longitudinal information).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped

try:
    from autoware_vehicle_msgs.msg import VelocityReport
except ImportError:
    from autoware_auto_vehicle_msgs.msg import VelocityReport


class VelocityReportConverter(Node):
    def __init__(self):
        super().__init__('velocity_report_converter')
        d = self.declare_parameter
        d('input_topic', '/vehicle/status/velocity_status')
        d('output_topic', '/sensing/vehicle_velocity_converter/twist_with_covariance')
        d('frame_id', 'base_link')
        d('speed_scale_factor', 1.0)
        d('stddev_vx', 0.04)        # m/s
        d('stddev_wz', 0.1)         # rad/s (gyro_odometer replaces wz with the IMU anyway)
        g = lambda k: self.get_parameter(k).value  # noqa
        self.k, self.svx, self.swz, self.frame = g('speed_scale_factor'), g('stddev_vx'), g('stddev_wz'), g('frame_id')
        self.pub = self.create_publisher(TwistWithCovarianceStamped, g('output_topic'), 10)
        self.create_subscription(VelocityReport, g('input_topic'), self.on_msg, 10)

    def on_msg(self, m):
        o = TwistWithCovarianceStamped()
        o.header.stamp = m.header.stamp
        o.header.frame_id = self.frame
        o.twist.twist.linear.x = self.k * m.longitudinal_velocity
        o.twist.twist.linear.y = m.lateral_velocity
        o.twist.twist.angular.z = m.heading_rate
        c = [0.0] * 36
        c[0] = self.svx ** 2
        c[7] = c[14] = 10000.0
        c[21] = c[28] = 10000.0
        c[35] = self.swz ** 2
        o.twist.covariance = c
        self.pub.publish(o)


def main():
    rclpy.init()
    node = VelocityReportConverter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
