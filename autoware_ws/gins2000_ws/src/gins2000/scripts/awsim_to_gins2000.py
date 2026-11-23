#!/usr/bin/env python3
"""
awsim_to_gins2000.py
读取 AWSIM 仿真话题，打包成 Gins2000 消息，模拟真实硬件输出
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
from gins2000.msg import Gins2000


class AwsimToGins2000(Node):
    def __init__(self):
        super().__init__('awsim_to_gins2000')

        self.latest_imu  = None
        self.latest_fix  = None
        self.latest_pose = None

        # 订阅 AWSIM 话题
        self.create_subscription(
            Imu, '/sensing/imu/imu_raw', self._imu_cb, 10)
        self.create_subscription(
            NavSatFix, '/sensing/gnss/fix', self._fix_cb, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/sensing/gnss/pose_with_covariance', self._pose_cb, 10)

        # 发布 Gins2000
        self.pub = self.create_publisher(Gins2000, '/gins2000_sim', 10)

        self.create_timer(0.1, self._publish)  # 10Hz
        self.get_logger().info('awsim_to_gins2000 已启动')

    def _imu_cb(self, msg):
        self.latest_imu = msg

    def _fix_cb(self, msg):
        self.latest_fix = msg

    def _pose_cb(self, msg):
        self.latest_pose = msg

    def _publish(self):
        if not self.latest_imu or not self.latest_fix:
            return

        out = Gins2000()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = 'gnss_link'

        # IMU 数据
        out.angular_velocity.x = self.latest_imu.angular_velocity.x
        out.angular_velocity.y = self.latest_imu.angular_velocity.y
        out.angular_velocity.z = self.latest_imu.angular_velocity.z
        out.linear_acceleration.x = self.latest_imu.linear_acceleration.x
        out.linear_acceleration.y = self.latest_imu.linear_acceleration.y
        out.linear_acceleration.z = self.latest_imu.linear_acceleration.z

        # 从四元数提取欧拉角（度）
        if self.latest_pose:
            q = self.latest_pose.pose.pose.orientation
            roll, pitch, yaw = self._quat_to_euler(q.x, q.y, q.z, q.w)
            out.attitude.x = math.degrees(roll)
            out.attitude.y = math.degrees(pitch)
            out.attitude.z = math.degrees(yaw)

        # GNSS 数据
        out.latitude  = self.latest_fix.latitude
        out.longitude = self.latest_fix.longitude
        out.altitude  = self.latest_fix.altitude

        out.ins_status = 4  # 组合导航正常
        self.pub.publish(out)

    def _quat_to_euler(self, x, y, z, w):
        # roll
        sinr = 2.0 * (w*x + y*z)
        cosr = 1.0 - 2.0 * (x*x + y*y)
        roll = math.atan2(sinr, cosr)
        # pitch
        sinp = 2.0 * (w*y - z*x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        # yaw
        siny = 2.0 * (w*z + x*y)
        cosy = 1.0 - 2.0 * (y*y + z*z)
        yaw = math.atan2(siny, cosy)
        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    node = AwsimToGins2000()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()