#!/usr/bin/env python3
"""
Odom → Eagleye twist 桥接
从底盘 /odom 提取车速，转成 TwistWithCovarianceStamped 供 Eagleye 使用
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped


class OdomToTwist(Node):
    def __init__(self):
        super().__init__('odom_to_twist')

        self.declare_parameter('input_topic', '/odom')
        self.declare_parameter('output_topic', '/vehicle/twist_with_covariance')

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)

        self.sub = self.create_subscription(Odometry, in_topic, self.cb, qos)
        self.pub = self.create_publisher(TwistWithCovarianceStamped, out_topic, qos)

        self._msg_count = 0
        self.create_timer(2.0, self.print_stats)

        self.get_logger().info(f'[Init] {in_topic} → {out_topic}')

    def cb(self, msg: Odometry):
        self._msg_count += 1
        out = TwistWithCovarianceStamped()
        out.header = msg.header
        out.header.frame_id = 'base_link'
        # 直接复用 odom 的 twist（含协方差）
        out.twist = msg.twist
        self.pub.publish(out)

    def print_stats(self):
        if self._msg_count == 0:
            self.get_logger().warn('[Stats] ⚠️ No /odom message in last 2s')
            return
        rate = self._msg_count / 2.0
        self.get_logger().info(f'[Stats] odom→twist rate: {rate:.1f} Hz')
        self._msg_count = 0


def main():
    rclpy.init()
    node = OdomToTwist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
