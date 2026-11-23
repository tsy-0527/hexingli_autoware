#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from autoware_control_msgs.msg import Control
import math

class HunterAutowareBridge(Node):
    def __init__(self):
        super().__init__('hunter_autoware_bridge')
        self.declare_parameter('wheelbase', 0.648)
        self.declare_parameter('max_steering_angle', 0.52)
        self.declare_parameter('max_linear_speed', 1.5)

        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steer = self.get_parameter('max_steering_angle').value
        self.max_speed = self.get_parameter('max_linear_speed').value

        self.sub = self.create_subscription(
            Control,
            '/control/command/control_cmd',
            self.control_callback,
            10)
        self.pub = self.create_publisher(Twist, '/hunter/cmd_vel', 10)
        self.get_logger().info(f'Bridge started | wheelbase={self.wheelbase}m | max_speed={self.max_speed}m/s')

    def control_callback(self, msg):
        linear_x = float(msg.longitudinal.velocity)
        steer = float(msg.lateral.steering_tire_angle)

        linear_x = max(-self.max_speed, min(self.max_speed, linear_x))
        steer = max(-self.max_steer, min(self.max_steer, steer))

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = linear_x * math.tan(steer) / self.wheelbase

        self.pub.publish(twist)
        self.get_logger().info(
            f'linear.x={linear_x:.3f}  angular.z={twist.angular.z:.3f}  steer={math.degrees(steer):.1f}deg')

def main(args=None):
    rclpy.init(args=args)
    node = HunterAutowareBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
