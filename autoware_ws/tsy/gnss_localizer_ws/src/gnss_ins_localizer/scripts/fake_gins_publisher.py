#!/usr/bin/env python3
"""Fake GINS2000 publisher for loopback testing with planning_simulator."""
import math
import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3

from pyproj import Transformer
from gins2000.msg import Gins2000


class FakeGinsPublisher(Node):
    def __init__(self):
        super().__init__('fake_gins_publisher')

        self.declare_parameter('map_path', '')
        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('ins_status', 15)
        self.declare_parameter('position_noise_std_m', 0.0)
        self.declare_parameter('yaw_noise_std_deg', 0.0)

        map_path = self.get_parameter('map_path').value
        self.ins_status = int(self.get_parameter('ins_status').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.pos_noise = float(self.get_parameter('position_noise_std_m').value)
        self.yaw_noise = float(self.get_parameter('yaw_noise_std_deg').value)

        origin_yaml = os.path.join(map_path, 'map_projector_info.yaml')
        with open(origin_yaml, 'r') as f:
            info = yaml.safe_load(f)
        self.origin_lat = float(info['map_origin']['latitude'])
        self.origin_lon = float(info['map_origin']['longitude'])
        self.get_logger().info(
            f'Origin lat={self.origin_lat:.9f} lon={self.origin_lon:.9f}')

        utm_zone = int((self.origin_lon + 180) / 6) + 1
        northp = self.origin_lat >= 0
        epsg_utm = 32600 + utm_zone if northp else 32700 + utm_zone
        self.to_utm = Transformer.from_crs(4326, epsg_utm, always_xy=True)
        self.to_wgs = Transformer.from_crs(epsg_utm, 4326, always_xy=True)

        self.origin_utm_x, self.origin_utm_y = self.to_utm.transform(
            self.origin_lon, self.origin_lat)
        self.get_logger().info(
            f'UTM zone {utm_zone} origin_utm=({self.origin_utm_x:.3f},{self.origin_utm_y:.3f})')

        self.latest_odom = None

        qos_sensor = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST)
        qos_reliable = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST)

        self.sub = self.create_subscription(
            Odometry, '/localization/kinematic_state',
            self.on_odom, 1)
        self.pub = self.create_publisher(Gins2000, '/sensing/gins2000', qos_sensor)
        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f'fake_gins_publisher up. Publishing at {rate} Hz, ins_status={self.ins_status}')

    def on_odom(self, msg: Odometry):
        self.latest_odom = msg

    @staticmethod
    def quat_to_yaw_enu(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def tick(self):
        self.ins_status = int(self.get_parameter("ins_status").value)
        if self.latest_odom is None:
            self.get_logger().warn(
                'Waiting for /localization/kinematic_state ...',
                throttle_duration_sec=2.0)
            return

        odom = self.latest_odom
        map_x = odom.pose.pose.position.x
        map_y = odom.pose.pose.position.y

        if self.pos_noise > 0:
            import random
            map_x += random.gauss(0, self.pos_noise)
            map_y += random.gauss(0, self.pos_noise)

        utm_x = map_x + self.origin_utm_x
        utm_y = map_y + self.origin_utm_y
        lon, lat = self.to_wgs.transform(utm_x, utm_y)

        yaw_enu = self.quat_to_yaw_enu(odom.pose.pose.orientation)
        yaw_gins_rad = yaw_enu - math.pi / 2.0
        while yaw_gins_rad >  math.pi: yaw_gins_rad -= 2 * math.pi
        while yaw_gins_rad < -math.pi: yaw_gins_rad += 2 * math.pi

        if self.yaw_noise > 0:
            import random
            yaw_gins_rad += math.radians(random.gauss(0, self.yaw_noise))

        yaw_gins_deg = math.degrees(yaw_gins_rad)

        vx_body = odom.twist.twist.linear.x
        vy_body = odom.twist.twist.linear.y
        vz_body = odom.twist.twist.linear.z
        cy = math.cos(yaw_enu); sy = math.sin(yaw_enu)
        v_east  = cy * vx_body - sy * vy_body
        v_north = sy * vx_body + cy * vy_body
        v_up    = vz_body

        msg = Gins2000()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gnss_ins_link'
        msg.frame_count = 0
        msg.attitude = Vector3(x=0.0, y=0.0, z=yaw_gins_deg)
        wz_flu = odom.twist.twist.angular.z
        msg.angular_velocity = Vector3(x=0.0, y=0.0, z=wz_flu)
        msg.linear_acceleration = Vector3(x=0.0, y=0.0, z=9.81)
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = odom.pose.pose.position.z
        msg.velocity = Vector3(x=float(v_east), y=float(v_north), z=float(v_up))
        msg.ins_status = self.ins_status
        msg.week = 2433
        msg.gps_millisec = 0
        msg.temperature = 40.0

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeGinsPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
