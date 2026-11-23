#!/usr/bin/env python3
"""
converter_node.py
将 /gins2000 (Gins2000) 转换为三个标准 ROS2 话题：

  /sensing/imu/imu_raw          sensor_msgs/Imu
      → 给 ekf_localizer / gyro_odometer / LIO-SAM 用

  /sensing/gnss/fix              sensor_msgs/NavSatFix
      → 给 gnss_poser / LIO-SAM 用

  /sensing/gnss/ins_pose_with_covariance
      autoware_sensing_msgs/GnssInsOrientationStamped
      → 给 Autoware gnss_poser 提供 GINS2000 航向角
"""

import math
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# 你的自定义消息（包名根据实际修改）
from gins2000.msg import Gins2000
from autoware_sensing_msgs.msg import GnssInsOrientationStamped

# 标准消息
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from geometry_msgs.msg import (
    Quaternion,
)
from std_msgs.msg import Header


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """
    欧拉角 (RPY，弧度) → 四元数
    Gins2000.attitude: x=roll, y=pitch, z=yaw
    """
    cr = math.cos(roll  / 2.0)
    sr = math.sin(roll  / 2.0)
    cp = math.cos(pitch / 2.0)
    sp = math.sin(pitch / 2.0)
    cy = math.cos(yaw   / 2.0)
    sy = math.sin(yaw   / 2.0)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class Gins2000Converter(Node):
    def __init__(self):
        super().__init__('gins2000_converter')

        # ── 参数 ─────────────────────────────────────────────
        self.declare_parameter('input_topic',  '/sensing/gins2000')
        self.declare_parameter('imu_frame',    'gnss_link')
        self.declare_parameter('gnss_frame',   'gnss_link')

        # IMU 噪声参数（根据 GINS2000 数据手册填写）
        # 下面是保守默认值，可在参数文件中覆盖
        self.declare_parameter('gyro_noise',   0.01)   # rad/s
        self.declare_parameter('accel_noise',  0.1)    # m/s^2

        input_topic = self.get_parameter('input_topic').value
        self.imu_frame  = self.get_parameter('imu_frame').value
        self.gnss_frame = self.get_parameter('gnss_frame').value
        self.gyro_noise  = self.get_parameter('gyro_noise').value
        self.accel_noise = self.get_parameter('accel_noise').value

        # ── 订阅 ─────────────────────────────────────────────
        self.sub = self.create_subscription(
            Gins2000,
            input_topic,
            self._callback,
            10
        )

        # ── 发布 ─────────────────────────────────────────────
        self.pub_imu = self.create_publisher(
            Imu, '/sensing/imu/imu_raw', 10)

        self.pub_fix = self.create_publisher(
            NavSatFix, '/sensing/gnss/fix', 10)

        self.pub_pose = self.create_publisher(
            GnssInsOrientationStamped,
            '/sensing/gnss/ins_pose_with_covariance', 10)

        self.get_logger().info(
            f'Gins2000Converter 已启动，订阅: {input_topic}')

    # ─────────────────────────────────────────────────────────
    def _callback(self, msg: Gins2000):
        stamp = msg.header.stamp

        # 1. 发布 IMU ─────────────────────────────────────────
        imu = Imu()
        imu.header.stamp    = stamp
        imu.header.frame_id = self.imu_frame

        # 姿态四元数（由欧拉角转换）
        # msg.attitude: x=roll, y=pitch, z=yaw
        imu.orientation = euler_to_quaternion(
            msg.attitude.x,   # roll
            msg.attitude.y,   # pitch
            msg.attitude.z    # yaw
        )
        # 姿态协方差（对角阵，单位 rad^2）
        imu.orientation_covariance = [
            0.001, 0.0,   0.0,
            0.0,   0.001, 0.0,
            0.0,   0.0,   0.01,   # yaw 误差略大
        ]

        # 角速度
        imu.angular_velocity.x = msg.angular_velocity.x
        imu.angular_velocity.y = msg.angular_velocity.y
        imu.angular_velocity.z = msg.angular_velocity.z
        n = self.gyro_noise ** 2
        imu.angular_velocity_covariance = [
            n,   0.0, 0.0,
            0.0, n,   0.0,
            0.0, 0.0, n,
        ]

        # 线加速度
        imu.linear_acceleration.x = msg.linear_acceleration.x
        imu.linear_acceleration.y = msg.linear_acceleration.y
        imu.linear_acceleration.z = msg.linear_acceleration.z
        a = self.accel_noise ** 2
        imu.linear_acceleration_covariance = [
            a,   0.0, 0.0,
            0.0, a,   0.0,
            0.0, 0.0, a,
        ]

        self.pub_imu.publish(imu)

        # 2. 发布 NavSatFix ────────────────────────────────────
        fix = NavSatFix()
        fix.header.stamp    = stamp
        fix.header.frame_id = self.gnss_frame

        fix.status.status  = (
            NavSatStatus.STATUS_FIX
            if msg.ins_status > 0
            else NavSatStatus.STATUS_NO_FIX
        )
        fix.status.service = NavSatStatus.SERVICE_GPS

        fix.latitude  = msg.latitude
        fix.longitude = msg.longitude
        fix.altitude  = msg.altitude

        # 位置协方差（单位 m^2，根据 GINS2000 精度指标填写）
        # 典型值：水平 0.1m，垂直 0.2m → 方差 0.01 / 0.04
        pos_cov = 0.01
        alt_cov = 0.04
        fix.position_covariance = [
            pos_cov, 0.0,     0.0,
            0.0,     pos_cov, 0.0,
            0.0,     0.0,     alt_cov,
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.pub_fix.publish(fix)

        # 3. 发布 Autoware GNSS-INS 航向 ─────────────────────
        # gnss_poser 订阅的类型必须是
        # autoware_sensing_msgs/msg/GnssInsOrientationStamped。
        # 该消息只传递 INS 航向和姿态均方根误差，位置由 gnss_poser
        # 根据 NavSatFix 和 map_projector_info 转换到 map 坐标。
        orientation_msg = GnssInsOrientationStamped()
        orientation_msg.header.stamp    = stamp
        orientation_msg.header.frame_id = self.gnss_frame
        orientation_msg.orientation.orientation = euler_to_quaternion(
            msg.attitude.x,
            msg.attitude.y,
            msg.attitude.z
        )
        # 与 IMU 姿态协方差对应的 RMS 误差，单位为 rad。
        orientation_msg.orientation.rmse_rotation_x = math.sqrt(0.001)
        orientation_msg.orientation.rmse_rotation_y = math.sqrt(0.001)
        orientation_msg.orientation.rmse_rotation_z = math.sqrt(0.01)
        self.pub_pose.publish(orientation_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Gins2000Converter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
