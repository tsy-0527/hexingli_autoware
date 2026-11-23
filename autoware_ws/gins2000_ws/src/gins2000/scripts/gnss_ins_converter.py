#!/usr/bin/env python3
"""
GnssIns → Eagleye 标准输入 转换节点（Autoware 主线适配版）

关键处理：
- IMU 100Hz 发布 /sensing/imu/imu_data
- GNSS 降频到 10Hz 发布 /sensing/gnss/ublox/nav_sat_fix + navpvt
- 车速由 Autoware 官方 vehicle_velocity_converter 从 /vehicle/status/velocity_status 转换，
  本节点不再发 twist（避免和 Autoware 官方节点重复发布导致 200Hz 冲突）
- 坐标系: RFU + 北偏西为正 (度) → FLU + ENU (弧度)
"""
import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from ublox_msgs.msg import NavPVT
from gins2000.msg import Gins2000


# ---------- INS 状态位标志 ----------
POS_ALIGNED = 0x01
VEL_ALIGNED = 0x02
ATT_ALIGNED = 0x04
YAW_ALIGNED = 0x08
ALL_ALIGNED = 0x0F


def normalize_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def quaternion_from_euler(roll, pitch, yaw):
    """ZYX 顺序，返回 (x, y, z, w)"""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return (x, y, z, w)


def euler_from_quaternion(q):
    """输入 (x, y, z, w)，返回 (roll, pitch, yaw)"""
    x, y, z, w = q
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (roll, pitch, yaw)


class GnssInsConverter(Node):
    def __init__(self):
        super().__init__('gnss_ins_converter')

        # ---------- 参数 ----------
        self.declare_parameter('input_topic', '/sensing/gins2000')
        self.declare_parameter('force_aligned', False)
        self.declare_parameter('gnss_pub_divisor', 10)

        input_topic = self.get_parameter('input_topic').value
        self.force_aligned = self.get_parameter('force_aligned').value
        self.gnss_pub_divisor = int(self.get_parameter('gnss_pub_divisor').value)

        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)

        # 订阅
        self.sub = self.create_subscription(Gins2000, input_topic, self.cb, qos)

        # 发布（Autoware 主线适配，不再发 twist，由 Autoware 官方节点从 velocity_status 转换）
        self.pub_imu = self.create_publisher(Imu, '/sensing/imu/imu_data', qos)
        self.pub_fix = self.create_publisher(NavSatFix, '/sensing/gnss/ublox/nav_sat_fix', qos)
        self.pub_pvt = self.create_publisher(NavPVT, '/sensing/gnss/ublox/navpvt', qos)

        # 状态跟踪
        self._last_status = -1
        self._msg_count = 0
        self._gnss_pub_counter = 0

        # 定时打印统计
        self.create_timer(2.0, self.print_stats)

        self.get_logger().info(f'[Init] Subscribing: {input_topic}')
        self.get_logger().info(
            '[Init] Publishing:\n'
            '  /sensing/imu/imu_data              (Imu, ~100Hz)\n'
            '  /sensing/gnss/ublox/nav_sat_fix    (NavSatFix, ~10Hz)\n'
            '  /sensing/gnss/ublox/navpvt         (NavPVT, ~10Hz)'
        )
        self.get_logger().info(
            '[Init] Twist not published — provided by Autoware '
            'vehicle_velocity_converter from /vehicle/status/velocity_status'
        )
        self.get_logger().info(f'[Init] GNSS pub divisor: {self.gnss_pub_divisor}')
        if self.force_aligned:
            self.get_logger().warn(
                '[Init] ⚠️ force_aligned=TRUE — INDOOR DEBUG MODE. '
                'Turn OFF for outdoor real test!'
            )

    def cb(self, msg: Gins2000):
        self._msg_count += 1

        # ---------- 状态位解析 ----------
        s_raw = msg.ins_status
        s = ALL_ALIGNED if self.force_aligned else s_raw
        pos_ok = bool(s & POS_ALIGNED)
        vel_ok = bool(s & VEL_ALIGNED)
        att_ok = bool(s & ATT_ALIGNED)
        yaw_ok = bool(s & YAW_ALIGNED)

        if s != self._last_status:
            tag = ' (FORCED)' if self.force_aligned else ''
            self.get_logger().info(
                f'[Status change] ins_status=0x{s:02X}{tag}  raw=0x{s_raw:02X}  '
                f'pos={pos_ok} vel={vel_ok} att={att_ok} yaw={yaw_ok}'
            )
            if s == ALL_ALIGNED and not self.force_aligned:
                self.get_logger().info('[Status] ✅ FULLY ALIGNED - Eagleye ready to use')
            self._last_status = s

        # ---------- 1. IMU (~100Hz) ----------
        imu = Imu()
        imu.header = msg.header
        imu.header.frame_id = 'imu_link'

        # 姿态: RFU + 北偏西为正 (度) → FLU + ENU (弧度)
        roll_ros = math.radians(msg.attitude.x)
        pitch_ros = -math.radians(msg.attitude.y)
        yaw_ros = normalize_angle(math.radians(msg.attitude.z) + math.pi / 2)
        qx, qy, qz, qw = quaternion_from_euler(roll_ros, pitch_ros, yaw_ros)
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw

        # 角速度 RFU→FLU
        imu.angular_velocity.x = msg.angular_velocity.y
        imu.angular_velocity.y = -msg.angular_velocity.x
        imu.angular_velocity.z = msg.angular_velocity.z

        # 加速度 RFU→FLU
        imu.linear_acceleration.x = msg.linear_acceleration.y
        imu.linear_acceleration.y = -msg.linear_acceleration.x
        imu.linear_acceleration.z = msg.linear_acceleration.z

        # 协方差
        rp_cov = 0.01 if att_ok else 1e6
        yaw_cov = 0.01 if yaw_ok else 1e6
        imu.orientation_covariance = [
            rp_cov, 0.0,    0.0,
            0.0,    rp_cov, 0.0,
            0.0,    0.0,    yaw_cov,
        ]
        for i in [0, 4, 8]:
            imu.angular_velocity_covariance[i] = 1e-4
            imu.linear_acceleration_covariance[i] = 1e-2
        self.pub_imu.publish(imu)

        # ---------- 2 & 3. GNSS 降频发布 (~10Hz) ----------
        self._gnss_pub_counter += 1
        if self._gnss_pub_counter >= self.gnss_pub_divisor:
            self._gnss_pub_counter = 0

            # NavSatFix
            fix = NavSatFix()
            fix.header = msg.header
            fix.header.frame_id = 'gnss_link'
            fix.latitude = msg.latitude
            fix.longitude = msg.longitude
            fix.altitude = msg.altitude
            fix.status.status = NavSatStatus.STATUS_FIX if pos_ok else NavSatStatus.STATUS_NO_FIX
            fix.status.service = NavSatStatus.SERVICE_GPS
            pos_cov = 0.05 if pos_ok else 100.0
            fix.position_covariance = [
                pos_cov, 0.0, 0.0,
                0.0, pos_cov, 0.0,
                0.0, 0.0, pos_cov * 4,
            ]
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            self.pub_fix.publish(fix)

            # NavPVT
            pvt = NavPVT()
            pvt.i_tow = msg.gps_millisec
            pvt.lat = int(msg.latitude * 1e7)
            pvt.lon = int(msg.longitude * 1e7)
            pvt.height = int(msg.altitude * 1000)
            pvt.vel_n = int(msg.velocity.y * 1000)
            pvt.vel_e = int(msg.velocity.x * 1000)
            pvt.vel_d = int(-msg.velocity.z * 1000)
            pvt.g_speed = int(math.hypot(msg.velocity.x, msg.velocity.y) * 1000)
            heading_ned = math.atan2(msg.velocity.x, msg.velocity.y)
            pvt.heading = int(math.degrees(heading_ned) * 1e5)
            pvt.flags = 0x01 if pos_ok else 0x00
            self.pub_pvt.publish(pvt)

    def print_stats(self):
        if self._msg_count == 0:
            self.get_logger().warn('[Stats] ⚠️ No GnssIns message received in last 2s!')
            return
        rate = self._msg_count / 2.0
        gnss_rate = rate / self.gnss_pub_divisor
        mode = 'FORCED' if self.force_aligned else 'REAL'
        self.get_logger().info(
            f'[Stats] INS {rate:.1f}Hz | GNSS pub {gnss_rate:.1f}Hz | '
            f'status=0x{self._last_status:02X} ({mode})'
        )
        self._msg_count = 0


def main():
    rclpy.init()
    node = GnssInsConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
