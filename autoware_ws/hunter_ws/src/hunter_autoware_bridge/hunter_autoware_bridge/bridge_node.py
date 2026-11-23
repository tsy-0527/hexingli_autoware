#!/usr/bin/env python3
"""
bridge_node.py

Hunter SE <-> Autoware 桥接节点

Autoware → Hunter:
  订阅 /control/command/control_cmd (autoware_control_msgs/Control)
  发布 /cmd_vel (geometry_msgs/Twist)

Hunter → Autoware:
  订阅 /hunter_status (hunter_msgs/HunterStatus)
  发布 /vehicle/status/velocity_status  (autoware_vehicle_msgs/VelocityReport)
  发布 /vehicle/status/steering_status  (autoware_vehicle_msgs/SteeringReport)
  发布 /vehicle/status/gear_status      (autoware_vehicle_msgs/GearReport)
  发布 /vehicle/status/control_mode     (autoware_vehicle_msgs/ControlModeReport)

注意事项：
  - hunter_base 已将内轮角转为中心角(rad)，bridge 直接使用
  - Autoware 发来 steering_tire_angle(rad)，bridge 转为 omega 发给 hunter_base
  - CONTROL_MODE_CAN 需要实车确认：
      ros2 topic echo /hunter_status --field control_mode
      遥控模式下记录值，切换自主模式后记录值，填入下方
"""

import math
import rclpy
from rclpy.node import Node

from hunter_msgs.msg import HunterStatus
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import (
    VelocityReport,
    SteeringReport,
    GearReport,
    ControlModeReport,
)
from geometry_msgs.msg import Twist

# ── 待实车确认的常量 ────────────────────────────────────────────────────────
# 实车确认方法：
#   ros2 topic echo /hunter_status --field control_mode
#   遥控时的值填入 CONTROL_MODE_RC，自主模式的值填入 CONTROL_MODE_CAN
CONTROL_MODE_RC  = 0   # 遥控模式（待确认）
CONTROL_MODE_CAN = 1   # 自主模式（待确认）

# ── 车辆参数 ────────────────────────────────────────────────────────────────
WHEELBASE       = 0.648   # 轴距(m)，需实测确认
MAX_LINEAR      = 1.5     # 最大线速度限幅(m/s)
MAX_ANGULAR     = 1.0     # 最大角速度限幅(rad/s)
MAX_STEER_ANGLE = 0.52    # 最大转向角(rad)，需与 vehicle_info.param.yaml 统一


class HunterAutowareBridge(Node):
    def __init__(self):
        super().__init__('hunter_autoware_bridge')

        # ── 参数声明（可通过 launch 文件覆盖）──────────────────────────────
        self.declare_parameter('wheelbase',          WHEELBASE)
        self.declare_parameter('max_linear_speed',   MAX_LINEAR)
        self.declare_parameter('max_angular_speed',  MAX_ANGULAR)
        self.declare_parameter('max_steer_angle',    MAX_STEER_ANGLE)
        self.declare_parameter('control_mode_can',   CONTROL_MODE_CAN)

        self.wheelbase        = self.get_parameter('wheelbase').value
        self.max_linear       = self.get_parameter('max_linear_speed').value
        self.max_angular      = self.get_parameter('max_angular_speed').value
        self.max_steer        = self.get_parameter('max_steer_angle').value
        self.control_mode_can = self.get_parameter('control_mode_can').value

        # ── 状态变量 ────────────────────────────────────────────────────────
        self._current_control_mode = -1
        self._last_control_mode    = -1

        # ── 订阅 ────────────────────────────────────────────────────────────
        self.sub_ctrl = self.create_subscription(
            Control,
            '/control/command/control_cmd',
            self._control_callback,
            10,
        )
        self.sub_status = self.create_subscription(
            HunterStatus,
            '/hunter_status',
            self._status_callback,
            10,
        )

        # ── 发布 ────────────────────────────────────────────────────────────
        self.pub_cmd   = self.create_publisher(Twist,             '/cmd_vel',                                          10)
        self.pub_vel   = self.create_publisher(VelocityReport,    '/vehicle/status/velocity_status',                   10)
        self.pub_steer = self.create_publisher(SteeringReport,    '/vehicle/status/steering_status',                   10)
        self.pub_gear  = self.create_publisher(GearReport,        '/vehicle/status/gear_status',                       10)
        self.pub_mode  = self.create_publisher(ControlModeReport, '/vehicle/status/control_mode',                      10)

        self.get_logger().info('HunterAutowareBridge 已启动')
        self.get_logger().warn(
            f'CONTROL_MODE_CAN={self.control_mode_can}，请实车确认后修改参数')

    # ── Autoware → Hunter ──────────────────────────────────────────────────
    def _control_callback(self, msg: Control):
        """
        将 Autoware 控制指令转换为 Hunter 底盘的 Twist 指令

        转换逻辑（阿克曼模型）：
          omega = v * tan(steering_tire_angle) / wheelbase
        """
        # 非自主模式时不转发，防止遥控时被 Autoware 抢控
        if self._current_control_mode != self.control_mode_can:
            return

        v     = float(msg.longitudinal.velocity)
        steer = float(msg.lateral.steering_tire_angle)

        # 限幅
        v     = max(-self.max_linear, min(self.max_linear, v))
        steer = max(-self.max_steer,  min(self.max_steer,  steer))

        # 阿克曼转差速
        omega = v * math.tan(steer) / self.wheelbase
        omega = max(-self.max_angular, min(self.max_angular, omega))

        twist = Twist()
        twist.linear.x  = v
        twist.angular.z = omega
        self.pub_cmd.publish(twist)

    # ── Hunter → Autoware ──────────────────────────────────────────────────
    def _status_callback(self, msg: HunterStatus):
        """
        将 Hunter 底盘状态转换为 Autoware 标准话题

        说明：
          - msg.steering_angle 由 hunter_base 已转换为中心角(rad)，直接使用
          - msg.linear_velocity 单位 m/s，直接使用
        """
        now = self.get_clock().now().to_msg()

        # ── 故障检测 ──────────────────────────────────────────────────────
        if msg.vehicle_state != 0:
            self.get_logger().error(
                f'Hunter 故障！vehicle_state={msg.vehicle_state} '
                f'error_code={msg.error_code}',
                throttle_duration_sec=1.0,
            )
            self.pub_cmd.publish(Twist())  # 急停
            return

        # ── 控制模式变化日志 ──────────────────────────────────────────────
        if msg.control_mode != self._last_control_mode:
            mode_str = (
                'CAN自主模式' if msg.control_mode == self.control_mode_can
                else f'其他模式({msg.control_mode})'
            )
            self.get_logger().info(f'控制模式切换 → {mode_str}')
            self._last_control_mode = msg.control_mode

        self._current_control_mode = msg.control_mode

        # ── 速度反馈 ──────────────────────────────────────────────────────
        # heading_rate 由阿克曼模型推导：ω = v * tan(δ) / L
        # v     = msg.linear_velocity
        raw_v = msg.linear_velocity
        v = 0.0 if abs(raw_v) < 0.05 else raw_v
        steer = msg.steering_angle  # 已是中心角(rad)

        heading_rate = (
            v * math.tan(steer) / self.wheelbase
            if abs(v) > 0.01 else 0.0
        )

        vel_msg = VelocityReport()
        vel_msg.header.stamp    = now
        vel_msg.header.frame_id = 'base_link'
        vel_msg.longitudinal_velocity = v
        vel_msg.lateral_velocity      = 0.0
        vel_msg.heading_rate          = heading_rate
        self.pub_vel.publish(vel_msg)

        # ── 转向角反馈 ────────────────────────────────────────────────────
        steer_msg = SteeringReport()
        steer_msg.stamp               = now
        steer_msg.steering_tire_angle = steer  # rad，已是中心角
        self.pub_steer.publish(steer_msg)

        # ── 档位反馈 ──────────────────────────────────────────────────────
        gear_msg = GearReport()
        gear_msg.stamp  = now
        gear_msg.report = (
            GearReport.DRIVE   if v >= -0.01
            else GearReport.REVERSE
        )
        self.pub_gear.publish(gear_msg)

        # ── 控制模式反馈 ──────────────────────────────────────────────────
        mode_msg = ControlModeReport()
        mode_msg.stamp = now
        mode_msg.mode  = (
            ControlModeReport.AUTONOMOUS
            if msg.control_mode == self.control_mode_can
            else ControlModeReport.MANUAL
        )
        self.pub_mode.publish(mode_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HunterAutowareBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 关闭前发零速，确保底盘停止
        node.pub_cmd.publish(Twist())
        node.get_logger().info('Bridge 关闭，已发送零速指令')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

