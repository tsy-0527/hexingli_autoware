#!/usr/bin/env python3

"""Translate between Autoware vehicle interfaces and the native Yunle chassis driver."""

import math
import time

import rclpy
from rclpy.node import Node

from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import (
    ControlModeReport,
    GearCommand,
    GearReport,
    HazardLightsCommand,
    HazardLightsReport,
    SteeringReport,
    TurnIndicatorsCommand,
    TurnIndicatorsReport,
    VelocityReport,
)
from autoware_vehicle_msgs.srv import ControlModeCommand
from chassis_interfaces.msg import CcuStatus, SasAngleFeedback, ScuControlCommand
from tier4_vehicle_msgs.msg import VehicleEmergencyStamped


class YunleAutowareBridge(Node):
    """Bridge gated Autoware commands to Yunle SCU commands and normalize feedback."""

    def __init__(self):
        super().__init__("yunle_autoware_bridge")

        self.wheel_base = float(self.declare_parameter("wheel_base", 0.475).value)
        self.max_target_speed_kmh = float(
            self.declare_parameter("max_target_speed_kmh", 15.0).value
        )
        self.max_steering_angle_deg = float(
            self.declare_parameter("max_steering_angle_deg", 21.0).value
        )
        # Keep the legacy parameter as a fallback for older parameter files,
        # but allow command and feedback conventions to be calibrated
        # independently.  Autoware uses left-positive steering while this
        # Yunle chassis uses right-positive steering commands.  The raw SAS
        # feedback is also normalized independently because its observed sign
        # is opposite to Autoware's left-positive convention.
        legacy_steering_sign = float(
            self.declare_parameter("steering_sign", 1.0).value
        )
        self.steering_command_sign = float(
            self.declare_parameter(
                "steering_command_sign", legacy_steering_sign
            ).value
        )
        self.steering_feedback_sign = float(
            self.declare_parameter(
                "steering_feedback_sign", legacy_steering_sign
            ).value
        )
        self.steering_feedback_offset_rad = math.radians(
            float(self.declare_parameter("steering_feedback_offset_deg", 0.0).value)
        )
        self.steering_feedback_deadband_rad = math.radians(
            max(
                0.0,
                float(
                    self.declare_parameter(
                        "steering_feedback_deadband_deg", 0.0
                    ).value
                ),
            )
        )
        self.autonomous_drive_mode = int(
            self.declare_parameter("autonomous_drive_mode", 1).value
        )
        # Protocol 0x51 bit 37 reports whether the hand-controller mode is
        # open.  On the current vehicle, remote AUTO is observed as false.
        # Keep the polarity configurable for other chassis revisions.
        self.handle_mode_flag_auto_value = bool(
            self.declare_parameter("handle_mode_flag_auto_value", False).value
        )
        # The current Yunle chassis keeps CCU_Parking_Status asserted even
        # when its physical brake buttons are released.  Make use of that bit
        # optional so the Autoware gear report can follow the actual D/N/R
        # feedback instead of being permanently overridden to PARK.
        self.use_ccu_parking_status_for_gear_report = bool(
            self.declare_parameter(
                "use_ccu_parking_status_for_gear_report", False
            ).value
        )
        self.command_timeout_sec = float(
            self.declare_parameter("command_timeout_sec", 0.2).value
        )
        self.feedback_timeout_sec = max(
            0.1, float(self.declare_parameter("feedback_timeout_sec", 0.5).value)
        )
        self.safe_stop_hold_sec = max(
            0.0, float(self.declare_parameter("safe_stop_hold_sec", 0.5).value)
        )
        self.release_control_on_timeout = bool(
            self.declare_parameter("release_control_on_timeout", True).value
        )
        self.stop_speed_threshold_mps = float(
            self.declare_parameter("stop_speed_threshold_mps", 0.01).value
        )
        self.frame_id = str(self.declare_parameter("frame_id", "base_link").value)

        control_topic = self._topic("autoware_control_topic", "/control/command/control_cmd")
        gear_topic = self._topic("autoware_gear_topic", "/control/command/gear_cmd")
        turn_topic = self._topic(
            "autoware_turn_indicators_topic", "/control/command/turn_indicators_cmd"
        )
        hazard_topic = self._topic(
            "autoware_hazard_lights_topic", "/control/command/hazard_lights_cmd"
        )
        emergency_topic = self._topic(
            "autoware_emergency_topic", "/control/command/emergency_cmd"
        )
        yunle_control_topic = self._topic(
            "yunle_control_topic", "/yunle_chassis/control/scu_control_command"
        )
        ccu_status_topic = self._topic(
            "yunle_ccu_status_topic", "/yunle_chassis/feedback/ccu_status"
        )
        sas_angle_topic = self._topic(
            "yunle_sas_angle_topic", "/yunle_chassis/feedback/sas_angle"
        )

        self.command_pub = self.create_publisher(ScuControlCommand, yunle_control_topic, 10)
        self.velocity_pub = self.create_publisher(
            VelocityReport, "/vehicle/status/velocity_status", 10
        )
        self.steering_pub = self.create_publisher(
            SteeringReport, "/vehicle/status/steering_status", 10
        )
        self.gear_pub = self.create_publisher(GearReport, "/vehicle/status/gear_status", 10)
        self.control_mode_pub = self.create_publisher(
            ControlModeReport, "/vehicle/status/control_mode", 10
        )
        self.turn_report_pub = self.create_publisher(
            TurnIndicatorsReport, "/vehicle/status/turn_indicators_status", 10
        )
        self.hazard_report_pub = self.create_publisher(
            HazardLightsReport, "/vehicle/status/hazard_lights_status", 10
        )

        self.create_subscription(Control, control_topic, self._on_control, 10)
        self.create_subscription(GearCommand, gear_topic, self._on_gear, 10)
        self.create_subscription(TurnIndicatorsCommand, turn_topic, self._on_turn, 10)
        self.create_subscription(HazardLightsCommand, hazard_topic, self._on_hazard, 10)
        self.create_subscription(
            VehicleEmergencyStamped, emergency_topic, self._on_emergency, 10
        )
        self.create_subscription(CcuStatus, ccu_status_topic, self._on_ccu_status, 10)
        self.create_subscription(SasAngleFeedback, sas_angle_topic, self._on_sas_angle, 10)
        self.create_service(
            ControlModeCommand,
            "/control/control_mode_request",
            self._on_control_mode_request,
        )

        self.shift_request = ScuControlCommand.SHIFT_LEVEL_N
        self.turn_command = TurnIndicatorsCommand.DISABLE
        self.hazard_command = HazardLightsCommand.DISABLE
        self.emergency = False
        # Match the Hunter integration: the physical handoff is the source of
        # truth for Autoware control authority.  For this Yunle chassis that
        # means the chassis AUTO button is pressed and the remote controller is
        # in its calibrated AUTO position.
        self.physical_control_granted = False
        self.latest_control = None
        self.last_control_monotonic = None
        self.latest_ccu_status = None
        self.last_ccu_status_monotonic = None
        self.last_ccu_drive_mode = None
        self.watchdog_active = False
        self.watchdog_started_monotonic = None
        self.control_released = False
        self.latest_steering_rad = 0.0

        watchdog_period = max(0.02, min(0.05, self.command_timeout_sec / 2.0))
        self.watchdog_timer = self.create_timer(watchdog_period, self._watchdog)

        self.get_logger().info(
            "Yunle Autoware bridge ready: "
            f"{control_topic} -> {yunle_control_topic}, "
            f"speed limit={self.max_target_speed_kmh:.2f} km/h, "
            f"steering limit={self.max_steering_angle_deg:.2f} deg, "
            f"steering signs=(command {self.steering_command_sign:+.1f}, "
            f"feedback {self.steering_feedback_sign:+.1f}), "
            "steering feedback offset="
            f"{math.degrees(self.steering_feedback_offset_rad):+.4f} deg, "
            "steering feedback deadband="
            f"{math.degrees(self.steering_feedback_deadband_rad):.4f} deg, "
            f"timeout={self.command_timeout_sec:.2f} s, "
            f"feedback timeout={self.feedback_timeout_sec:.2f} s, "
            f"remote AUTO handle flag={self.handle_mode_flag_auto_value}, "
            "parking bit for gear report="
            f"{self.use_ccu_parking_status_for_gear_report}, "
            f"safe-stop hold={self.safe_stop_hold_sec:.2f} s, "
            f"release on timeout={self.release_control_on_timeout}"
        )

    def _topic(self, parameter_name, default):
        return str(self.declare_parameter(parameter_name, default).value)

    @staticmethod
    def _clamp(value, lower, upper):
        return max(lower, min(value, upper))

    def _on_control(self, msg):
        self.latest_control = msg
        self.last_control_monotonic = time.monotonic()
        if not self.physical_control_granted:
            return
        self.watchdog_active = False
        self.watchdog_started_monotonic = None
        self.control_released = False
        self._publish_control(msg)

    def _ccu_status_is_fresh(self):
        if self.latest_ccu_status is None or self.last_ccu_status_monotonic is None:
            return False
        return (
            time.monotonic() - self.last_ccu_status_monotonic
            <= self.feedback_timeout_sec
        )

    def _physical_handoff_is_granted(self, msg):
        chassis_auto = bool(msg.ccu_drive_mode_shift_button)
        remote_auto = (
            bool(msg.handle_mode_flag_status) == self.handle_mode_flag_auto_value
        )
        return chassis_auto and remote_auto

    def _physical_control_is_active(self):
        return self.physical_control_granted and self._ccu_status_is_fresh()

    def _on_control_mode_request(self, request, response):
        if request.mode == ControlModeCommand.Request.AUTONOMOUS:
            if not self._ccu_status_is_fresh():
                self.get_logger().warn(
                    "Rejecting autonomous control: CCU status is missing or stale"
                )
                response.success = False
                return response
            if not self._physical_handoff_is_granted(self.latest_ccu_status):
                self.get_logger().warn(
                    "Rejecting autonomous control: physical handoff is not granted "
                    "by both the remote controller and chassis AUTO button"
                )
                response.success = False
                return response

            self.watchdog_active = False
            self.watchdog_started_monotonic = None
            self.control_released = False
            response.success = True
            self.get_logger().info(
                "Autonomous control confirmed: physical handoff is granted"
            )

            # The Yunle VCU changes from remote/manual to autonomous only after
            # receiving valid 0x121 frames, so do not wait for ccu_drive_mode.
            if self._control_is_fresh():
                self._publish_control(self.latest_control)
            return response

        if request.mode == ControlModeCommand.Request.MANUAL:
            if self._physical_control_is_active():
                self.get_logger().warn(
                    "Rejecting manual control request while the physical handoff "
                    "still grants control to Autoware"
                )
                response.success = False
                return response
            self.watchdog_active = False
            self.watchdog_started_monotonic = None
            self.control_released = True
            response.success = True
            self.get_logger().info("Manual control confirmed by physical switches")
            return response

        self.get_logger().warn(
            f"Unsupported control mode request: {int(request.mode)}"
        )
        response.success = False
        return response

    def _control_is_fresh(self):
        if self.latest_control is None or self.last_control_monotonic is None:
            return False
        return (
            time.monotonic() - self.last_control_monotonic
            <= self.command_timeout_sec
        )

    def _on_gear(self, msg):
        if msg.command in range(GearCommand.DRIVE, GearCommand.DRIVE_18 + 1):
            self.shift_request = ScuControlCommand.SHIFT_LEVEL_D
        elif msg.command in (GearCommand.REVERSE, GearCommand.REVERSE_2):
            self.shift_request = ScuControlCommand.SHIFT_LEVEL_R
        else:
            self.shift_request = ScuControlCommand.SHIFT_LEVEL_N
        if self._control_is_fresh():
            self._publish_control(self.latest_control)

    def _on_turn(self, msg):
        self.turn_command = msg.command
        if self._control_is_fresh():
            self._publish_control(self.latest_control)

    def _on_hazard(self, msg):
        self.hazard_command = msg.command
        if self._control_is_fresh():
            self._publish_control(self.latest_control)

    def _on_emergency(self, msg):
        self.emergency = bool(msg.emergency)
        if self.emergency:
            self._publish_safe_stop()
        elif self._control_is_fresh():
            self._publish_control(self.latest_control)

    def _publish_control(self, control):
        if self.emergency:
            self._publish_safe_stop()
            return
        if not self._physical_control_is_active():
            return

        velocity_mps = float(control.longitudinal.velocity)
        direction_valid = (
            self.shift_request == ScuControlCommand.SHIFT_LEVEL_D
            and velocity_mps >= -self.stop_speed_threshold_mps
        ) or (
            self.shift_request == ScuControlCommand.SHIFT_LEVEL_R
            and velocity_mps <= self.stop_speed_threshold_mps
        )

        speed_kmh = abs(velocity_mps) * 3.6 if direction_valid else 0.0
        speed_kmh = self._clamp(speed_kmh, 0.0, self.max_target_speed_kmh)
        steering_deg = math.degrees(float(control.lateral.steering_tire_angle))
        steering_deg = self._clamp(
            steering_deg * self.steering_command_sign,
            -self.max_steering_angle_deg,
            self.max_steering_angle_deg,
        )

        command = self._base_command()
        command.scu_shift_level_request = self.shift_request
        command.scu_target_speed = float(speed_kmh)
        command.scu_steering_angle_front = float(steering_deg)
        command.scu_brake_enable = speed_kmh <= self.stop_speed_threshold_mps * 3.6
        self.command_pub.publish(command)

    def _base_command(self):
        command = ScuControlCommand()
        command.scu_shift_level_request = self.shift_request
        command.scu_steering_angle_front = 0.0
        command.scu_steering_angle_rear = 0.0
        command.scu_target_speed = 0.0
        command.scu_brake_enable = False

        hazard_on = self.hazard_command == HazardLightsCommand.ENABLE
        command.gw_left_turn_light_request = int(
            hazard_on or self.turn_command == TurnIndicatorsCommand.ENABLE_LEFT
        )
        command.gw_right_turn_light_request = int(
            hazard_on or self.turn_command == TurnIndicatorsCommand.ENABLE_RIGHT
        )
        command.gw_position_light_request = 0
        command.gw_low_beam_request = 0
        # This bridge sends SCU_Target_Speed, so select the DBC's SpeedMode
        # (0). TorqueMode (1) requires a separate wheel-torque command and can
        # prevent the VCU from accepting this 0x121 command as autonomous
        # speed control.
        command.scu_torque_or_speed_mode = 0
        # The steering/speed command is valid.  Do not assert the EMB brake
        # force flag because this bridge does not publish the corresponding
        # per-wheel brake-force command.
        command.steering_angle_speed_valid = True
        command.brake_force_command_valid = False
        return command

    def _publish_safe_stop(self):
        command = self._base_command()
        command.scu_target_speed = 0.0
        command.scu_brake_enable = True
        self.command_pub.publish(command)

    def _watchdog(self):
        # An asserted emergency must keep the brake command alive regardless of
        # normal command freshness or manual-takeover behavior.
        if self.emergency:
            self._publish_safe_stop()
            return

        if not self._physical_control_is_active():
            return

        if self.last_control_monotonic is None:
            return
        now = time.monotonic()
        if now - self.last_control_monotonic <= self.command_timeout_sec:
            return

        if not self.watchdog_active:
            self.get_logger().warn(
                "Autoware control command timeout; commanding zero speed and brake"
            )
            self.watchdog_active = True
            self.watchdog_started_monotonic = now
            # Always replace a possibly moving cached command with at least one
            # explicit stop command before allowing the driver to time out.
            self._publish_safe_stop()
            return

        if not self.release_control_on_timeout:
            self._publish_safe_stop()
            return

        if now - self.watchdog_started_monotonic < self.safe_stop_hold_sec:
            self._publish_safe_stop()
            return

        if not self.control_released:
            self.get_logger().warn(
                "Safe-stop hold complete; stop publishing SCU commands so the "
                "chassis driver and VCU can time out and return control to manual mode"
            )
            self.control_released = True

        # The chassis driver repeats the last safe-stop frame for its own short
        # timeout and then stops transmitting 0x121. A later fresh command may
        # resume output only
        # while the physical handoff remains granted, matching Hunter behavior.

    def _on_sas_angle(self, msg):
        normalized_steering_rad = (
            math.radians(float(msg.sas_front_angle)) * self.steering_feedback_sign
        )
        max_steering_rad = math.radians(self.max_steering_angle_deg)
        corrected_steering_rad = (
            normalized_steering_rad - self.steering_feedback_offset_rad
        )
        if abs(corrected_steering_rad) <= self.steering_feedback_deadband_rad:
            corrected_steering_rad = 0.0
        self.latest_steering_rad = self._clamp(
            corrected_steering_rad,
            -max_steering_rad,
            max_steering_rad,
        )
        report = SteeringReport()
        report.stamp = msg.stamp
        report.steering_tire_angle = float(self.latest_steering_rad)
        self.steering_pub.publish(report)

    def _on_ccu_status(self, msg):
        previous_handoff = self.physical_control_granted
        self.latest_ccu_status = msg
        self.last_ccu_status_monotonic = time.monotonic()
        self.physical_control_granted = self._physical_handoff_is_granted(msg)

        if self.physical_control_granted != previous_handoff:
            self.get_logger().info(
                "Physical control handoff changed to "
                + ("AUTONOMOUS" if self.physical_control_granted else "MANUAL")
                + f" (chassis_button={bool(msg.ccu_drive_mode_shift_button)}, "
                + f"handle_mode_flag={bool(msg.handle_mode_flag_status)})"
            )
            self.watchdog_active = False
            self.watchdog_started_monotonic = None
            self.control_released = not self.physical_control_granted
            if self.physical_control_granted and self._control_is_fresh():
                self._publish_control(self.latest_control)

        if int(msg.ccu_drive_mode) != self.last_ccu_drive_mode:
            self.last_ccu_drive_mode = int(msg.ccu_drive_mode)
            label = (
                "AUTO"
                if self.last_ccu_drive_mode == self.autonomous_drive_mode
                else f"OTHER({self.last_ccu_drive_mode})"
            )
            self.get_logger().info(f"VCU actual drive mode changed to {label}")

        stamp = msg.stamp
        velocity_mps = float(msg.ccu_vehicle_speed) / 3.6
        if msg.ccu_shift_level_status == ScuControlCommand.SHIFT_LEVEL_R:
            velocity_mps = -velocity_mps

        velocity = VelocityReport()
        velocity.header.stamp = stamp
        velocity.header.frame_id = self.frame_id
        velocity.longitudinal_velocity = velocity_mps
        velocity.lateral_velocity = 0.0
        velocity.heading_rate = (
            velocity_mps * math.tan(self.latest_steering_rad) / self.wheel_base
            if self.wheel_base > 0.0
            else 0.0
        )
        self.velocity_pub.publish(velocity)

        gear = GearReport()
        gear.stamp = stamp
        if (
            self.use_ccu_parking_status_for_gear_report
            and msg.ccu_parking_status
        ):
            gear.report = GearReport.PARK
        elif msg.ccu_shift_level_status == ScuControlCommand.SHIFT_LEVEL_D:
            gear.report = GearReport.DRIVE
        elif msg.ccu_shift_level_status == ScuControlCommand.SHIFT_LEVEL_R:
            gear.report = GearReport.REVERSE
        else:
            gear.report = GearReport.NEUTRAL
        self.gear_pub.publish(gear)

        mode = ControlModeReport()
        mode.stamp = stamp
        mode.mode = (
            ControlModeReport.AUTONOMOUS
            if self.physical_control_granted
            else ControlModeReport.MANUAL
        )
        self.control_mode_pub.publish(mode)

        both_lights = bool(msg.left_turn_light_status and msg.right_turn_light_status)
        turn = TurnIndicatorsReport()
        turn.stamp = stamp
        if both_lights:
            turn.report = TurnIndicatorsReport.DISABLE
        elif msg.left_turn_light_status:
            turn.report = TurnIndicatorsReport.ENABLE_LEFT
        elif msg.right_turn_light_status:
            turn.report = TurnIndicatorsReport.ENABLE_RIGHT
        else:
            turn.report = TurnIndicatorsReport.DISABLE
        self.turn_report_pub.publish(turn)

        hazard = HazardLightsReport()
        hazard.stamp = stamp
        hazard.report = (
            HazardLightsReport.ENABLE if both_lights else HazardLightsReport.DISABLE
        )
        self.hazard_report_pub.publish(hazard)


def main(args=None):
    rclpy.init(args=args)
    node = YunleAutowareBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._publish_safe_stop()
            node.destroy_node()
            rclpy.shutdown()
        else:
            node.destroy_node()


if __name__ == "__main__":
    main()
