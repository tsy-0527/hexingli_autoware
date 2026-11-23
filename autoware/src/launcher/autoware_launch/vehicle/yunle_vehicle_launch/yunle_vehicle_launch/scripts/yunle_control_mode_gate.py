#!/usr/bin/env python3

"""Synchronize the Yunle physical control handoff with Autoware authority."""

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_adapi_v1_msgs.srv import ChangeOperationMode
from chassis_interfaces.msg import CcuStatus


class YunleControlModeGate(Node):
    """Use the Yunle remote and chassis switches as the engage gate."""

    def __init__(self):
        super().__init__("yunle_control_mode_gate")

        self.declare_parameter("switch_stable_sec", 0.5)
        self.declare_parameter("status_timeout_sec", 1.0)
        self.declare_parameter("retry_interval_sec", 1.0)
        self.declare_parameter("handle_mode_flag_auto_value", False)
        self.declare_parameter(
            "yunle_ccu_status_topic", "/yunle_chassis/feedback/ccu_status"
        )

        self._switch_stable = Duration(
            seconds=float(self.get_parameter("switch_stable_sec").value)
        )
        self._status_timeout = Duration(
            seconds=float(self.get_parameter("status_timeout_sec").value)
        )
        self._retry_interval = Duration(
            seconds=float(self.get_parameter("retry_interval_sec").value)
        )
        self._handle_mode_flag_auto_value = bool(
            self.get_parameter("handle_mode_flag_auto_value").value
        )

        self._switch_enabled = None
        self._switch_changed_at = None
        self._last_ccu_status_at = None
        self._operation_state = None
        self._request_pending = False
        self._last_request_at = None
        self._last_wait_reason = None

        self.create_subscription(
            CcuStatus,
            str(self.get_parameter("yunle_ccu_status_topic").value),
            self._on_ccu_status,
            10,
        )
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OperationModeState,
            "/api/operation_mode/state",
            self._on_operation_state,
            state_qos,
        )

        self._enable_client = self.create_client(
            ChangeOperationMode,
            "/api/operation_mode/enable_autoware_control",
        )
        self._disable_client = self.create_client(
            ChangeOperationMode,
            "/api/operation_mode/disable_autoware_control",
        )
        self.create_timer(0.1, self._update)

        self.get_logger().info(
            "Yunle control mode gate started: the remote-controller AUTO position "
            "and chassis AUTO button form the physical control handoff"
        )

    def _now(self):
        return self.get_clock().now()

    def _on_ccu_status(self, msg):
        now = self._now()
        chassis_auto = bool(msg.ccu_drive_mode_shift_button)
        remote_auto = (
            bool(msg.handle_mode_flag_status)
            == self._handle_mode_flag_auto_value
        )
        enabled = chassis_auto and remote_auto
        self._last_ccu_status_at = now
        if enabled != self._switch_enabled:
            self._switch_enabled = enabled
            self._switch_changed_at = now
            self.get_logger().info(
                "Yunle physical handoff changed to "
                + ("AUTONOMOUS" if enabled else "MANUAL")
                + f" (chassis_button={chassis_auto}, "
                + f"handle_mode_flag={bool(msg.handle_mode_flag_status)})"
            )

    def _on_operation_state(self, msg):
        self._operation_state = msg

    def _log_wait_once(self, reason):
        if reason != self._last_wait_reason:
            self.get_logger().info(reason)
            self._last_wait_reason = reason

    def _update(self):
        if self._request_pending or self._operation_state is None:
            return
        if self._switch_enabled is None or self._last_ccu_status_at is None:
            return

        now = self._now()
        if (
            self._last_request_at is not None
            and now - self._last_request_at < self._retry_interval
        ):
            return

        status_is_stale = now - self._last_ccu_status_at > self._status_timeout
        switch_is_stable = (
            self._switch_changed_at is not None
            and now - self._switch_changed_at >= self._switch_stable
        )
        switch_requests_control = self._switch_enabled and not status_is_stale
        control_is_enabled = self._operation_state.is_autoware_control_enabled

        if not switch_requests_control:
            self._last_wait_reason = None
            if control_is_enabled:
                reason = (
                    "CCU status timed out"
                    if status_is_stale
                    else "Yunle physical handoff is MANUAL"
                )
                self._send_request(self._disable_client, "disable", reason)
            return

        if not switch_is_stable:
            return
        if control_is_enabled:
            self._last_wait_reason = None
            return
        if self._operation_state.mode != OperationModeState.AUTONOMOUS:
            self._log_wait_once(
                "Yunle handoff requests control; waiting for Autonomous operation mode"
            )
            return
        if not self._operation_state.is_autonomous_mode_available:
            self._log_wait_once(
                "Yunle handoff requests control, but Autonomous mode is unavailable"
            )
            return

        self._last_wait_reason = None
        self._send_request(
            self._enable_client,
            "enable",
            "Yunle physical handoff is AUTONOMOUS and mode is available",
        )

    def _send_request(self, client, action, reason):
        if not client.service_is_ready():
            self._log_wait_once(f"Autoware control {action} service is not ready")
            return

        self._request_pending = True
        self._last_request_at = self._now()
        self.get_logger().info(f"Requesting {action} Autoware control: {reason}")
        future = client.call_async(ChangeOperationMode.Request())
        future.add_done_callback(
            lambda completed, requested=action: self._on_request_done(
                completed, requested
            )
        )

    def _on_request_done(self, future, action):
        self._request_pending = False
        try:
            response = future.result()
            if response.status.success:
                self.get_logger().info(
                    f"Autoware control {action} request succeeded"
                )
            else:
                self.get_logger().error(
                    f"Autoware control {action} request rejected: "
                    f"code={response.status.code}, message={response.status.message}"
                )
        except Exception as error:  # noqa: BLE001 - report ROS service failures
            self.get_logger().error(
                f"Autoware control {action} service call failed: {error}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = YunleControlModeGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
