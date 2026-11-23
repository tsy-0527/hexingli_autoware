#!/usr/bin/env python3

"""Synchronize the Hunter remote-control switch with Autoware control authority."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_adapi_v1_msgs.srv import ChangeOperationMode
from autoware_vehicle_msgs.msg import ControlModeReport


class HunterControlModeGate(Node):
    """Use the physical Hunter mode switch as the final Autoware engage gate."""

    def __init__(self):
        super().__init__("hunter_control_mode_gate")

        self.declare_parameter("mode_stable_sec", 0.5)
        self.declare_parameter("status_timeout_sec", 1.0)
        self.declare_parameter("retry_interval_sec", 1.0)

        self._mode_stable_ns = int(
            float(self.get_parameter("mode_stable_sec").value) * 1e9
        )
        self._status_timeout_ns = int(
            float(self.get_parameter("status_timeout_sec").value) * 1e9
        )
        self._retry_interval_ns = int(
            float(self.get_parameter("retry_interval_sec").value) * 1e9
        )

        self._vehicle_mode = None
        self._vehicle_mode_since_ns = None
        self._last_vehicle_status_ns = None
        self._operation_state = None
        self._request_pending = False
        self._last_request_ns = 0
        self._last_wait_reason = None

        self.create_subscription(
            ControlModeReport,
            "/vehicle/status/control_mode",
            self._on_vehicle_mode,
            10,
        )
        self.create_subscription(
            OperationModeState,
            "/api/operation_mode/state",
            self._on_operation_state,
            10,
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
            "Hunter remote mode gate started: CAN/autonomous enables Autoware control; "
            "manual mode disables it"
        )

    def _now_ns(self):
        return self.get_clock().now().nanoseconds

    def _on_vehicle_mode(self, msg):
        now_ns = self._now_ns()
        self._last_vehicle_status_ns = now_ns
        if msg.mode != self._vehicle_mode:
            self._vehicle_mode = msg.mode
            self._vehicle_mode_since_ns = now_ns
            label = (
                "AUTONOMOUS"
                if msg.mode == ControlModeReport.AUTONOMOUS
                else f"MANUAL/OTHER({msg.mode})"
            )
            self.get_logger().info(f"Hunter remote switch changed to {label}")

    def _on_operation_state(self, msg):
        self._operation_state = msg

    def _log_wait_once(self, reason):
        if reason != self._last_wait_reason:
            self.get_logger().info(reason)
            self._last_wait_reason = reason

    def _update(self):
        if self._request_pending or self._vehicle_mode is None:
            return

        now_ns = self._now_ns()
        if now_ns - self._last_request_ns < self._retry_interval_ns:
            return

        status_is_stale = (
            self._last_vehicle_status_ns is not None
            and now_ns - self._last_vehicle_status_ns > self._status_timeout_ns
        )
        mode_is_stable = (
            self._vehicle_mode_since_ns is not None
            and now_ns - self._vehicle_mode_since_ns >= self._mode_stable_ns
        )

        if self._operation_state is None:
            return

        remote_requests_control = (
            not status_is_stale
            and self._vehicle_mode == ControlModeReport.AUTONOMOUS
        )
        control_is_enabled = self._operation_state.is_autoware_control_enabled

        if not remote_requests_control:
            self._last_wait_reason = None
            if control_is_enabled:
                reason = (
                    "vehicle control-mode status timed out"
                    if status_is_stale
                    else "Hunter remote switch is not autonomous"
                )
                self._send_request(self._disable_client, False, reason)
            return

        # Debounce only the transition into autonomous control. A transition
        # back to manual, or loss of vehicle status, must disengage immediately.
        if not mode_is_stable:
            return

        if control_is_enabled:
            self._last_wait_reason = None
            return

        if self._operation_state.mode != OperationModeState.AUTONOMOUS:
            self._log_wait_once(
                "Remote switch requests autonomous control; waiting for RViz Auto mode"
            )
            return

        if not self._operation_state.is_autonomous_mode_available:
            self._log_wait_once(
                "Remote switch requests autonomous control, but Autonomous mode is unavailable"
            )
            return

        self._last_wait_reason = None
        self._send_request(
            self._enable_client,
            True,
            "Hunter remote switch entered autonomous mode",
        )

    def _send_request(self, client, enable, reason):
        if not client.service_is_ready():
            self._log_wait_once(
                f"Autoware {'enable' if enable else 'disable'} control service is not ready"
            )
            return

        self._request_pending = True
        self._last_request_ns = self._now_ns()
        action = "enable" if enable else "disable"
        self.get_logger().info(f"Requesting {action} Autoware control: {reason}")
        future = client.call_async(ChangeOperationMode.Request())
        future.add_done_callback(
            lambda completed, requested=action: self._on_request_done(completed, requested)
        )

    def _on_request_done(self, future, action):
        self._request_pending = False
        try:
            response = future.result()
            if response.status.success:
                self.get_logger().info(f"Autoware control {action} request succeeded")
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
    node = HunterControlModeGate()
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
