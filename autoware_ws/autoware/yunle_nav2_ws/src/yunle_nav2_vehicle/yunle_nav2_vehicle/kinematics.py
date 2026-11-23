"""Pure Ackermann kinematics helpers, kept independent from ROS for testing."""

import math


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp value to the inclusive interval [lower, upper]."""
    return max(lower, min(upper, value))


def steering_from_twist(
    linear_velocity: float,
    angular_velocity: float,
    wheel_base: float,
    max_steering_angle: float,
    stopped_velocity: float = 0.01,
) -> float:
    """
    Convert a planar Twist command to a front tire steering angle.

    Nav2's angular velocity is a yaw-rate command.  It must never be copied
    directly into a steering-angle field.  A car cannot execute an in-place
    rotation, so commands below ``stopped_velocity`` result in zero steering.
    """
    if wheel_base <= 0.0:
        raise ValueError("wheel_base must be positive")
    if max_steering_angle <= 0.0:
        raise ValueError("max_steering_angle must be positive")
    if abs(linear_velocity) <= stopped_velocity:
        return 0.0

    steering = math.atan(wheel_base * angular_velocity / linear_velocity)
    return clamp(steering, -max_steering_angle, max_steering_angle)


def yaw_rate_from_steering(
    linear_velocity: float, steering_angle: float, wheel_base: float
) -> float:
    """Return bicycle-model yaw rate from measured speed and tire angle."""
    if wheel_base <= 0.0:
        raise ValueError("wheel_base must be positive")
    return linear_velocity * math.tan(steering_angle) / wheel_base


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi)."""
    return math.atan2(math.sin(angle), math.cos(angle))


def locked_map_to_odom(
    map_base_x: float,
    map_base_y: float,
    map_base_yaw: float,
    odom_base_x: float,
    odom_base_y: float,
    odom_base_yaw: float,
) -> tuple[float, float, float]:
    """Return the fixed planar map->odom transform for a requested base pose."""
    map_odom_yaw = normalize_angle(map_base_yaw - odom_base_yaw)
    cos_yaw = math.cos(map_odom_yaw)
    sin_yaw = math.sin(map_odom_yaw)
    map_odom_x = map_base_x - (
        cos_yaw * odom_base_x - sin_yaw * odom_base_y
    )
    map_odom_y = map_base_y - (
        sin_yaw * odom_base_x + cos_yaw * odom_base_y
    )
    return map_odom_x, map_odom_y, map_odom_yaw


def map_correction_is_acceptable(
    current_x: float,
    current_y: float,
    current_yaw: float,
    desired_x: float,
    desired_y: float,
    desired_yaw: float,
    covariance_x: float,
    covariance_y: float,
    covariance_yaw: float,
    max_translation_innovation: float,
    max_yaw_innovation: float,
    max_xy_covariance: float,
    max_yaw_covariance: float,
) -> bool:
    """Return whether an AMCL-derived map correction is safe to apply."""
    translation_innovation = math.hypot(desired_x - current_x, desired_y - current_y)
    yaw_innovation = abs(normalize_angle(desired_yaw - current_yaw))
    return (
        covariance_x <= max_xy_covariance
        and covariance_y <= max_xy_covariance
        and covariance_yaw <= max_yaw_covariance
        and translation_innovation <= max_translation_innovation
        and yaw_innovation <= max_yaw_innovation
    )


def soft_correct_map_to_odom(
    current_x: float,
    current_y: float,
    current_yaw: float,
    desired_x: float,
    desired_y: float,
    desired_yaw: float,
    gain: float,
    max_translation_step: float,
    max_yaw_step: float,
) -> tuple[float, float, float]:
    """Move map->odom smoothly toward a desired transform with bounded steps."""
    if not 0.0 <= gain <= 1.0:
        raise ValueError("gain must be in [0, 1]")
    if max_translation_step <= 0.0 or max_yaw_step <= 0.0:
        raise ValueError("correction step limits must be positive")

    step_x = gain * (desired_x - current_x)
    step_y = gain * (desired_y - current_y)
    step_norm = math.hypot(step_x, step_y)
    if step_norm > max_translation_step:
        scale = max_translation_step / step_norm
        step_x *= scale
        step_y *= scale

    yaw_error = normalize_angle(desired_yaw - current_yaw)
    yaw_step = clamp(gain * yaw_error, -max_yaw_step, max_yaw_step)
    return (
        current_x + step_x,
        current_y + step_y,
        normalize_angle(current_yaw + yaw_step),
    )
