"""Unit tests for Yunle Ackermann kinematics."""

import math

import pytest

from yunle_nav2_vehicle.kinematics import (
    locked_map_to_odom,
    map_correction_is_acceptable,
    normalize_angle,
    soft_correct_map_to_odom,
    steering_from_twist,
    yaw_rate_from_steering,
)


def test_twist_round_trip_forward():
    """Forward Twist conversion preserves the requested yaw rate."""
    steering = steering_from_twist(0.5, 0.2, 0.475, math.radians(21.0))
    assert yaw_rate_from_steering(0.5, steering, 0.475) == pytest.approx(0.2)


def test_twist_round_trip_reverse():
    """Reverse conversion applies the correct steering sign."""
    steering = steering_from_twist(-0.2, 0.1, 0.475, math.radians(21.0))
    assert steering < 0.0
    assert yaw_rate_from_steering(-0.2, steering, 0.475) == pytest.approx(0.1)


def test_in_place_rotation_is_rejected():
    """A car-like chassis must not accept an in-place rotation."""
    assert steering_from_twist(0.0, 1.0, 0.475, math.radians(21.0)) == 0.0


def test_steering_is_limited():
    """Steering conversion observes the configured tire-angle limit."""
    limit = math.radians(21.0)
    assert steering_from_twist(0.1, 10.0, 0.475, limit) == pytest.approx(limit)


def test_normalize_angle():
    """Angle normalization keeps values in one revolution."""
    assert normalize_angle(3.0 * math.pi) == pytest.approx(math.pi)


def test_locked_map_to_odom_translation():
    """A translated odom pose is removed from the requested map base pose."""
    x, y, heading = locked_map_to_odom(5.0, 1.0, 0.0, 2.0, 0.0, 0.0)
    assert x == pytest.approx(3.0)
    assert y == pytest.approx(1.0)
    assert heading == pytest.approx(0.0)


def test_locked_map_to_odom_rotation():
    """The odom translation is rotated before solving the fixed transform."""
    x, y, heading = locked_map_to_odom(
        2.0, 3.0, math.pi, 1.0, 0.0, math.pi / 2.0
    )
    assert x == pytest.approx(2.0)
    assert y == pytest.approx(2.0)
    assert heading == pytest.approx(math.pi / 2.0)


def test_soft_map_correction_limits_each_step():
    """A large valid innovation is applied gradually, never as a TF jump."""
    x, y, heading = soft_correct_map_to_odom(
        0.0, 0.0, 0.0, 1.0, 0.0, math.pi / 2.0, 0.5, 0.1, math.radians(10.0)
    )
    assert x == pytest.approx(0.1)
    assert y == pytest.approx(0.0)
    assert heading == pytest.approx(math.radians(10.0))


def test_soft_map_correction_handles_yaw_wraparound():
    """Correction takes the short path across the +/- pi boundary."""
    _, _, heading = soft_correct_map_to_odom(
        0.0,
        0.0,
        math.radians(179.0),
        0.0,
        0.0,
        math.radians(-179.0),
        0.5,
        0.1,
        math.radians(10.0),
    )
    assert abs(abs(math.degrees(heading)) - 180.0) < 1.0e-6


def test_map_correction_rejects_pose_jump():
    """A large AMCL innovation is rejected even with good covariance."""
    assert not map_correction_is_acceptable(
        0.0,
        0.0,
        0.0,
        2.0,
        0.0,
        0.0,
        0.01,
        0.01,
        0.01,
        0.5,
        math.radians(10.0),
        0.1,
        0.1,
    )


def test_map_correction_rejects_uncertain_pose():
    """An uncertain AMCL estimate cannot move the locked map transform."""
    assert not map_correction_is_acceptable(
        0.0,
        0.0,
        0.0,
        0.05,
        0.0,
        math.radians(1.0),
        0.20,
        0.01,
        0.01,
        0.5,
        math.radians(10.0),
        0.1,
        0.1,
    )
