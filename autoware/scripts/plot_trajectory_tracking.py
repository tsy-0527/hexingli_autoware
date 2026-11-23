#!/usr/bin/env python3

"""Plot tracking errors derived from a planning trajectory and odometry rosbag."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import json
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TRAJECTORY_TOPIC_CANDIDATES = (
    "/planning/trajectory",
    "/planning/scenario_planning/trajectory",
)
# pose_estimator publishes at a higher rate than the 20 Hz fused odometry in
# the usual configuration, so prefer it for a finer error trace.
ACTUAL_POSE_TOPIC_CANDIDATES = (
    "/localization/pose_estimator/pose_with_covariance",
    "/localization/kinematic_state",
)
ODOMETRY_TOPIC = "/localization/kinematic_state"
CONTROL_TOPIC_CANDIDATES = (
    "/control/trajectory_follower/control_cmd",
    "/control/command/control_cmd",
)
VELOCITY_STATUS_TOPIC = "/vehicle/status/velocity_status"
STEERING_STATUS_TOPIC = "/vehicle/status/steering_status"
BAG_LABELS = {
    "trajectory_eval_20260904_182747": "Point-cloud localization — 5 km/h",
    "trajectory_eval_20260904_183216": "Point-cloud localization — 8 km/h",
    "trajectory_eval_20260907_142829": "Integrated navigation — 8 km/h",
    "trajectory_eval_20260907_143216": "Integrated navigation — 5 km/h",
}


@dataclass
class PlannedTrajectory:
    timestamp: float
    path: np.ndarray
    headings: np.ndarray
    velocities: np.ndarray


@dataclass
class ActualPose:
    timestamp: float
    x: float
    y: float
    heading: float


@dataclass
class OdometrySample(ActualPose):
    velocity: float


@dataclass
class ScalarSample:
    timestamp: float
    value: float


@dataclass
class RecordedData:
    trajectories: list[PlannedTrajectory]
    poses: list[ActualPose]
    odometry: list[OdometrySample]
    velocity_commands: dict[str, list[ScalarSample]]
    steering_commands: dict[str, list[ScalarSample]]
    measured_velocity: list[ScalarSample]
    measured_steering: list[ScalarSample]
    trajectory_topic: str
    actual_topic: str


def open_reader(bag_path: Path) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def topic_types(bag_path: Path) -> dict[str, str]:
    reader = open_reader(bag_path)
    return {topic.name: topic.type for topic in reader.get_all_topics_and_types()}


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quaternion_to_yaw(quaternion) -> float:
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def message_timestamp(message, bag_time: int) -> float:
    """Use a message/header stamp, falling back to the rosbag receive time."""
    if hasattr(message, "header"):
        stamp = message.header.stamp
    elif hasattr(message, "stamp"):
        stamp = message.stamp
    else:
        return float(bag_time) * 1e-9
    timestamp = stamp_to_seconds(stamp)
    return timestamp if timestamp > 0.0 else float(bag_time) * 1e-9


def read_data(bag_path: Path) -> RecordedData:
    types = topic_types(bag_path)
    trajectory_topic = next(
        (topic for topic in TRAJECTORY_TOPIC_CANDIDATES if topic in types), None
    )
    actual_topic = next(
        (topic for topic in ACTUAL_POSE_TOPIC_CANDIDATES if topic in types), None
    )
    if trajectory_topic is None or actual_topic is None:
        raise RuntimeError(
            f"录包缺少必要 topic，需要 {TRAJECTORY_TOPIC_CANDIDATES} 和 "
            f"{ACTUAL_POSE_TOPIC_CANDIDATES}"
        )

    relevant_topics = {
        trajectory_topic,
        actual_topic,
        ODOMETRY_TOPIC,
        VELOCITY_STATUS_TOPIC,
        STEERING_STATUS_TOPIC,
        *CONTROL_TOPIC_CANDIDATES,
    }
    message_types = {
        topic: get_message(type_name)
        for topic, type_name in types.items()
        if topic in relevant_topics
    }
    trajectories: list[PlannedTrajectory] = []
    poses: list[ActualPose] = []
    odometry: list[OdometrySample] = []
    velocity_commands: dict[str, list[ScalarSample]] = {
        topic: [] for topic in CONTROL_TOPIC_CANDIDATES if topic in types
    }
    steering_commands: dict[str, list[ScalarSample]] = {
        topic: [] for topic in CONTROL_TOPIC_CANDIDATES if topic in types
    }
    measured_velocity: list[ScalarSample] = []
    measured_steering: list[ScalarSample] = []
    reader = open_reader(bag_path)

    while reader.has_next():
        current_topic, serialized, bag_time = reader.read_next()
        if current_topic not in message_types:
            continue
        message = deserialize_message(serialized, message_types[current_topic])
        timestamp = message_timestamp(message, bag_time)

        if current_topic == trajectory_topic:
            if len(message.points) < 2:
                continue
            path = np.asarray(
                [(point.pose.position.x, point.pose.position.y) for point in message.points],
                dtype=float,
            )
            headings = np.asarray(
                [quaternion_to_yaw(point.pose.orientation) for point in message.points],
                dtype=float,
            )
            velocities = np.asarray(
                [float(point.longitudinal_velocity_mps) for point in message.points],
                dtype=float,
            )
            trajectories.append(
                PlannedTrajectory(timestamp, path, headings, velocities)
            )
            continue

        if current_topic == actual_topic or current_topic == ODOMETRY_TOPIC:
            x = float(message.pose.pose.position.x)
            y = float(message.pose.pose.position.y)
            heading = quaternion_to_yaw(message.pose.pose.orientation)
            if current_topic == actual_topic:
                poses.append(ActualPose(timestamp, x, y, heading))
            if current_topic == ODOMETRY_TOPIC:
                odometry.append(
                    OdometrySample(
                        timestamp,
                        x,
                        y,
                        heading,
                        float(message.twist.twist.linear.x),
                    )
                )
            continue

        if current_topic in CONTROL_TOPIC_CANDIDATES:
            velocity_commands[current_topic].append(
                ScalarSample(timestamp, float(message.longitudinal.velocity))
            )
            steering_commands[current_topic].append(
                ScalarSample(timestamp, float(message.lateral.steering_tire_angle))
            )
        elif current_topic == VELOCITY_STATUS_TOPIC:
            measured_velocity.append(
                ScalarSample(timestamp, float(message.longitudinal_velocity))
            )
        elif current_topic == STEERING_STATUS_TOPIC:
            measured_steering.append(
                ScalarSample(timestamp, float(message.steering_tire_angle))
            )

    trajectories.sort(key=lambda item: item.timestamp)
    poses.sort(key=lambda item: item.timestamp)
    odometry.sort(key=lambda item: item.timestamp)
    measured_velocity.sort(key=lambda item: item.timestamp)
    measured_steering.sort(key=lambda item: item.timestamp)
    for sample_map in (velocity_commands, steering_commands):
        for samples in sample_map.values():
            samples.sort(key=lambda item: item.timestamp)
    if not trajectories or len(poses) < 2:
        raise RuntimeError("规划轨迹或实际定位数据为空。")
    return RecordedData(
        trajectories,
        poses,
        odometry,
        velocity_commands,
        steering_commands,
        measured_velocity,
        measured_steering,
        trajectory_topic,
        actual_topic,
    )


def wrap_angle(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def closest_point_error(
    actual_x: float,
    actual_y: float,
    trajectory: PlannedTrajectory,
) -> tuple[float, float, float]:
    """Return lateral error [m], reference heading [rad], and reference speed [m/s]."""
    path = trajectory.path
    point = np.asarray([actual_x, actual_y], dtype=float)
    segments = path[1:] - path[:-1]
    lengths_squared = np.einsum("ij,ij->i", segments, segments)
    safe_lengths_squared = np.where(lengths_squared > 1e-12, lengths_squared, 1.0)
    projection = np.einsum("ij,ij->i", point - path[:-1], segments)
    fraction = np.clip(projection / safe_lengths_squared, 0.0, 1.0)
    projected = path[:-1] + segments * fraction[:, None]
    distance_squared = np.einsum("ij,ij->i", point - projected, point - projected)
    segment_index = int(np.argmin(distance_squared))

    segment = segments[segment_index]
    segment_length = float(np.sqrt(lengths_squared[segment_index]))
    if segment_length <= 1e-6:
        if segment_index > 0:
            segment = segments[segment_index - 1]
            segment_length = float(np.linalg.norm(segment))
        if segment_length <= 1e-6:
            reference_heading = float(trajectory.headings[segment_index])
            tangent = np.asarray(
                [np.cos(reference_heading), np.sin(reference_heading)], dtype=float
            )
        else:
            tangent = segment / segment_length
    else:
        tangent = segment / segment_length

    error_vector = point - projected[segment_index]
    lateral_error = float(tangent[0] * error_vector[1] - tangent[1] * error_vector[0])

    # Interpolate the trajectory-point orientation along the closest segment.
    # Unwrapping avoids an artificial jump when the path crosses +/- pi.
    headings = np.unwrap(trajectory.headings)
    reference_heading = float(
        headings[segment_index]
        + fraction[segment_index]
        * (headings[segment_index + 1] - headings[segment_index])
    )
    reference_velocity = float(
        trajectory.velocities[segment_index]
        + fraction[segment_index]
        * (
            trajectory.velocities[segment_index + 1]
            - trajectory.velocities[segment_index]
        )
    )
    return lateral_error, wrap_angle(reference_heading), reference_velocity


def derive_errors(
    trajectories: list[PlannedTrajectory], poses: list[ActualPose]
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    planning_times = np.asarray([item.timestamp for item in trajectories])
    pose_times = np.asarray([item.timestamp for item in poses])
    lateral_errors: list[float] = []
    heading_errors: list[float] = []
    used_times: list[float] = []
    actual_xy: list[tuple[float, float]] = []

    for pose in poses:
        plan_index = int(np.searchsorted(planning_times, pose.timestamp, side="right") - 1)
        if plan_index < 0:
            plan_index = 0
        trajectory = trajectories[plan_index]
        lateral_error, reference_heading, _ = closest_point_error(
            pose.x, pose.y, trajectory
        )
        used_times.append(pose.timestamp)
        lateral_errors.append(lateral_error)
        heading_errors.append(wrap_angle(pose.heading - reference_heading))
        actual_xy.append((pose.x, pose.y))

    times = np.asarray(used_times)
    times -= times[0]
    return (
        times,
        np.asarray(lateral_errors),
        np.asarray(heading_errors),
        np.asarray(actual_xy),
        pose_times,
    )


def derive_velocity_errors(
    trajectories: list[PlannedTrajectory],
    odometry: list[OdometrySample],
    time_origin: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Compare odometry speed with the spatially corresponding trajectory speed."""
    if not odometry:
        empty = np.asarray([], dtype=float)
        return empty, empty, empty, empty

    planning_times = np.asarray([item.timestamp for item in trajectories])
    times: list[float] = []
    reference: list[float] = []
    actual: list[float] = []
    for sample in odometry:
        plan_index = int(np.searchsorted(planning_times, sample.timestamp, side="right") - 1)
        if plan_index < 0:
            continue
        _, _, reference_velocity = closest_point_error(
            sample.x, sample.y, trajectories[plan_index]
        )
        times.append(sample.timestamp - time_origin)
        reference.append(reference_velocity)
        actual.append(sample.velocity)

    reference_array = np.asarray(reference, dtype=float)
    actual_array = np.asarray(actual, dtype=float)
    return (
        np.asarray(times, dtype=float),
        reference_array,
        actual_array,
        actual_array - reference_array,
    )


def calculate_stats(values: np.ndarray, threshold: float | None = None) -> dict[str, float]:
    finite = values[np.isfinite(values)]
    if len(finite) == 0:
        return {}
    absolute = np.abs(finite)
    result = {
        "count": int(len(finite)),
        "min": float(np.min(finite)),
        "max": float(np.max(finite)),
        "mean": float(np.mean(finite)),
        "std": float(np.std(finite)),
        "mae": float(np.mean(absolute)),
        "rmse": float(np.sqrt(np.mean(finite**2))),
        "p95_abs": float(np.percentile(absolute, 95.0)),
        "max_abs": float(np.max(absolute)),
    }
    if threshold is not None:
        result["threshold"] = float(threshold)
        result["exceedance_percent"] = float(np.mean(absolute > threshold) * 100.0)
    return result


def print_stats(
    name: str,
    values: np.ndarray,
    unit: str,
    threshold: float | None = None,
) -> dict[str, float]:
    stats = calculate_stats(values, threshold)
    if not stats:
        print(f"{name}: 无有效样本")
        return stats
    text = (
        f"{name}: mean={stats['mean']:.4f}, MAE={stats['mae']:.4f}, "
        f"RMSE={stats['rmse']:.4f}, P95(abs)={stats['p95_abs']:.4f}, "
        f"max(abs)={stats['max_abs']:.4f} {unit}"
    )
    if threshold is not None:
        text += (
            f"，|error|>{threshold:g} {unit} 占比="
            f"{stats['exceedance_percent']:.2f}%"
        )
    print(text)
    return stats


def samples_to_arrays(
    samples: list[ScalarSample], time_origin: float
) -> tuple[np.ndarray, np.ndarray]:
    return (
        np.asarray([sample.timestamp - time_origin for sample in samples], dtype=float),
        np.asarray([sample.value for sample in samples], dtype=float),
    )


def aligned_tracking_error(
    commands: list[ScalarSample],
    measurements: list[ScalarSample],
    time_origin: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Interpolate commands at feedback timestamps and return actual-command error."""
    if len(commands) < 2 or not measurements:
        empty = np.asarray([], dtype=float)
        return empty, empty, empty, empty
    command_times, command_values = samples_to_arrays(commands, time_origin)
    measured_times, measured_values = samples_to_arrays(measurements, time_origin)
    unique_times, unique_indices = np.unique(command_times, return_index=True)
    valid = (measured_times >= unique_times[0]) & (measured_times <= unique_times[-1])
    measured_times = measured_times[valid]
    measured_values = measured_values[valid]
    if len(measured_times) == 0:
        empty = np.asarray([], dtype=float)
        return empty, empty, empty, empty
    desired = np.interp(
        measured_times, unique_times, command_values[unique_indices]
    )
    return measured_times, desired, measured_values, measured_values - desired


def resample_errors(
    times: np.ndarray,
    lateral: np.ndarray,
    heading: np.ndarray,
    sample_hz: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Resample errors onto a regular time grid for a finer-looking plot.

    This only changes the display samples; error statistics continue to use the
    original localization samples.  Heading is unwrapped before interpolation
    so a crossing of +/- pi does not create a false spike.
    """
    if sample_hz <= 0.0:
        raise ValueError("绘图采样频率必须大于 0 Hz。")
    if len(times) < 2 or times[-1] <= times[0]:
        return times, lateral, heading

    # A bag can contain repeated timestamps.  np.interp requires an increasing
    # x-axis, so retain the first value at each timestamp.
    unique_times, unique_indices = np.unique(times, return_index=True)
    if len(unique_times) < 2:
        return times, lateral, heading

    step = 1.0 / sample_hz
    plot_times = np.arange(unique_times[0], unique_times[-1], step, dtype=float)
    if len(plot_times) == 0 or plot_times[-1] < unique_times[-1]:
        plot_times = np.append(plot_times, unique_times[-1])

    plot_lateral = np.interp(
        plot_times, unique_times, lateral[unique_indices]
    )
    unwrapped_heading = np.unwrap(heading[unique_indices])
    plot_heading = np.asarray(
        [wrap_angle(value) for value in np.interp(
            plot_times, unique_times, unwrapped_heading
        )],
        dtype=float,
    )
    return plot_times, plot_lateral, plot_heading


def plot_errors(
    bag_path: Path,
    trajectory_topic: str,
    actual_topic: str,
    times: np.ndarray,
    lateral: np.ndarray,
    heading: np.ndarray,
    output_path: Path,
    condition_label: str = "",
    title_suffix: str = "",
    x_label: str = "Time from first localization message [s]",
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    heading_degrees = np.rad2deg(heading)
    figure, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    label_suffix = f" — {condition_label}" if condition_label else ""
    figure.suptitle(
        f"Trajectory Tracking Errors{label_suffix}{title_suffix}\n{bag_path.name}"
    )

    axes[0].plot(times, lateral, color="#1f77b4", linewidth=1.0)
    axes[0].axhline(0.0, color="black", linewidth=0.7)
    axes[0].set_ylabel("Lateral error [m]\n(+ left of path)")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(times, heading_degrees, color="#d62728", linewidth=1.0)
    axes[1].axhline(0.0, color="black", linewidth=0.7)
    axes[1].set_ylabel("Heading error [deg]")
    axes[1].set_xlabel(x_label)
    axes[1].grid(True, alpha=0.3)

    figure.text(
        0.99,
        0.01,
        f"planned: {trajectory_topic} | actual: {actual_topic}",
        ha="right",
        va="bottom",
        fontsize=8,
        color="dimgray",
    )
    figure.tight_layout(rect=(0, 0.03, 1, 0.95))
    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def plot_actual_path(
    bag_path: Path,
    trajectories: list[PlannedTrajectory],
    actual_xy: np.ndarray,
    trajectory_topic: str,
    actual_topic: str,
    output_path: Path,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure, axis = plt.subplots(figsize=(11, 8))

    step = max(1, len(trajectories) // 60)
    for trajectory in trajectories[::step]:
        axis.plot(
            trajectory.path[:, 0],
            trajectory.path[:, 1],
            color="#4c9bd1",
            alpha=0.12,
            linewidth=0.7,
        )
    latest = trajectories[-1].path
    axis.plot(
        latest[:, 0],
        latest[:, 1],
        color="#1769aa",
        linewidth=2.0,
        label="Latest planned trajectory",
    )
    axis.plot(
        actual_xy[:, 0],
        actual_xy[:, 1],
        color="#d62728",
        linewidth=1.8,
        label="Actual vehicle path",
    )
    axis.scatter(*actual_xy[0], color="#2ca02c", s=45, zorder=5, label="Actual start")
    axis.scatter(*actual_xy[-1], color="#9467bd", s=45, zorder=5, label="Actual end")
    axis.set_title(f"Driving Trajectory\n{bag_path.name}")
    axis.set_xlabel("X [m]")
    axis.set_ylabel("Y [m]")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, alpha=0.3)
    axis.legend()
    figure.text(
        0.01,
        0.01,
        f"planned: {trajectory_topic} | actual: {actual_topic}",
        ha="left",
        va="bottom",
        fontsize=8,
        color="dimgray",
    )
    figure.tight_layout(rect=(0, 0.03, 1, 1))
    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def plot_control_tracking(
    bag_path: Path,
    data: RecordedData,
    time_origin: float,
    trajectory_velocity: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray],
    output_path: Path,
) -> bool:
    """Plot planned/commanded signals against odometry and chassis feedback."""
    velocity_times, reference_velocity, actual_velocity, _ = trajectory_velocity
    has_control_data = bool(
        len(velocity_times)
        or data.measured_velocity
        or data.measured_steering
        or data.velocity_commands
        or data.steering_commands
    )
    if not has_control_data:
        return False

    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
    figure.suptitle(f"Longitudinal and Steering Tracking\n{bag_path.name}")

    if len(velocity_times):
        axes[0].plot(
            velocity_times, reference_velocity, label="Trajectory reference", linewidth=1.0
        )
        axes[0].plot(
            velocity_times, actual_velocity, label="Odometry actual", linewidth=1.0
        )
    else:
        axes[0].text(0.5, 0.5, "No odometry speed", ha="center", transform=axes[0].transAxes)
    axes[0].set_ylabel("Velocity [m/s]")
    axes[0].set_title("Trajectory speed tracking at the closest path point")

    command_labels = {
        "/control/trajectory_follower/control_cmd": "Controller command",
        "/control/command/control_cmd": "Final chassis command",
    }
    for topic, samples in data.velocity_commands.items():
        times, values = samples_to_arrays(samples, time_origin)
        axes[1].plot(times, values, linewidth=0.9, label=command_labels.get(topic, topic))
    if data.measured_velocity:
        times, values = samples_to_arrays(data.measured_velocity, time_origin)
        axes[1].plot(times, values, linewidth=1.0, label="Vehicle feedback")
    axes[1].set_ylabel("Velocity [m/s]")
    axes[1].set_title("Commanded speed vs vehicle feedback")

    for topic, samples in data.steering_commands.items():
        times, values = samples_to_arrays(samples, time_origin)
        axes[2].plot(
            times,
            np.rad2deg(values),
            linewidth=0.9,
            label=command_labels.get(topic, topic),
        )
    if data.measured_steering:
        times, values = samples_to_arrays(data.measured_steering, time_origin)
        axes[2].plot(times, np.rad2deg(values), linewidth=1.0, label="Vehicle feedback")
    axes[2].set_ylabel("Steering tire angle [deg]")
    axes[2].set_xlabel("Time from first localization message [s]")
    axes[2].set_title("Commanded steering vs vehicle feedback")

    for axis in axes:
        axis.grid(True, alpha=0.3)
        if axis.lines:
            axis.legend(loc="best")
    figure.tight_layout(rect=(0, 0, 1, 0.96))
    figure.savefig(output_path, dpi=160)
    plt.close(figure)
    return True


def write_tracking_csv(
    output_path: Path,
    times: np.ndarray,
    lateral: np.ndarray,
    heading: np.ndarray,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("time_s", "lateral_error_m", "heading_error_rad", "heading_error_deg"))
        writer.writerows(
            zip(times, lateral, heading, np.rad2deg(heading), strict=True)
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="rosbag 目录")
    parser.add_argument(
        "-o",
        "--output-dir",
        type=Path,
        default=Path("plots"),
        help="PNG 输出目录（默认：./plots）",
    )
    parser.add_argument(
        "--exclude-start",
        type=float,
        help="剔除误差时间段的起始时间（相对第一条定位消息，单位：s）",
    )
    parser.add_argument(
        "--exclude-end",
        type=float,
        help="剔除误差时间段的结束时间（相对第一条定位消息，单位：s）",
    )
    parser.add_argument(
        "--plot-hz",
        type=float,
        default=100.0,
        help=(
            "误差图的等时间间隔采样频率（默认：100 Hz）；"
            "只影响绘图，不改变录包数据和统计值"
        ),
    )
    parser.add_argument(
        "--errors-only",
        action="store_true",
        help="只生成横向/航向误差图，不生成轨迹图和控制跟踪图",
    )
    parser.add_argument(
        "--lateral-threshold",
        type=float,
        default=0.20,
        help="横向误差统计阈值，单位 m（默认：0.20）",
    )
    parser.add_argument(
        "--heading-threshold-deg",
        type=float,
        default=5.0,
        help="航向误差统计阈值，单位 deg（默认：5.0）",
    )
    args = parser.parse_args()

    bag_path = Path(args.bag).expanduser().resolve()
    if not (bag_path / "metadata.yaml").is_file():
        print(f"错误：找不到 rosbag metadata.yaml：{bag_path}", file=sys.stderr)
        return 1
    if args.lateral_threshold <= 0.0 or args.heading_threshold_deg <= 0.0:
        print("错误：误差统计阈值必须大于 0。", file=sys.stderr)
        return 1

    try:
        data = read_data(bag_path)
        times, lateral, heading, actual_xy, _ = derive_errors(
            data.trajectories, data.poses
        )
        time_origin = data.poses[0].timestamp
        trajectory_velocity = derive_velocity_errors(
            data.trajectories, data.odometry, time_origin
        )
    except (RuntimeError, ValueError, AttributeError) as error:
        print(f"错误：{error}", file=sys.stderr)
        return 1

    if (args.exclude_start is None) != (args.exclude_end is None):
        print("错误：--exclude-start 和 --exclude-end 必须同时指定。", file=sys.stderr)
        return 1

    title_suffix = ""
    if args.exclude_start is not None and args.exclude_end is not None:
        if args.exclude_start < 0.0 or args.exclude_end <= args.exclude_start:
            print("错误：剔除时间段必须满足 0 <= start < end。", file=sys.stderr)
            return 1
        keep = (times < args.exclude_start) | (times > args.exclude_end)
        removed_count = int(np.count_nonzero(~keep))
        if removed_count == 0:
            print("错误：指定的剔除时间段内没有误差样本。", file=sys.stderr)
            return 1
        times = times[keep].copy()
        lateral = lateral[keep]
        heading = heading[keep]
        # Compress the time axis so the removed manual-driving interval is absent
        # instead of leaving a large blank gap in the plot.
        times[times > args.exclude_end] -= args.exclude_end - args.exclude_start
        title_suffix = (
            f" (manual segment {args.exclude_start:.1f}–{args.exclude_end:.1f} s removed)"
        )
        print(
            f"剔除手动驾驶段：{args.exclude_start:.3f}–{args.exclude_end:.3f} s，"
            f"{removed_count} 个样本"
        )

    try:
        plot_times, plot_lateral, plot_heading = resample_errors(
            times, lateral, heading, args.plot_hz
        )
    except ValueError as error:
        print(f"错误：{error}", file=sys.stderr)
        return 1

    output_dir = args.output_dir.expanduser().resolve()
    output_name = (
        f"{bag_path.name}_tracking_errors_manual_removed.png"
        if args.exclude_start is not None
        else f"{bag_path.name}_tracking_errors_derived.png"
    )
    error_path = output_dir / output_name
    trajectory_path = output_dir / f"{bag_path.name}_driving_trajectory.png"
    control_path = output_dir / f"{bag_path.name}_control_tracking.png"
    csv_path = output_dir / f"{bag_path.name}_tracking_samples.csv"
    summary_path = output_dir / f"{bag_path.name}_tracking_summary.json"
    plot_errors(
        bag_path,
        data.trajectory_topic,
        data.actual_topic,
        plot_times,
        plot_lateral,
        plot_heading,
        error_path,
        BAG_LABELS.get(bag_path.name, ""),
        f"{title_suffix} (display resampled at {args.plot_hz:g} Hz)",
    )
    if not args.errors_only:
        plot_actual_path(
            bag_path,
            data.trajectories,
            actual_xy,
            data.trajectory_topic,
            data.actual_topic,
            trajectory_path,
        )
        control_plot_created = plot_control_tracking(
            bag_path, data, time_origin, trajectory_velocity, control_path
        )
    else:
        control_plot_created = False

    write_tracking_csv(csv_path, times, lateral, heading)

    heading_degrees = np.rad2deg(heading)
    metrics: dict[str, dict[str, float]] = {
        "lateral_error_m": calculate_stats(lateral, args.lateral_threshold),
        "heading_error_deg": calculate_stats(
            heading_degrees, args.heading_threshold_deg
        ),
    }
    velocity_times, reference_velocity, actual_velocity, velocity_error = trajectory_velocity
    if len(velocity_error):
        metrics["trajectory_velocity_error_mps"] = calculate_stats(velocity_error)

    final_topic = "/control/command/control_cmd"
    controller_topic = "/control/trajectory_follower/control_cmd"
    selected_topic = (
        final_topic
        if data.velocity_commands.get(final_topic)
        else controller_topic
        if data.velocity_commands.get(controller_topic)
        else None
    )
    if selected_topic is not None:
        _, _, _, command_velocity_error = aligned_tracking_error(
            data.velocity_commands[selected_topic], data.measured_velocity, time_origin
        )
        _, _, _, command_steering_error = aligned_tracking_error(
            data.steering_commands[selected_topic], data.measured_steering, time_origin
        )
        if len(command_velocity_error):
            metrics["command_velocity_feedback_error_mps"] = calculate_stats(
                command_velocity_error
            )
        if len(command_steering_error):
            metrics["command_steering_feedback_error_deg"] = calculate_stats(
                np.rad2deg(command_steering_error)
            )

    if data.velocity_commands.get(controller_topic) and data.velocity_commands.get(final_topic):
        _, _, _, gate_velocity_difference = aligned_tracking_error(
            data.velocity_commands[controller_topic],
            data.velocity_commands[final_topic],
            time_origin,
        )
        _, _, _, gate_steering_difference = aligned_tracking_error(
            data.steering_commands[controller_topic],
            data.steering_commands[final_topic],
            time_origin,
        )
        metrics["gate_velocity_difference_mps"] = calculate_stats(
            gate_velocity_difference
        )
        metrics["gate_steering_difference_deg"] = calculate_stats(
            np.rad2deg(gate_steering_difference)
        )

    summary = {
        "bag": str(bag_path),
        "trajectory_topic": data.trajectory_topic,
        "actual_pose_topic": data.actual_topic,
        "control_feedback_reference_topic": selected_topic,
        "duration_s": float(times[-1]),
        "metrics": metrics,
        "sign_convention": {
            "lateral_error": "positive means vehicle is left of the path",
            "tracking_error": "actual minus reference/command",
        },
    }
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    print(f"录包：{bag_path}")
    print(f"规划轨迹：{data.trajectory_topic}，{len(data.trajectories)} 条")
    print(f"定位轨迹：{data.actual_topic}，{len(data.poses)} 个点")
    print(f"时长：{times[-1]:.3f} s")
    print(f"误差图采样：{len(plot_times)} 个显示点，{args.plot_hz:g} Hz（原始定位样本不变）")
    print_stats("横向误差", lateral, "m", args.lateral_threshold)
    print_stats(
        "航向误差", heading_degrees, "deg", args.heading_threshold_deg
    )
    if len(velocity_error):
        print_stats("轨迹速度误差（实际-规划）", velocity_error, "m/s")
    if "command_velocity_feedback_error_mps" in metrics:
        command_error = metrics["command_velocity_feedback_error_mps"]
        print(
            "底盘速度响应误差（反馈-命令）："
            f"MAE={command_error['mae']:.4f}, RMSE={command_error['rmse']:.4f}, "
            f"P95(abs)={command_error['p95_abs']:.4f} m/s"
        )
    if "command_steering_feedback_error_deg" in metrics:
        command_error = metrics["command_steering_feedback_error_deg"]
        print(
            "底盘转角响应误差（反馈-命令）："
            f"MAE={command_error['mae']:.4f}, RMSE={command_error['rmse']:.4f}, "
            f"P95(abs)={command_error['p95_abs']:.4f} deg"
        )
    print(f"误差图：{error_path}")
    print(f"逐样本数据：{csv_path}")
    print(f"统计摘要：{summary_path}")
    if not args.errors_only:
        print(f"行驶轨迹图：{trajectory_path}")
        if control_plot_created:
            print(f"速度/转角跟踪图：{control_path}")
        else:
            print("提示：录包中没有控制或底盘反馈数据，未生成控制跟踪图。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
