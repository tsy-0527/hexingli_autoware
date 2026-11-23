#!/usr/bin/env python3

"""Plot lateral and heading tracking errors from a ROS 2 bag."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


ERROR_TOPIC = "/control_performance/performance_vars"
DIAGNOSTIC_TOPIC = "/control/trajectory_follower/lateral/diagnostic"
ERROR_TYPE = "autoware_control_performance_analysis/msg/ErrorStamped"
DIAGNOSTIC_TYPE = "autoware_internal_debug_msgs/msg/Float32MultiArrayStamped"
DEFAULT_SEARCH_ROOTS = (
    Path.home() / "autoware_bag",
    Path.home() / "tsy" / "rosbags",
)


def topic_types(bag_path: Path) -> dict[str, str]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    return {topic.name: topic.type for topic in reader.get_all_topics_and_types()}


def bag_candidates() -> list[Path]:
    candidates: list[Path] = []
    for root in DEFAULT_SEARCH_ROOTS:
        if root.is_dir():
            candidates.extend(path.parent for path in root.rglob("metadata.yaml"))
    return sorted(set(candidates), key=lambda path: path.stat().st_mtime, reverse=True)


def select_bag(requested: str | None) -> tuple[Path, str, str]:
    if requested:
        bag_path = Path(requested).expanduser().resolve()
        if not (bag_path / "metadata.yaml").is_file():
            raise FileNotFoundError(f"找不到 rosbag metadata.yaml：{bag_path}")
        types = topic_types(bag_path)
        if ERROR_TOPIC in types:
            return bag_path, ERROR_TOPIC, types[ERROR_TOPIC]
        if DIAGNOSTIC_TOPIC in types:
            return bag_path, DIAGNOSTIC_TOPIC, types[DIAGNOSTIC_TOPIC]
        raise RuntimeError(
            f"录包中没有目标 topic：{bag_path}\n"
            f"需要 {ERROR_TOPIC} 或 {DIAGNOSTIC_TOPIC}"
        )

    for bag_path in bag_candidates():
        types = topic_types(bag_path)
        if ERROR_TOPIC in types:
            return bag_path, ERROR_TOPIC, types[ERROR_TOPIC]
        if DIAGNOSTIC_TOPIC in types:
            return bag_path, DIAGNOSTIC_TOPIC, types[DIAGNOSTIC_TOPIC]

    raise RuntimeError(
        "没有找到包含误差 topic 的 rosbag。\n"
        f"需要 {ERROR_TOPIC} 或 {DIAGNOSTIC_TOPIC}"
    )


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def read_errors(bag_path: Path, topic: str, type_name: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    message_type = get_message(type_name)

    times: list[float] = []
    lateral: list[float] = []
    heading: list[float] = []

    while reader.has_next():
        current_topic, serialized, bag_time = reader.read_next()
        if current_topic != topic:
            continue

        message = deserialize_message(serialized, message_type)
        if topic == ERROR_TOPIC:
            stamp = message.header.stamp
            lateral_value = message.error.lateral_error
            heading_value = message.error.heading_error
        else:
            stamp = message.stamp
            if len(message.data) <= 8:
                continue
            lateral_value = message.data[5]
            heading_value = message.data[8]

        timestamp = stamp_to_seconds(stamp)
        if timestamp <= 0.0:
            timestamp = float(bag_time) * 1e-9
        times.append(timestamp)
        lateral.append(float(lateral_value))
        heading.append(float(heading_value))

    if not times:
        raise RuntimeError(f"topic 存在但没有可读取的数据：{topic}")

    time_array = np.asarray(times)
    time_array -= time_array[0]
    return time_array, np.asarray(lateral), np.asarray(heading)


def print_stats(name: str, values: np.ndarray, unit: str) -> None:
    print(
        f"{name}: min={values.min():.4f}, max={values.max():.4f}, "
        f"mean={values.mean():.4f}, RMS={np.sqrt(np.mean(values**2)):.4f} {unit}"
    )


def plot_errors(
    bag_path: Path,
    topic: str,
    times: np.ndarray,
    lateral: np.ndarray,
    heading: np.ndarray,
    output_path: Path,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)

    figure, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    figure.suptitle(f"Autoware Tracking Errors\n{bag_path.name}")

    axes[0].plot(times, lateral, color="#1f77b4", linewidth=1.0)
    axes[0].axhline(0.0, color="black", linewidth=0.7)
    axes[0].set_ylabel("Lateral error [m]")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(times, heading, color="#d62728", linewidth=1.0)
    axes[1].axhline(0.0, color="black", linewidth=0.7)
    axes[1].set_ylabel("Heading error [rad]")
    axes[1].set_xlabel("Time from first message [s]")
    axes[1].grid(True, alpha=0.3)

    figure.text(0.99, 0.01, topic, ha="right", va="bottom", fontsize=8, color="dimgray")
    figure.tight_layout(rect=(0, 0.03, 1, 0.95))
    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "bag",
        nargs="?",
        help="rosbag 目录；不填写时自动选择最近一个包含误差 topic 的包",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="PNG 输出路径；默认保存到 plots/ 目录",
    )
    args = parser.parse_args()

    try:
        bag_path, topic, type_name = select_bag(args.bag)
        times, lateral, heading = read_errors(bag_path, topic, type_name)
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        print(f"错误：{error}", file=sys.stderr)
        return 1

    output_path = args.output or (
        Path(__file__).resolve().parents[1] / "plots" / f"{bag_path.name}_tracking_errors.png"
    )
    plot_errors(bag_path, topic, times, lateral, heading, output_path)

    print(f"录包：{bag_path}")
    print(f"topic：{topic}")
    print(f"样本数：{len(times)}，时长：{times[-1]:.3f} s")
    print_stats("横向误差", lateral, "m")
    print_stats("航向误差", heading, "rad")
    print(f"图片：{output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

