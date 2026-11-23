#!/usr/bin/env python3

"""Plot planned trajectories and the actual vehicle path from a ROS 2 bag."""

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


TRAJECTORY_TOPIC_CANDIDATES = (
    "/planning/trajectory",
    "/planning/scenario_planning/trajectory",
)
ODOMETRY_TOPIC = "/localization/kinematic_state"
DEFAULT_SEARCH_ROOTS = (
    Path.home() / "tsy" / "rosbags",
    Path.home() / "autoware_bag",
)


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


def find_bag(requested: str | None) -> Path:
    if requested:
        bag_path = Path(requested).expanduser().resolve()
        if not (bag_path / "metadata.yaml").is_file():
            raise FileNotFoundError(f"找不到 rosbag：{bag_path}")
        return bag_path

    candidates: list[Path] = []
    for root in DEFAULT_SEARCH_ROOTS:
        if root.is_dir():
            candidates.extend(path.parent for path in root.rglob("metadata.yaml"))

    for bag_path in sorted(set(candidates), key=lambda path: path.stat().st_mtime, reverse=True):
        types = topic_types(bag_path)
        if ODOMETRY_TOPIC in types and any(topic in types for topic in TRAJECTORY_TOPIC_CANDIDATES):
            return bag_path

    raise RuntimeError("没有找到同时包含规划轨迹和定位轨迹的 rosbag。")


def read_paths(
    bag_path: Path,
) -> tuple[list[np.ndarray], np.ndarray, str, str]:
    reader = open_reader(bag_path)
    types = topic_types(bag_path)
    trajectory_topic = next(
        (topic for topic in TRAJECTORY_TOPIC_CANDIDATES if topic in types), None
    )
    if trajectory_topic is None or ODOMETRY_TOPIC not in types:
        raise RuntimeError(
            f"录包缺少必要 topic，需要 {TRAJECTORY_TOPIC_CANDIDATES} 和 {ODOMETRY_TOPIC}"
        )

    trajectory_type = get_message(types[trajectory_topic])
    odometry_type = get_message(types[ODOMETRY_TOPIC])
    planned_paths: list[np.ndarray] = []
    actual_path: list[tuple[float, float]] = []

    while reader.has_next():
        current_topic, serialized, _ = reader.read_next()
        if current_topic == trajectory_topic:
            message = deserialize_message(serialized, trajectory_type)
            path = np.asarray(
                [(point.pose.position.x, point.pose.position.y) for point in message.points],
                dtype=float,
            )
            if len(path) >= 2:
                planned_paths.append(path)
        elif current_topic == ODOMETRY_TOPIC:
            message = deserialize_message(serialized, odometry_type)
            actual_path.append(
                (message.pose.pose.position.x, message.pose.pose.position.y)
            )

    if not planned_paths or len(actual_path) < 2:
        raise RuntimeError("规划轨迹或实际定位数据为空。")
    return planned_paths, np.asarray(actual_path), trajectory_topic, types[ODOMETRY_TOPIC]


def plot_paths(
    bag_path: Path,
    planned_paths: list[np.ndarray],
    actual_path: np.ndarray,
    trajectory_topic: str,
    output_path: Path,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure, axis = plt.subplots(figsize=(11, 8))

    # 规划器会连续发布相似的未来轨迹。淡色显示轨迹变化，粗线显示最后一条规划轨迹。
    step = max(1, len(planned_paths) // 60)
    for path in planned_paths[::step]:
        axis.plot(path[:, 0], path[:, 1], color="#4c9bd1", alpha=0.12, linewidth=0.7)
    latest = planned_paths[-1]
    axis.plot(
        latest[:, 0], latest[:, 1], color="#1769aa", linewidth=2.0,
        label="Latest planned trajectory",
    )

    axis.plot(
        actual_path[:, 0], actual_path[:, 1], color="#d62728", linewidth=1.8,
        label="Actual vehicle path",
    )
    axis.scatter(*actual_path[0], color="#2ca02c", s=45, zorder=5, label="Actual start")
    axis.scatter(*actual_path[-1], color="#9467bd", s=45, zorder=5, label="Actual end")

    axis.set_title(f"Planning Path vs Actual Path\n{bag_path.name}")
    axis.set_xlabel("X [m]")
    axis.set_ylabel("Y [m]")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, alpha=0.3)
    axis.legend()
    figure.text(0.01, 0.01, f"planned: {trajectory_topic} | actual: {ODOMETRY_TOPIC}",
                ha="left", va="bottom", fontsize=8, color="dimgray")
    figure.tight_layout(rect=(0, 0.03, 1, 1))
    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "bag", nargs="?", help="rosbag 目录；不填写时自动选择最近的路径包"
    )
    parser.add_argument("-o", "--output", type=Path, help="PNG 输出路径")
    args = parser.parse_args()

    try:
        bag_path = find_bag(args.bag)
        planned_paths, actual_path, trajectory_topic, _ = read_paths(bag_path)
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        print(f"错误：{error}", file=sys.stderr)
        return 1

    output_path = args.output or (
        Path(__file__).resolve().parents[1] / "plots" / f"{bag_path.name}_planning_vs_actual.png"
    )
    plot_paths(bag_path, planned_paths, actual_path, trajectory_topic, output_path)

    print(f"录包：{bag_path}")
    print(f"规划轨迹：{trajectory_topic}，共 {len(planned_paths)} 条")
    print(f"实际轨迹：{ODOMETRY_TOPIC}，共 {len(actual_path)} 个定位点")
    print(f"图片：{output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

