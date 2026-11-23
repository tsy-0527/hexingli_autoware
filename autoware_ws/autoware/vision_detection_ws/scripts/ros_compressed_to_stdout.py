#!/usr/bin/env python3

import argparse
import os
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


class LatestFramePipe:
    def __init__(self, repeat_fps: float = 0.0) -> None:
        self._condition = threading.Condition()
        self._frame = None
        self._stopped = False
        self._warned_format = False
        self._repeat_interval = 1.0 / repeat_fps if repeat_fps > 0.0 else None

    def submit_jpeg(self, frame: bytes) -> None:
        if len(frame) < 2 or frame[:2] != b"\xff\xd8":
            if not self._warned_format:
                print(
                    "Ignoring a frame that is not JPEG encoded",
                    file=sys.stderr,
                    flush=True,
                )
                self._warned_format = True
            return

        with self._condition:
            self._frame = frame
            self._condition.notify()

    def submit_compressed(self, message: CompressedImage) -> None:
        self.submit_jpeg(bytes(message.data))

    def stop(self) -> None:
        with self._condition:
            self._stopped = True
            self._condition.notify_all()

    def write(self) -> None:
        output_fd = sys.stdout.fileno()
        announced = False
        last_frame = None

        while True:
            with self._condition:
                if self._repeat_interval is None or last_frame is None:
                    self._condition.wait_for(
                        lambda: self._frame is not None or self._stopped
                    )
                else:
                    self._condition.wait(timeout=self._repeat_interval)
                if self._stopped:
                    return
                if self._frame is not None:
                    last_frame = self._frame
                    self._frame = None
                frame = last_frame

            if self._repeat_interval is None:
                last_frame = None

            if not announced:
                print("First JPEG frame received; starting encoder", file=sys.stderr, flush=True)
                announced = True

            try:
                remaining = memoryview(frame)
                while remaining:
                    written = os.write(output_fd, remaining)
                    remaining = remaining[written:]
            except BrokenPipeError:
                self.stop()
                return


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Write the newest ROS CompressedImage JPEG frame to stdout"
    )
    parser.add_argument(
        "--topic",
        default="/camera/color/image_raw/compressed",
        help="sensor_msgs/msg/CompressedImage topic",
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        help="subscribe to sensor_msgs/msg/Image and encode BGR8/RGB8 frames as JPEG",
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=80,
        choices=range(1, 101),
        metavar="1..100",
        help="JPEG quality used with --raw (default: 80)",
    )
    parser.add_argument(
        "--repeat-fps",
        type=float,
        default=0.0,
        help="repeat the latest JPEG at this rate; 0 writes only new frames",
    )
    args, ros_args = parser.parse_known_args()
    if args.repeat_fps < 0.0:
        parser.error("--repeat-fps must be non-negative")

    rclpy.init(args=ros_args)
    node = Node("camera_jpeg_pipe")
    frame_pipe = LatestFramePipe(args.repeat_fps)
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    if args.raw:
        import cv2
        import numpy as np

        warned_raw = False

        def submit_raw(message: Image) -> None:
            nonlocal warned_raw
            try:
                if message.encoding not in ("bgr8", "rgb8"):
                    raise ValueError(f"unsupported encoding {message.encoding!r}")
                if message.height < 1 or message.width < 1 or message.step < message.width * 3:
                    raise ValueError("invalid image dimensions or row stride")
                if len(message.data) != message.height * message.step:
                    raise ValueError("image data length does not match height * step")
                image = np.ndarray(
                    (message.height, message.width, 3),
                    dtype=np.uint8,
                    buffer=message.data,
                    strides=(message.step, 3, 1),
                )
                if message.encoding == "rgb8":
                    image = image[:, :, ::-1]
                image = np.ascontiguousarray(image)
                ok, jpeg = cv2.imencode(
                    ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality]
                )
                if not ok:
                    raise ValueError("OpenCV JPEG encoding failed")
                frame_pipe.submit_jpeg(jpeg.tobytes())
            except Exception as exc:
                if not warned_raw:
                    print(f"Ignoring invalid raw image: {exc}", file=sys.stderr, flush=True)
                    warned_raw = True

        node.create_subscription(Image, args.topic, submit_raw, qos)
        topic_kind = "raw image"
    else:
        node.create_subscription(CompressedImage, args.topic, frame_pipe.submit_compressed, qos)
        topic_kind = "JPEG"

    writer = threading.Thread(target=frame_pipe.write, name="jpeg-writer", daemon=True)
    writer.start()
    repeat_note = f"; holding latest at {args.repeat_fps:g} fps" if args.repeat_fps else ""
    print(
        f"Waiting for {topic_kind} frames on {args.topic}{repeat_note}",
        file=sys.stderr,
        flush=True,
    )

    try:
        while rclpy.ok() and not frame_pipe._stopped:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        frame_pipe.stop()
        writer.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
