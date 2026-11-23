#!/usr/bin/env python3
"""Run the installed D455 detector with field-friendly RGB-D pairing."""

import importlib.util
from importlib.machinery import SourceFileLoader
import hashlib
import math
import os
from pathlib import Path
import sys
import time


def main() -> int:
    workspace = Path(os.environ.get("D455_WS", "/home/tang/tsy/D455_ROS2_Humble"))
    detector_entry = workspace / "install/d455_perception/lib/d455_perception/detect"
    if not detector_entry.is_file():
        raise FileNotFoundError(f"Installed D455 detector not found: {detector_entry}")

    sync_mode = os.environ.get("D455_SYNC_MODE", "latest")
    sync_slop = float(os.environ.get("D455_SYNC_SLOP", "0.1"))
    if not math.isfinite(sync_slop) or sync_slop <= 0.0:
        raise ValueError("D455_SYNC_SLOP must be a positive number of seconds")
    if sync_mode not in ("latest", "approximate"):
        raise ValueError("D455_SYNC_MODE must be 'latest' or 'approximate'")

    loader = SourceFileLoader("d455_perception_installed_detect", str(detector_entry))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    if spec is None:
        raise RuntimeError(f"Cannot load D455 detector: {detector_entry}")
    detector = importlib.util.module_from_spec(spec)
    loader.exec_module(detector)

    model_override = os.environ.get("D455_MODEL")
    if model_override:
        model_path = Path(model_override).expanduser().resolve()
        if not model_path.is_file():
            raise FileNotFoundError(f"D455 model override not found: {model_path}")
        model_sha256 = hashlib.sha256(model_path.read_bytes()).hexdigest()
        expected_sha256 = os.environ.get("D455_MODEL_SHA256", "").strip().lower()
        if expected_sha256 and model_sha256 != expected_sha256:
            raise ValueError(
                "D455 model checksum mismatch: "
                f"expected {expected_sha256}, got {model_sha256}"
            )

        packaged_detector = detector.BollardDetector

        class ModelOverrideDetector(packaged_detector):
            def __init__(self, _packaged_weights, *args, **kwargs):
                super().__init__(model_path, *args, **kwargs)

        detector.BollardDetector = ModelOverrideDetector
        print(
            f"Using D455 model override: {model_path} (sha256={model_sha256})",
            file=sys.stderr,
            flush=True,
        )

    if sync_mode == "approximate":
        original_synchronizer = detector.ApproximateTimeSynchronizer

        def synchronizer_with_field_slop(
            subscribers, queue_size, _packaged_slop, *args, **kwargs
        ):
            return original_synchronizer(
                subscribers, queue_size, sync_slop, *args, **kwargs
            )

        detector.ApproximateTimeSynchronizer = synchronizer_with_field_slop
        sync_description = f"approximate slop={sync_slop:.3f}s"
    else:
        class LatestPairSynchronizer:
            """Pair the newest RGB and depth without accumulating frames during inference."""

            def __init__(self, subscribers, _queue_size, _packaged_slop, *args, **kwargs):
                del args, kwargs
                self._messages = [None] * len(subscribers)
                self._callback = None
                self._callback_args = ()
                self._pair_count = 0
                for index, subscriber in enumerate(subscribers):
                    subscriber.registerCallback(self._receive, index)

            def registerCallback(self, callback, *args):
                self._callback = callback
                self._callback_args = args

            def _receive(self, message, index):
                self._messages[index] = message
                if self._callback is None or any(item is None for item in self._messages):
                    return
                messages = self._messages
                self._messages = [None] * len(messages)
                self._pair_count += 1
                report = self._pair_count <= 3 or self._pair_count % 10 == 0
                if report:
                    print(
                        f"D455 latest-pair callback {self._pair_count} started",
                        file=sys.stderr,
                        flush=True,
                    )
                begin = time.perf_counter()
                self._callback(*messages, *self._callback_args)
                if report:
                    elapsed = time.perf_counter() - begin
                    print(
                        f"D455 latest-pair callback {self._pair_count} finished in {elapsed:.3f}s",
                        file=sys.stderr,
                        flush=True,
                    )

        detector.ApproximateTimeSynchronizer = LatestPairSynchronizer
        sync_description = "latest RGB + latest raw depth (software aligned)"

    print(
        f"Starting installed D455 detector with sync mode: {sync_description}",
        file=sys.stderr,
        flush=True,
    )

    try:
        detector.main()
    except detector.rclpy.executors.ExternalShutdownException:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
