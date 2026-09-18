"""Thin wrapper around rosbag2_py for recording a demonstration episode."""

from __future__ import annotations

import time
from pathlib import Path

import rosbag2_py
from rclpy.serialization import serialize_message


class EpisodeBagWriter:
    """Writes one ROS 2 bag per demonstration episode."""

    def __init__(self, output_dir: Path):
        self._output_dir = output_dir
        self._writer: rosbag2_py.SequentialWriter | None = None
        self._known_topics: set[str] = set()

    @property
    def is_open(self) -> bool:
        return self._writer is not None

    def open(self) -> Path:
        if self._writer is not None:
            raise RuntimeError("bag already open")
        bag_path = self._output_dir / f"episode_{time.strftime('%Y%m%d_%H%M%S')}"
        writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3")
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        )
        writer.open(storage_options, converter_options)
        self._writer = writer
        self._known_topics = set()
        return bag_path

    def register_topic(self, topic: str, msg_type: str) -> None:
        if topic in self._known_topics:
            return
        self._writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=topic, type=msg_type, serialization_format="cdr"
            )
        )
        self._known_topics.add(topic)

    def write(self, topic: str, msg, stamp_ns: int) -> None:
        if self._writer is None:
            return
        self._writer.write(topic, serialize_message(msg), stamp_ns)

    def close(self) -> None:
        self._writer = None
