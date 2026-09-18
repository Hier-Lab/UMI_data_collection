"""Helpers for reading an episode bag's messages and rewriting it with an
added topic, since rosbag2 bags are write-once (no append-in-place)."""

from __future__ import annotations

import shutil
import tempfile
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message


def get_topic_types(bag_path: Path) -> dict[str, str]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    return {t.name: t.type for t in reader.get_all_topics_and_types()}


def read_messages(bag_path: Path, topics: set[str]):
    """Yield (topic, msg, stamp_ns) for the given topics, in on-disk order."""
    topic_types = get_topic_types(bag_path)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    msg_classes = {name: get_message(type_str) for name, type_str in topic_types.items()}
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic in topics:
            yield topic, deserialize_message(data, msg_classes[topic]), stamp_ns


class AugmentedBagWriter:
    """Copies every message from a source bag into a new bag, plus whatever
    extra (topic, msg, stamp_ns) triples are added via write_extra(), then
    replaces the source bag with the augmented one."""

    def __init__(self, source_bag_path: Path):
        self._source_path = source_bag_path
        self._tmp_dir = Path(tempfile.mkdtemp(prefix="rgbd_odometry_"))
        self._tmp_path = self._tmp_dir / source_bag_path.name
        self._extra_topics_registered: set[str] = set()

        self._writer = rosbag2_py.SequentialWriter()
        self._writer.open(
            rosbag2_py.StorageOptions(uri=str(self._tmp_path), storage_id="sqlite3"),
            rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
        )

        topic_types = get_topic_types(source_bag_path)
        for name, type_str in topic_types.items():
            self._writer.create_topic(
                rosbag2_py.TopicMetadata(name=name, type=type_str, serialization_format="cdr")
            )
        self._existing_topics = set(topic_types.keys())

        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(source_bag_path), storage_id="sqlite3"),
            rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
        )
        while reader.has_next():
            topic, data, stamp_ns = reader.read_next()
            self._writer.write(topic, data, stamp_ns)

    def write_extra(self, topic: str, msg_type: str, msg, stamp_ns: int) -> None:
        if topic not in self._existing_topics and topic not in self._extra_topics_registered:
            self._writer.create_topic(
                rosbag2_py.TopicMetadata(name=topic, type=msg_type, serialization_format="cdr")
            )
            self._extra_topics_registered.add(topic)
        self._writer.write(topic, serialize_message(msg), stamp_ns)

    def finalize(self) -> None:
        """Close the augmented bag and atomically replace the source bag with it."""
        self._writer.close()
        shutil.rmtree(self._source_path)
        shutil.move(str(self._tmp_path), str(self._source_path))
        shutil.rmtree(self._tmp_dir, ignore_errors=True)
