"""Post-process one camera of a recorded umi_recorder episode bag: compute
frame-to-frame RGBD visual odometry with Open3D and write the resulting
trajectory back into the same bag as a new /<camera>/visual_odom
(geometry_msgs/PoseStamped) topic.

The actual Open3D math runs as a subprocess inside this package's own venv
(.venv/, modern numpy) rather than in this process: the numpy ABI that
open3d 0.20 needs is incompatible with the numpy this ROS 2 Humble
environment pins for rclpy/cv_bridge/rosbag2_py, and Image() construction
from a numpy array fails outright if both try to share one interpreter.
Frame data crosses the boundary via a temporary .npz file.

Bags can't be appended to in place (rosbag2's SequentialWriter is
write-once), so this reads every message from the source bag, computes
odometry, writes a new bag containing everything plus the new topic, and
atomically replaces the original bag with it.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image

from rgbd_odometry.bag_io import AugmentedBagWriter, get_topic_types, read_messages

VISUAL_ODOM_MSG_TYPE = "geometry_msgs/msg/PoseStamped"
PACKAGE_DIR = Path(__file__).resolve().parent.parent
VENV_PYTHON = PACKAGE_DIR / ".venv" / "bin" / "python3"
VENV_ODOMETRY_SCRIPT = Path(__file__).resolve().parent / "_venv_odometry.py"


def rotation_matrix_to_quaternion(rot: np.ndarray) -> tuple[float, float, float, float]:
    """Standard matrix->quaternion conversion (Shepperd's method), returned
    as (x, y, z, w). Avoids depending on scipy, which - like open3d - would
    need to match whatever numpy ABI this process's numpy is."""
    trace = rot[0, 0] + rot[1, 1] + rot[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (rot[2, 1] - rot[1, 2]) * s
        y = (rot[0, 2] - rot[2, 0]) * s
        z = (rot[1, 0] - rot[0, 1]) * s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2])
        w = (rot[2, 1] - rot[1, 2]) / s
        x = 0.25 * s
        y = (rot[0, 1] + rot[1, 0]) / s
        z = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2])
        w = (rot[0, 2] - rot[2, 0]) / s
        x = (rot[0, 1] + rot[1, 0]) / s
        y = 0.25 * s
        z = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1])
        w = (rot[1, 0] - rot[0, 1]) / s
        x = (rot[0, 2] + rot[2, 0]) / s
        y = (rot[1, 2] + rot[2, 1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


def pose_to_msg(pose: np.ndarray, frame_id: str, stamp_ns: int) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp.sec = int(stamp_ns // 1_000_000_000)
    msg.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
    msg.pose.position.x = float(pose[0, 3])
    msg.pose.position.y = float(pose[1, 3])
    msg.pose.position.z = float(pose[2, 3])
    qx, qy, qz, qw = rotation_matrix_to_quaternion(pose[:3, :3])
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    return msg


def load_camera_frames(
    bag_path: Path, camera: str
) -> tuple[CameraInfo, list[tuple[int, np.ndarray, np.ndarray]]]:
    """Reads and pairs this camera's color/depth/camera_info from the bag.
    Returns (camera_info, [(stamp_ns, color_rgb_uint8, depth_mm_uint16), ...])
    in ascending timestamp order."""
    color_topic = f"/{camera}/color/image_raw"
    depth_topic = f"/{camera}/aligned_depth_to_color/image_raw"
    camera_info_topic = f"/{camera}/color/camera_info"

    topic_types = get_topic_types(bag_path)
    for topic in (color_topic, depth_topic, camera_info_topic):
        if topic not in topic_types:
            raise ValueError(
                f"Bag {bag_path} has no topic {topic!r} - available topics: "
                f"{sorted(topic_types)}"
            )

    bridge = CvBridge()
    camera_info: CameraInfo | None = None
    color_by_stamp: dict[int, np.ndarray] = {}
    depth_by_stamp: dict[int, np.ndarray] = {}
    for topic, msg, stamp_ns in read_messages(
        bag_path, {color_topic, depth_topic, camera_info_topic}
    ):
        if topic == camera_info_topic and camera_info is None:
            camera_info = msg
        elif topic == color_topic:
            color_by_stamp[stamp_ns] = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        elif topic == depth_topic:
            depth_by_stamp[stamp_ns] = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    if camera_info is None:
        raise ValueError(
            f"Bag {bag_path} has no {camera_info_topic!r} message - re-record with a "
            "version of umi_recorder that writes CameraInfo, or this camera's real "
            "intrinsics can't be recovered."
        )
    if not depth_by_stamp:
        return camera_info, []

    depth_stamps = sorted(depth_by_stamp)

    def nearest_depth_stamp(color_stamp: int) -> int:
        idx = min(np.searchsorted(depth_stamps, color_stamp), len(depth_stamps) - 1)
        if idx > 0 and abs(depth_stamps[idx - 1] - color_stamp) < abs(depth_stamps[idx] - color_stamp):
            idx -= 1
        return depth_stamps[idx]

    frames = []
    for color_stamp in sorted(color_by_stamp):
        depth_stamp = nearest_depth_stamp(color_stamp)
        frames.append((color_stamp, color_by_stamp[color_stamp], depth_by_stamp[depth_stamp]))
    return camera_info, frames


def compute_trajectory_via_venv(
    camera_info: CameraInfo,
    frames: list[tuple[int, np.ndarray, np.ndarray]],
    depth_trunc_m: float,
) -> list[tuple[int, np.ndarray]]:
    if not VENV_PYTHON.exists():
        raise RuntimeError(
            f"rgbd_odometry's venv not found at {VENV_PYTHON} - create it with:\n"
            f"  virtualenv {PACKAGE_DIR}/.venv && "
            f"{PACKAGE_DIR}/.venv/bin/pip install open3d numpy"
        )

    stamps_ns = np.array([f[0] for f in frames], dtype=np.int64)
    color = np.stack([f[1] for f in frames]).astype(np.uint8)
    depth_mm = np.stack([f[2] for f in frames]).astype(np.uint16)

    with tempfile.TemporaryDirectory(prefix="rgbd_odometry_") as tmp_dir:
        in_path = Path(tmp_dir) / "frames.npz"
        out_path = Path(tmp_dir) / "trajectory.npz"
        np.savez(
            in_path,
            color=color,
            depth_mm=depth_mm,
            stamps_ns=stamps_ns,
            fx=camera_info.k[0],
            fy=camera_info.k[4],
            cx=camera_info.k[2],
            cy=camera_info.k[5],
            width=camera_info.width,
            height=camera_info.height,
            depth_trunc_m=depth_trunc_m,
        )
        subprocess.run(
            [str(VENV_PYTHON), str(VENV_ODOMETRY_SCRIPT), str(in_path), str(out_path)],
            check=True,
        )
        result = np.load(out_path)
        return list(zip(result["stamps_ns"].tolist(), result["poses"]))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, help="Path to an episode_<timestamp> bag directory")
    parser.add_argument("--camera", required=True, help="Camera name (matches umi_recorder's camera_names)")
    parser.add_argument(
        "--depth-trunc",
        type=float,
        default=2.0,
        help="Truncate depth beyond this many meters before computing odometry (default: 2.0)",
    )
    args = parser.parse_args(argv)

    bag_path = Path(args.bag).resolve()
    if not bag_path.exists():
        print(f"error: bag not found: {bag_path}", file=sys.stderr)
        return 1

    print(f"Reading '{args.camera}' color/depth/camera_info from {bag_path} ...")
    camera_info, frames = load_camera_frames(bag_path, args.camera)
    if len(frames) < 2:
        print("error: need at least 2 color/depth frame pairs to compute odometry", file=sys.stderr)
        return 1
    print(f"Loaded {len(frames)} frame pairs. Computing odometry (via venv subprocess) ...")

    trajectory = compute_trajectory_via_venv(camera_info, frames, args.depth_trunc)
    print(f"Computed {len(trajectory)} poses.")

    visual_odom_topic = f"/{args.camera}/visual_odom"
    frame_id = f"{args.camera}_color_optical_frame"

    print(f"Writing {visual_odom_topic} back into {bag_path} ...")
    writer = AugmentedBagWriter(bag_path)
    origin_stamp = frames[0][0]
    writer.write_extra(
        visual_odom_topic, VISUAL_ODOM_MSG_TYPE, pose_to_msg(np.eye(4), frame_id, origin_stamp), origin_stamp
    )
    for stamp_ns, pose in trajectory:
        writer.write_extra(
            visual_odom_topic, VISUAL_ODOM_MSG_TYPE, pose_to_msg(pose, frame_id, stamp_ns), stamp_ns
        )
    writer.finalize()

    print(f"Done. {bag_path} now contains {visual_odom_topic}.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
