"""Runs MediaPipe Pose on an exocentric camera's live color feed to track
human torso motion (for mapping to linear base motion), publishing:

  /<exo_camera_name>/torso_pose      geometry_msgs/PointStamped
      Normalized image coordinates (x, y in [0, 1], z unused/0), the
      average of the shoulder-midpoint and hip-midpoint landmarks. Not a
      metric position - depth back-projection was deliberately left out
      for simplicity; add it in post-processing if a real XYZ is needed
      later, the same way rgbd_odometry post-processes bags rather than
      doing everything live.

  /<exo_camera_name>/pose_overlay/image_raw/compressed   sensor_msgs/CompressedImage
      The color frame with the detected skeleton drawn on it, for the
      umi_recorder monitor UI - not recorded into the bag (derived from
      data that's already recorded: color + torso_pose).

Both the subscribed color feed and the published overlay use
CompressedImage (JPEG), not raw Image: publishing raw 640x480 bgr8 frames
(~0.9MB each) at 30fps through rclpy/DDS measured only ~3-5fps actually
delivered even with the subscriber's CPU barely loaded - the same class of
ROS-transport overhead that got realsense2_camera dropped from this
project (see umi_recorder/camera_stream.py's publish_live comment).
Decoding/encoding JPEG costs a bit of CPU here but keeps every hop in this
pipeline at a steady 30fps.

This node intentionally does NOT depend on cv_bridge: mediapipe needs its
own numpy (2.x) and opencv-contrib-python, installed in this package's own
venv (.venv/, gitignored, built with --system-site-packages so rclpy/
sensor_msgs/geometry_msgs - which don't touch numpy's C API - still work).
cv_bridge's compiled extension (cv_bridge_boost.so) is built against the
system's pinned numpy 1.x and crashes with "AttributeError: _ARRAY_API not
found" the moment mediapipe's numpy 2.x shadows it in this venv (verified
empirically). JPEG decode/encode via plain cv2 avoids the conflict
entirely rather than fighting it.

Must be run with THIS package's venv Python, not the system one:
  ros2_ws/src/exo_pose_bridge/.venv/bin/python3 -m exo_pose_bridge.pose_node --ros-args -p exo_camera_name:=exo
"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision

# Standard BlazePose landmark indices (stable across MediaPipe versions).
LEFT_SHOULDER, RIGHT_SHOULDER = 11, 12
LEFT_HIP, RIGHT_HIP = 23, 24

# BlazePose's 33-landmark skeleton, as (from, to) index pairs, for the
# overlay drawing - intentionally not using mediapipe's own drawing_utils
# (it imports matplotlib, adding a heavy and ABI-fragile dependency for a
# handful of cv2.line calls we can just do directly).
POSE_CONNECTIONS = [
    (11, 12), (11, 13), (13, 15), (12, 14), (14, 16),  # arms + shoulders
    (11, 23), (12, 24), (23, 24),  # torso
    (23, 25), (25, 27), (27, 29), (29, 31),  # left leg
    (24, 26), (26, 28), (28, 30), (30, 32),  # right leg
]


def _decode_jpeg_to_bgr(msg: CompressedImage) -> np.ndarray:
    return cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)


def _encode_bgr_to_jpeg_msg(bgr: np.ndarray, stamp) -> CompressedImage:
    msg = CompressedImage()
    msg.header.stamp = stamp
    msg.format = "jpeg"
    ok, jpeg = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    msg.data = jpeg.tobytes() if ok else b""
    return msg


class PoseNode(Node):
    def __init__(self):
        super().__init__("exo_pose_bridge_node")

        # Empty by default so this node can always be launched alongside
        # umi_recorder (same convention as umi_recorder_node's own
        # exo_camera_name param) without erroring when no exo camera is
        # configured - it just does nothing instead of loading a model and
        # creating topics for a camera that doesn't exist.
        self.declare_parameter("exo_camera_name", "")
        self.declare_parameter(
            "model_path",
            str(Path(__file__).resolve().parent.parent / "models" / "pose_landmarker_lite.task"),
        )

        camera_name = self.get_parameter("exo_camera_name").value
        if not camera_name:
            self.get_logger().info("exo_camera_name is empty - exo_pose_bridge_node disabled")
            self._landmarker = None
            return

        model_path = self.get_parameter("model_path").value

        base_options = mp_python.BaseOptions(model_asset_path=model_path)
        options = mp_vision.PoseLandmarkerOptions(
            base_options=base_options,
            running_mode=mp_vision.RunningMode.IMAGE,
            num_poses=1,
        )
        self._landmarker = mp_vision.PoseLandmarker.create_from_options(options)

        color_topic = f"/{camera_name}/color/image_raw/compressed"
        self._torso_pose_pub = self.create_publisher(PointStamped, f"/{camera_name}/torso_pose", 10)
        self._overlay_pub = self.create_publisher(
            CompressedImage, f"/{camera_name}/pose_overlay/image_raw/compressed", 10
        )
        self.create_subscription(CompressedImage, color_topic, self._on_color, qos_profile_sensor_data)

        self.get_logger().info(f"exo_pose_bridge_node ready, subscribed to {color_topic}")

    def _on_color(self, msg: CompressedImage) -> None:
        bgr = _decode_jpeg_to_bgr(msg)
        rgb = bgr[:, :, ::-1]
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=np.ascontiguousarray(rgb))
        result = self._landmarker.detect(mp_image)

        overlay = bgr.copy()
        if result.pose_landmarks:
            landmarks = result.pose_landmarks[0]
            self._draw_skeleton(overlay, landmarks)

            torso_x = (
                landmarks[LEFT_SHOULDER].x
                + landmarks[RIGHT_SHOULDER].x
                + landmarks[LEFT_HIP].x
                + landmarks[RIGHT_HIP].x
            ) / 4.0
            torso_y = (
                landmarks[LEFT_SHOULDER].y
                + landmarks[RIGHT_SHOULDER].y
                + landmarks[LEFT_HIP].y
                + landmarks[RIGHT_HIP].y
            ) / 4.0

            point_msg = PointStamped()
            point_msg.header.stamp = msg.header.stamp
            point_msg.header.frame_id = msg.header.frame_id
            point_msg.point.x = float(torso_x)
            point_msg.point.y = float(torso_y)
            point_msg.point.z = 0.0
            self._torso_pose_pub.publish(point_msg)

        self._overlay_pub.publish(_encode_bgr_to_jpeg_msg(overlay, msg.header.stamp))

    def _draw_skeleton(self, image: np.ndarray, landmarks) -> None:
        h, w = image.shape[:2]
        points = [(int(lm.x * w), int(lm.y * h)) for lm in landmarks]
        for a, b in POSE_CONNECTIONS:
            cv2.line(image, points[a], points[b], (0, 255, 0), 2)
        for x, y in points:
            cv2.circle(image, (x, y), 3, (0, 0, 255), -1)


def main(args=None):
    rclpy.init(args=args)
    node = PoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
