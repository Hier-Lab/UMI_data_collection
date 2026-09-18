"""ROS 2 node that records synchronized RealSense RGB-D + mocap trajectory
demonstrations into a ROS 2 bag, controlled by a Flask web UI.

Color/depth frames are pulled directly from the camera via pyrealsense2
(bypassing the realsense2_camera ROS node), since that ROS wrapper was
measured to degrade from ~30fps to ~5-8fps within seconds under this
node's config, while the raw SDK sustains a steady 30fps. Depth is
aligned to color; no point cloud is computed or published.
"""

from __future__ import annotations

import logging
import threading
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from flask import Flask, Response, jsonify, render_template
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from umi_recorder.bag_writer import EpisodeBagWriter

MAX_TRAJECTORY_POINTS = 20


class RecorderNode(Node):
    def __init__(self):
        super().__init__("umi_recorder_node")

        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("pose_topic", "/mocap/umi_gripper/pose")
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("camera_fps", 30)
        self.declare_parameter("max_depth_m", 0.5)
        self.declare_parameter("output_dir", str(Path.home() / "umi_episodes"))
        self.declare_parameter("web_host", "0.0.0.0")
        self.declare_parameter("web_port", 8080)

        self._color_topic = self.get_parameter("color_topic").value
        self._depth_topic = self.get_parameter("depth_topic").value
        pose_topic = self._pose_topic = self.get_parameter("pose_topic").value
        self._max_depth_m = float(self.get_parameter("max_depth_m").value)
        width = int(self.get_parameter("camera_width").value)
        height = int(self.get_parameter("camera_height").value)
        fps = int(self.get_parameter("camera_fps").value)
        output_dir = Path(self.get_parameter("output_dir").value)
        output_dir.mkdir(parents=True, exist_ok=True)

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._color_ready = threading.Condition(self._lock)
        self._depth_ready = threading.Condition(self._lock)
        self._latest_color_jpeg: bytes | None = None
        self._latest_depth_jpeg: bytes | None = None
        self._trajectory: list[dict] = []
        self._recording = False
        self._bag = EpisodeBagWriter(output_dir)
        self._current_bag_path: Path | None = None

        pose_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            PoseStamped, pose_topic, self._on_pose, qos_profile_sensor_data, callback_group=pose_group
        )

        self._camera_stop = threading.Event()
        self._camera_thread = threading.Thread(
            target=self._camera_loop, args=(width, height, fps), daemon=True
        )
        self._camera_thread.start()

        self._app = self._build_flask_app()
        self._web_thread = threading.Thread(
            target=self._app.run,
            kwargs={
                "host": self.get_parameter("web_host").value,
                "port": self.get_parameter("web_port").value,
                "threaded": True,
                "use_reloader": False,
            },
            daemon=True,
        )
        self._web_thread.start()

        self.get_logger().info(
            f"umi_recorder_node ready. UI on http://{self.get_parameter('web_host').value}:"
            f"{self.get_parameter('web_port').value}  episodes -> {output_dir}"
        )

    # -- Camera capture (pyrealsense2, direct SDK access) -------------------

    def _camera_loop(self, width: int, height: int, fps: int) -> None:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        align = rs.align(rs.stream.color)

        pipeline.start(config)
        self.get_logger().info(f"pyrealsense2 pipeline started ({width}x{height}@{fps})")
        try:
            while not self._camera_stop.is_set() and rclpy.ok():
                frames = pipeline.wait_for_frames(timeout_ms=1000)
                frames = align.process(frames)
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue

                color = np.asanyarray(color_frame.get_data())
                depth_mm = np.asanyarray(depth_frame.get_data())
                stamp = self.get_clock().now()
                self._handle_color(color, stamp)
                self._handle_depth(depth_mm, stamp)
        except Exception:
            self.get_logger().exception("pyrealsense2 capture loop stopped")
        finally:
            pipeline.stop()

    def _handle_color(self, color_bgr: np.ndarray, stamp) -> None:
        ok, jpeg = cv2.imencode(".jpg", color_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if ok:
            with self._color_ready:
                self._latest_color_jpeg = jpeg.tobytes()
                self._color_ready.notify_all()

        if self._recording:
            msg = self._bridge.cv2_to_imgmsg(color_bgr, encoding="bgr8")
            msg.header.stamp = stamp.to_msg()
            self._bag.register_topic(self._color_topic, "sensor_msgs/msg/Image")
            self._bag.write(self._color_topic, msg, stamp.nanoseconds)

    def _handle_depth(self, depth_mm: np.ndarray, stamp) -> None:
        max_depth_mm = self._max_depth_m * 1000.0
        normalized = np.clip(depth_mm.astype(np.float32), 0, max_depth_mm) / max_depth_mm
        gray = (255 - (normalized * 255)).astype(np.uint8)  # near = bright, far/clipped = dark
        colorized = cv2.applyColorMap(gray, cv2.COLORMAP_JET)
        colorized[depth_mm == 0] = 0  # no-return pixels stay black instead of "far"
        ok, jpeg = cv2.imencode(".jpg", colorized, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if ok:
            with self._depth_ready:
                self._latest_depth_jpeg = jpeg.tobytes()
                self._depth_ready.notify_all()

        if self._recording:
            msg = self._bridge.cv2_to_imgmsg(depth_mm, encoding="16UC1")
            msg.header.stamp = stamp.to_msg()
            self._bag.register_topic(self._depth_topic, "sensor_msgs/msg/Image")
            self._bag.write(self._depth_topic, msg, stamp.nanoseconds)

    # -- ROS callbacks -------------------------------------------------------

    def _on_pose(self, msg: PoseStamped) -> None:
        point = {
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "z": msg.pose.position.z,
            "qx": msg.pose.orientation.x,
            "qy": msg.pose.orientation.y,
            "qz": msg.pose.orientation.z,
            "qw": msg.pose.orientation.w,
        }
        with self._lock:
            self._trajectory.append(point)
            if len(self._trajectory) > MAX_TRAJECTORY_POINTS:
                self._trajectory.pop(0)

        if self._recording:
            self._bag.register_topic(self._pose_topic, "geometry_msgs/msg/PoseStamped")
            self._bag.write(self._pose_topic, msg, self.get_clock().now().nanoseconds)

    # -- Recording control ---------------------------------------------------

    def start_recording(self) -> str:
        if self._recording:
            return str(self._current_bag_path)
        self._current_bag_path = self._bag.open()
        with self._lock:
            self._trajectory = []
        self._recording = True
        self.get_logger().info(f"Recording started -> {self._current_bag_path}")
        return str(self._current_bag_path)

    def stop_recording(self) -> str | None:
        if not self._recording:
            return None
        self._recording = False
        self._bag.close()
        path = self._current_bag_path
        self.get_logger().info(f"Recording stopped -> {path}")
        self._current_bag_path = None
        return str(path) if path else None

    def destroy_node(self) -> bool:
        self._camera_stop.set()
        self._camera_thread.join(timeout=2.0)
        return super().destroy_node()

    # -- Flask app -----------------------------------------------------------

    def _build_flask_app(self) -> Flask:
        logging.getLogger("werkzeug").setLevel(logging.WARNING)
        template_folder = Path(get_package_share_directory("umi_recorder")) / "templates"
        app = Flask(__name__, template_folder=str(template_folder))

        @app.route("/")
        def index():
            return render_template("index.html")

        @app.route("/video_feed")
        def video_feed():
            return Response(
                self._mjpeg_generator("color"),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

        @app.route("/depth_feed")
        def depth_feed():
            return Response(
                self._mjpeg_generator("depth"),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

        @app.route("/trajectory")
        def trajectory():
            with self._lock:
                return jsonify(self._trajectory[-MAX_TRAJECTORY_POINTS:])

        @app.route("/config")
        def config():
            return jsonify({"max_depth_m": self._max_depth_m})

        @app.route("/status")
        def status():
            return jsonify(
                {
                    "recording": self._recording,
                    "bag_path": str(self._current_bag_path) if self._current_bag_path else None,
                }
            )

        @app.route("/record", methods=["POST"])
        def record():
            path = self.start_recording()
            return jsonify({"recording": True, "bag_path": path})

        @app.route("/stop", methods=["POST"])
        def stop():
            path = self.stop_recording()
            return jsonify({"recording": False, "bag_path": path})

        return app

    def _mjpeg_generator(self, stream: str):
        blank = np.zeros((480, 640, 3), dtype=np.uint8)
        _, blank_jpeg = cv2.imencode(".jpg", blank)
        blank_jpeg = blank_jpeg.tobytes()
        condition = self._color_ready if stream == "color" else self._depth_ready
        last_sent: bytes | None = None

        while rclpy.ok():
            with condition:
                condition.wait(timeout=1.0)
                latest = self._latest_color_jpeg if stream == "color" else self._latest_depth_jpeg
            frame = latest or blank_jpeg
            if frame is last_sent:
                continue  # timed out with no new frame; avoid resending the same bytes
            last_sent = frame
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")


def main(args=None):
    rclpy.init(args=args)
    node = RecorderNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
