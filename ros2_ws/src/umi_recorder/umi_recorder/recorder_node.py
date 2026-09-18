"""ROS 2 node that records synchronized RealSense RGB-D + mocap trajectory
demonstrations into a ROS 2 bag, controlled by a Flask web UI."""

from __future__ import annotations

import threading
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from flask import Flask, Response, jsonify, render_template
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from umi_recorder.bag_writer import EpisodeBagWriter

MAX_TRAJECTORY_POINTS = 20


class RecorderNode(Node):
    def __init__(self):
        super().__init__("umi_recorder_node")

        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("pose_topic", "/mocap/umi_gripper/pose")
        self.declare_parameter("output_dir", str(Path.home() / "umi_episodes"))
        self.declare_parameter("web_host", "0.0.0.0")
        self.declare_parameter("web_port", 8080)

        color_topic = self.get_parameter("color_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        pose_topic = self.get_parameter("pose_topic").value
        output_dir = Path(self.get_parameter("output_dir").value)
        output_dir.mkdir(parents=True, exist_ok=True)

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_color_jpeg: bytes | None = None
        self._trajectory: list[dict] = []
        self._recording = False
        self._bag = EpisodeBagWriter(output_dir)
        self._current_bag_path: Path | None = None

        self._color_topic = color_topic
        self._depth_topic = depth_topic
        self._pose_topic = pose_topic

        self.create_subscription(Image, color_topic, self._on_color, qos_profile_sensor_data)
        self.create_subscription(Image, depth_topic, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, pose_topic, self._on_pose, qos_profile_sensor_data)

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

    # -- ROS callbacks -----------------------------------------------------

    def _on_color(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        ok, jpeg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if ok:
            with self._lock:
                self._latest_color_jpeg = jpeg.tobytes()

        if self._recording:
            self._bag.register_topic(self._color_topic, "sensor_msgs/msg/Image")
            self._bag.write(self._color_topic, msg, self.get_clock().now().nanoseconds)

    def _on_depth(self, msg: Image) -> None:
        if self._recording:
            self._bag.register_topic(self._depth_topic, "sensor_msgs/msg/Image")
            self._bag.write(self._depth_topic, msg, self.get_clock().now().nanoseconds)

    def _on_pose(self, msg: PoseStamped) -> None:
        point = {
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "z": msg.pose.position.z,
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

    # -- Flask app -----------------------------------------------------------

    def _build_flask_app(self) -> Flask:
        template_folder = Path(get_package_share_directory("umi_recorder")) / "templates"
        app = Flask(__name__, template_folder=str(template_folder))

        @app.route("/")
        def index():
            return render_template("index.html")

        @app.route("/video_feed")
        def video_feed():
            return Response(
                self._mjpeg_generator(),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

        @app.route("/trajectory")
        def trajectory():
            with self._lock:
                return jsonify(self._trajectory[-MAX_TRAJECTORY_POINTS:])

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

    def _mjpeg_generator(self):
        blank = np.zeros((480, 640, 3), dtype=np.uint8)
        _, blank_jpeg = cv2.imencode(".jpg", blank)
        while rclpy.ok():
            with self._lock:
                frame = self._latest_color_jpeg or blank_jpeg.tobytes()
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
            threading.Event().wait(1 / 30)


def main(args=None):
    rclpy.init(args=args)
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
