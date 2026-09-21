"""ROS 2 node that records synchronized RealSense RGB-D (up to 4 cameras) +
mocap trajectory demonstrations into a ROS 2 bag, controlled by a Flask web UI.

Color/depth frames are pulled directly from each camera via pyrealsense2
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
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from flask import Flask, Response, abort, jsonify, render_template
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from umi_recorder.bag_writer import EpisodeBagWriter
from umi_recorder.camera_stream import CameraStream

MAX_TRAJECTORY_POINTS = 20
MAX_CAMERAS = 4


class RecorderNode(Node):
    def __init__(self):
        super().__init__("umi_recorder_node")

        self.declare_parameter("camera_names", ["camera_0"])
        self.declare_parameter("camera_serials", [""])
        self.declare_parameter("camera_widths", [640])
        self.declare_parameter("camera_heights", [480])
        self.declare_parameter("camera_fps", [30])
        self.declare_parameter("camera_max_depth_m", [0.5])
        self.declare_parameter("pose_topic", "/mocap/umi_gripper/pose")
        self.declare_parameter("output_dir", str(Path.home() / "umi_episodes"))
        self.declare_parameter("web_host", "0.0.0.0")
        self.declare_parameter("web_port", 8080)

        camera_names = list(self.get_parameter("camera_names").value)
        camera_serials = list(self.get_parameter("camera_serials").value)
        camera_widths = list(self.get_parameter("camera_widths").value)
        camera_heights = list(self.get_parameter("camera_heights").value)
        camera_fps_list = list(self.get_parameter("camera_fps").value)
        camera_max_depths = list(self.get_parameter("camera_max_depth_m").value)

        if len(camera_names) > MAX_CAMERAS:
            raise ValueError(
                f"Configured {len(camera_names)} cameras (camera_names), but at most "
                f"{MAX_CAMERAS} are supported."
            )
        per_camera_lists = {
            "camera_serials": camera_serials,
            "camera_widths": camera_widths,
            "camera_heights": camera_heights,
            "camera_fps": camera_fps_list,
            "camera_max_depth_m": camera_max_depths,
        }
        for param_name, values in per_camera_lists.items():
            if len(values) != len(camera_names):
                raise ValueError(
                    f"camera_names and {param_name} must have the same length "
                    f"(got {len(camera_names)} names, {len(values)} entries in {param_name})."
                )

        pose_topic = self._pose_topic = self.get_parameter("pose_topic").value
        output_dir = Path(self.get_parameter("output_dir").value)
        output_dir.mkdir(parents=True, exist_ok=True)

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._trajectory: list[dict] = []
        self._recording = False
        self._bag = EpisodeBagWriter(output_dir)
        self._current_bag_path: Path | None = None

        self._cameras: dict[str, CameraStream] = {}
        for i, name in enumerate(camera_names):
            if name in self._cameras:
                raise ValueError(f"Duplicate camera name in camera_names: {name!r}")
            self._cameras[name] = CameraStream(
                name=name,
                serial=camera_serials[i],
                width=int(camera_widths[i]),
                height=int(camera_heights[i]),
                fps=int(camera_fps_list[i]),
                max_depth_m=float(camera_max_depths[i]),
                bridge=self._bridge,
                bag=self._bag,
                logger=self.get_logger(),
                clock=self.get_clock(),
            )
        for camera in self._cameras.values():
            camera.start()

        pose_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            PoseStamped, pose_topic, self._on_pose, qos_profile_sensor_data, callback_group=pose_group
        )

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
            f"{self.get_parameter('web_port').value}  episodes -> {output_dir}  "
            f"cameras -> {list(self._cameras.keys())}"
        )

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
        for camera in self._cameras.values():
            camera.set_recording(True)
        self.get_logger().info(f"Recording started -> {self._current_bag_path}")
        return str(self._current_bag_path)

    def stop_recording(self) -> str | None:
        if not self._recording:
            return None
        self._recording = False
        for camera in self._cameras.values():
            camera.set_recording(False)
        self._bag.close()
        path = self._current_bag_path
        self.get_logger().info(f"Recording stopped -> {path}")
        self._current_bag_path = None
        return str(path) if path else None

    def destroy_node(self) -> bool:
        for camera in self._cameras.values():
            camera.stop()
        return super().destroy_node()

    # -- Flask app -----------------------------------------------------------

    def _build_flask_app(self) -> Flask:
        logging.getLogger("werkzeug").setLevel(logging.WARNING)
        template_folder = Path(get_package_share_directory("umi_recorder")) / "templates"
        app = Flask(__name__, template_folder=str(template_folder))

        @app.route("/")
        def index():
            return render_template("index.html")

        @app.route("/cameras")
        def cameras():
            return jsonify(list(self._cameras.keys()))

        @app.route("/video_feed/<name>")
        def video_feed(name):
            if name not in self._cameras:
                abort(404)
            return Response(
                self._mjpeg_generator(name, "color"),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

        @app.route("/depth_feed/<name>")
        def depth_feed(name):
            if name not in self._cameras:
                abort(404)
            return Response(
                self._mjpeg_generator(name, "depth"),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

        @app.route("/trajectory")
        def trajectory():
            with self._lock:
                return jsonify(self._trajectory[-MAX_TRAJECTORY_POINTS:])

        @app.route("/config")
        def config():
            return jsonify(
                {name: {"max_depth_m": camera.max_depth_m} for name, camera in self._cameras.items()}
            )

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

    def _mjpeg_generator(self, name: str, stream: str):
        camera = self._cameras[name]
        blank = np.zeros((480, 640, 3), dtype=np.uint8)
        _, blank_jpeg = cv2.imencode(".jpg", blank)
        blank_jpeg = blank_jpeg.tobytes()
        condition = camera.color_ready if stream == "color" else camera.depth_ready
        last_sent: bytes | None = None

        while rclpy.ok():
            with condition:
                condition.wait(timeout=1.0)
                latest = camera.latest_color_jpeg if stream == "color" else camera.latest_depth_jpeg
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
