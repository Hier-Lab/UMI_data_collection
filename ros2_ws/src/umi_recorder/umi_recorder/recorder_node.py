"""ROS 2 node that records synchronized RealSense RGB-D (up to 4 cameras) +
mocap trajectory demonstrations into a ROS 2 bag, controlled by a Flask web UI.

Color/depth frames are pulled directly from each camera via pyrealsense2
(bypassing the realsense2_camera ROS node), since that ROS wrapper was
measured to degrade from ~30fps to ~5-8fps within seconds under this
node's config, while the raw SDK sustains a steady 30fps. Depth is
aligned to color; no point cloud is computed or published.
"""

from __future__ import annotations

import base64
import json
import logging
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from flask import Flask, Response, jsonify, render_template
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

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
        # pose_names/pose_topics are matched by position (same convention as
        # camera_names/camera_serials) - one entry per tracked UMI device, so
        # multiple grippers each get their own recorded bag topic, trajectory
        # buffer, and 3D trail in the UI instead of only the first one
        # mattering.
        self.declare_parameter("pose_names", ["umi_gripper"])
        self.declare_parameter("pose_topics", ["/mocap/umi_gripper/pose"])
        self.declare_parameter("output_dir", str(Path.home() / "umi_episodes"))
        self.declare_parameter("web_host", "0.0.0.0")
        self.declare_parameter("web_port", 8080)

        # Exocentric camera (e.g. a D435i facing the human operator) - a
        # single camera outside the MAX_CAMERAS-capped camera_* lists above,
        # since it serves a different purpose (torso tracking) and isn't
        # subject to that cap. Empty name/serial disables it entirely.
        self.declare_parameter("exo_camera_name", "")
        self.declare_parameter("exo_camera_serial", "")
        self.declare_parameter("exo_camera_width", 640)
        self.declare_parameter("exo_camera_height", 480)
        self.declare_parameter("exo_camera_fps", 30)
        self.declare_parameter("exo_camera_max_depth_m", 3.0)

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

        pose_names = list(self.get_parameter("pose_names").value)
        pose_topics = list(self.get_parameter("pose_topics").value)
        if len(pose_topics) != len(pose_names):
            raise ValueError(
                f"pose_names and pose_topics must have the same length "
                f"(got {len(pose_names)} names, {len(pose_topics)} topics)."
            )
        if len(set(pose_names)) != len(pose_names):
            raise ValueError(f"Duplicate name in pose_names: {pose_names!r}")

        output_dir = Path(self.get_parameter("output_dir").value)
        output_dir.mkdir(parents=True, exist_ok=True)

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._pose_topics: dict[str, str] = dict(zip(pose_names, pose_topics))
        self._trajectories: dict[str, list[dict]] = {name: [] for name in pose_names}
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

        self._exo_camera: CameraStream | None = None
        self._latest_torso_pose: dict | None = None
        exo_name = self.get_parameter("exo_camera_name").value
        if exo_name:
            if exo_name in self._cameras:
                raise ValueError(
                    f"exo_camera_name {exo_name!r} collides with a camera_names entry; "
                    "pick a different name."
                )
            self._exo_camera = CameraStream(
                name=exo_name,
                serial=self.get_parameter("exo_camera_serial").value,
                width=int(self.get_parameter("exo_camera_width").value),
                height=int(self.get_parameter("exo_camera_height").value),
                fps=int(self.get_parameter("exo_camera_fps").value),
                max_depth_m=float(self.get_parameter("exo_camera_max_depth_m").value),
                bridge=self._bridge,
                bag=self._bag,
                logger=self.get_logger(),
                clock=self.get_clock(),
                node=self,
                publish_live=True,  # exo_pose_bridge subscribes to this live, not just while recording
            )
            self._cameras[exo_name] = self._exo_camera
            self._exo_camera.start()

            self._torso_pose_topic = f"/{exo_name}/torso_pose"
            torso_group = MutuallyExclusiveCallbackGroup()
            self.create_subscription(
                PointStamped,
                self._torso_pose_topic,
                self._on_torso_pose,
                qos_profile_sensor_data,
                callback_group=torso_group,
            )

            # exo_pose_bridge publishes the color frame with the detected
            # skeleton drawn on it; shown as its own panel, not recorded
            # (it's derived/redundant with color + torso_pose, both of
            # which already are recorded).
            self._pose_overlay_jpeg: bytes | None = None
            self._pose_overlay_lock = threading.Lock()
            self._pose_overlay_ready = threading.Condition(self._pose_overlay_lock)
            overlay_group = MutuallyExclusiveCallbackGroup()
            self.create_subscription(
                CompressedImage,
                f"/{exo_name}/pose_overlay/image_raw/compressed",
                self._on_pose_overlay,
                qos_profile_sensor_data,
                callback_group=overlay_group,
            )

        # One MutuallyExclusiveCallbackGroup per device (not one shared group
        # for all pose subscriptions) so a slow callback for one UMI device
        # can't delay another's - same independence guarantee as cameras,
        # just for pose topics instead of frames.
        for name in pose_names:
            pose_group = MutuallyExclusiveCallbackGroup()
            self.create_subscription(
                PoseStamped,
                self._pose_topics[name],
                self._make_pose_callback(name),
                qos_profile_sensor_data,
                callback_group=pose_group,
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

    def _make_pose_callback(self, name: str):
        def _on_pose(msg: PoseStamped) -> None:
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
                trajectory = self._trajectories[name]
                trajectory.append(point)
                if len(trajectory) > MAX_TRAJECTORY_POINTS:
                    trajectory.pop(0)

            if self._recording:
                topic = self._pose_topics[name]
                self._bag.register_topic(topic, "geometry_msgs/msg/PoseStamped")
                self._bag.write(topic, msg, self.get_clock().now().nanoseconds)

        return _on_pose

    def _on_torso_pose(self, msg: PointStamped) -> None:
        # x/y are normalized image coordinates (0-1) from exo_pose_bridge,
        # not metric - see exo_pose_bridge's module docstring for why.
        self._latest_torso_pose = {"x": msg.point.x, "y": msg.point.y}
        if self._recording:
            self._bag.register_topic(self._torso_pose_topic, "geometry_msgs/msg/PointStamped")
            self._bag.write(self._torso_pose_topic, msg, self.get_clock().now().nanoseconds)

    def _on_pose_overlay(self, msg: CompressedImage) -> None:
        with self._pose_overlay_ready:
            self._pose_overlay_jpeg = bytes(msg.data)
            self._pose_overlay_ready.notify_all()

    # -- Recording control ---------------------------------------------------

    def start_recording(self) -> str:
        if self._recording:
            return str(self._current_bag_path)
        self._current_bag_path = self._bag.open()
        with self._lock:
            for name in self._trajectories:
                self._trajectories[name] = []
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

        # One Server-Sent Events stream carries every panel's frames, instead
        # of one MJPEG connection per panel. Browsers cap concurrent HTTP/1.1
        # connections at ~6 per origin, and each MJPEG response never
        # completes - with 3 cameras (6 panels) the cap was fully consumed by
        # video alone, so /trajectory, /pose_devices and /status queued
        # forever and the UI's pose trails never appeared. This holds one
        # connection no matter how many cameras are configured.
        @app.route("/stream")
        def stream():
            return Response(
                self._frame_event_generator(),
                mimetype="text/event-stream",
                headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
            )

        @app.route("/pose_devices")
        def pose_devices():
            return jsonify(list(self._trajectories.keys()))

        @app.route("/trajectory")
        def trajectory():
            # Keyed by pose device name, e.g. {"umi_gripper": [...]} - one
            # entry per pose_names configured, each capped at the most
            # recent MAX_TRAJECTORY_POINTS poses for that device.
            with self._lock:
                return jsonify(
                    {name: points[-MAX_TRAJECTORY_POINTS:] for name, points in self._trajectories.items()}
                )

        @app.route("/config")
        def config():
            return jsonify(
                {
                    name: {
                        "max_depth_m": camera.max_depth_m,
                        "is_exo": self._exo_camera is not None and camera is self._exo_camera,
                    }
                    for name, camera in self._cameras.items()
                }
            )

        @app.route("/torso_pose")
        def torso_pose():
            return jsonify(self._latest_torso_pose)

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

    def _frame_event_generator(self, interval_s: float = 1 / 30):
        """Yield SSE events for every panel whose JPEG changed since the last
        tick. Each event is {"id": "<camera>:<color|depth>", "jpeg": <base64>};
        the client assigns it to that panel's img as a data: URL.

        Polls rather than waiting on each camera's condition variable because
        one generator serves all cameras - the bytes objects are swapped
        atomically by the capture threads, so an unsynchronized read either
        sees the previous frame or the next one, never a torn one.
        """
        last_sent: dict[str, bytes] = {}

        while rclpy.ok():
            for name, camera in self._cameras.items():
                is_exo = self._exo_camera is not None and camera is self._exo_camera
                # The exo panel shows exo_pose_bridge's skeleton overlay in
                # place of the plain color frame (same image, annotated).
                color = self._pose_overlay_jpeg if is_exo else camera.latest_color_jpeg
                for sid, jpeg in ((f"{name}:color", color), (f"{name}:depth", camera.latest_depth_jpeg)):
                    if jpeg is None or last_sent.get(sid) is jpeg:
                        continue
                    last_sent[sid] = jpeg
                    payload = json.dumps(
                        {"id": sid, "jpeg": base64.b64encode(jpeg).decode("ascii")}
                    )
                    yield f"data: {payload}\n\n"
            time.sleep(interval_s)


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
