"""Single RealSense camera capture thread: pyrealsense2 pipeline (color+depth
aligned to color, no point cloud) feeding a live MJPEG buffer and, when the
owning node is recording, a ROS 2 bag via a shared EpisodeBagWriter.
"""

from __future__ import annotations

import threading
import traceback

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


class CameraStream:
    def __init__(
        self,
        name: str,
        serial: str,
        width: int,
        height: int,
        fps: int,
        max_depth_m: float,
        bridge,
        bag,
        logger,
        clock,
        node=None,
        publish_live: bool = False,
    ):
        self.name = name
        self.serial = serial
        self.color_topic = f"/{name}/color/image_raw"
        self.depth_topic = f"/{name}/aligned_depth_to_color/image_raw"
        self.camera_info_topic = f"/{name}/color/camera_info"
        self._camera_info_msg: CameraInfo | None = None
        self._camera_info_written_this_recording = False

        self._width = width
        self._height = height
        self._fps = fps
        self.max_depth_m = max_depth_m
        self._bridge = bridge
        self._bag = bag
        self._logger = logger
        self._clock = clock

        # Normally color frames only go out as JPEG (for the MJPEG preview)
        # and, when recording, into the bag - there's no live ROS topic, to
        # avoid publishing overhead on cameras nothing subscribes to. A
        # camera that something needs to subscribe to live (e.g. a pose
        # estimation node) opts in via publish_live=True.
        #
        # Published as CompressedImage (reusing the JPEG already produced
        # for the MJPEG preview), not raw Image: publishing raw 640x480
        # bgr8 frames (~0.9MB each) at 30fps through rclpy/DDS measured
        # only ~3-5fps actually delivered with the subscriber's CPU barely
        # loaded - the same class of ROS-transport overhead that got
        # realsense2_camera dropped from this project, just triggered by a
        # different code path. JPEG cuts message size ~20x and restored a
        # steady 30fps.
        self.live_color_topic = f"/{name}/color/image_raw/compressed"
        self._live_color_pub = None
        if publish_live:
            if node is None:
                raise ValueError("publish_live=True requires a node to create the publisher on")
            self._live_color_pub = node.create_publisher(CompressedImage, self.live_color_topic, 10)

        self._lock = threading.Lock()
        self.color_ready = threading.Condition(self._lock)
        self.depth_ready = threading.Condition(self._lock)
        self.latest_color_jpeg: bytes | None = None
        self.latest_depth_jpeg: bytes | None = None

        self._recording = False
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)

    def set_recording(self, recording: bool) -> None:
        self._recording = recording
        if recording:
            self._camera_info_written_this_recording = False

    def _build_camera_info(self, profile) -> CameraInfo:
        """Build a sensor_msgs/CameraInfo from the color stream's real
        intrinsics (aligned depth shares the color stream's intrinsics)."""
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        msg = CameraInfo()
        msg.width = intr.width
        msg.height = intr.height
        msg.distortion_model = "plumb_bob"
        msg.d = list(intr.coeffs)
        msg.k = [intr.fx, 0.0, intr.ppx, 0.0, intr.fy, intr.ppy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [intr.fx, 0.0, intr.ppx, 0.0, 0.0, intr.fy, intr.ppy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

    def _wait_for_device(self, poll_interval_s: float = 2.0) -> None:
        """Block until self.serial is enumerated, logging periodically instead
        of letting pipeline.start() hang silently with no feedback."""
        logged_wait = False
        while not self._stop.is_set() and rclpy.ok():
            serials = [
                d.get_info(rs.camera_info.serial_number) for d in rs.context().query_devices()
            ]
            if self.serial in serials:
                return
            if not logged_wait:
                self._logger.warning(
                    f"[{self.name}] camera with serial={self.serial!r} not found "
                    f"(detected: {serials or 'none'}); waiting..."
                )
                logged_wait = True
            self._stop.wait(poll_interval_s)

    def _run(self) -> None:
        # Outer loop: (re)connect to the camera. A pipeline-level failure
        # (camera unplugged mid-session, driver error) logs and retries
        # after a short backoff instead of permanently ending this camera's
        # thread - one camera dying must never affect any other CameraStream
        # or stop the web UI/other cameras from working.
        while not self._stop.is_set() and rclpy.ok():
            self._wait_for_device()
            if self._stop.is_set():
                return
            try:
                self._run_pipeline()
            except Exception:
                self._logger.error(
                    f"[{self.name}] pyrealsense2 pipeline failed, will retry: "
                    f"{traceback.format_exc()}"
                )
            if not self._stop.is_set():
                self._stop.wait(2.0)

    def _run_pipeline(self) -> None:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(self.serial)
        config.enable_stream(rs.stream.color, self._width, self._height, rs.format.bgr8, self._fps)
        config.enable_stream(rs.stream.depth, self._width, self._height, rs.format.z16, self._fps)
        align = rs.align(rs.stream.color)

        # Standard librealsense depth-denoising pipeline, applied in the
        # order the SDK documents: depth -> disparity (denoising filters
        # work better in disparity space) -> spatial -> temporal ->
        # disparity -> depth. Deliberately no hole-filling filter: that
        # would extrapolate depth for pixels with no real return, and
        # a downstream policy should not be trained on fabricated depth.
        depth_to_disparity = rs.disparity_transform(True)
        spatial_filter = rs.spatial_filter()
        temporal_filter = rs.temporal_filter()
        disparity_to_depth = rs.disparity_transform(False)

        try:
            profile = pipeline.start(config)
            # Depth scale (meters per raw depth unit) varies by camera model - e.g.
            # ~0.001 (1mm/unit) on a D435i vs ~0.0001 (0.1mm/unit) on a D405.
            # Treating raw units as literal millimeters unconditionally silently
            # produced a 10x-wrong depth reading on the D405 (everything looked "far").
            depth_scale_m = profile.get_device().first_depth_sensor().get_depth_scale()
            self._camera_info_msg = self._build_camera_info(profile)
            self._logger.info(
                f"[{self.name}] pyrealsense2 pipeline started ({self._width}x{self._height}"
                f"@{self._fps}, serial={self.serial}, depth_scale={depth_scale_m}m/unit)"
            )

            while not self._stop.is_set() and rclpy.ok():
                try:
                    frames = pipeline.wait_for_frames(timeout_ms=1000)
                except RuntimeError:
                    # Transient: a single frame was late/dropped. Keep the
                    # pipeline alive and try again rather than tearing it
                    # down over what is usually a one-off hiccup.
                    self._logger.warning(f"[{self.name}] frame wait timed out; continuing")
                    continue

                frames = align.process(frames)
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue

                depth_frame = depth_to_disparity.process(depth_frame)
                depth_frame = spatial_filter.process(depth_frame)
                depth_frame = temporal_filter.process(depth_frame)
                depth_frame = disparity_to_depth.process(depth_frame)

                color = np.asanyarray(color_frame.get_data())
                depth_raw = np.asanyarray(depth_frame.get_data())
                # Normalize to millimeters regardless of camera model, so
                # colorization and recorded bags are consistent across cameras.
                depth_mm = (depth_raw.astype(np.float32) * depth_scale_m * 1000.0).astype(np.uint16)
                stamp = self._clock.now()
                self._handle_color(color, stamp)
                self._handle_depth(depth_mm, stamp)
        finally:
            pipeline.stop()

    def _handle_color(self, color_bgr: np.ndarray, stamp) -> None:
        ok, jpeg = cv2.imencode(".jpg", color_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        jpeg_bytes = jpeg.tobytes() if ok else None
        if jpeg_bytes is not None:
            with self.color_ready:
                self.latest_color_jpeg = jpeg_bytes
                self.color_ready.notify_all()

        if self._live_color_pub is not None and jpeg_bytes is not None:
            live_msg = CompressedImage()
            live_msg.header.stamp = stamp.to_msg()
            live_msg.format = "jpeg"
            live_msg.data = jpeg_bytes
            self._live_color_pub.publish(live_msg)

        if self._recording:
            msg = self._bridge.cv2_to_imgmsg(color_bgr, encoding="bgr8")
            msg.header.stamp = stamp.to_msg()
            self._bag.register_topic(self.color_topic, "sensor_msgs/msg/Image")
            self._bag.write(self.color_topic, msg, stamp.nanoseconds)

            if not self._camera_info_written_this_recording and self._camera_info_msg is not None:
                self._camera_info_msg.header.stamp = stamp.to_msg()
                self._bag.register_topic(self.camera_info_topic, "sensor_msgs/msg/CameraInfo")
                self._bag.write(self.camera_info_topic, self._camera_info_msg, stamp.nanoseconds)
                self._camera_info_written_this_recording = True

    def _handle_depth(self, depth_mm: np.ndarray, stamp) -> None:
        max_depth_mm = self.max_depth_m * 1000.0
        normalized = np.clip(depth_mm.astype(np.float32), 0, max_depth_mm) / max_depth_mm
        gray = (255 - (normalized * 255)).astype(np.uint8)  # near = bright, far/clipped = dark
        colorized = cv2.applyColorMap(gray, cv2.COLORMAP_JET)
        colorized[depth_mm == 0] = 0  # no-return pixels stay black instead of "far"
        ok, jpeg = cv2.imencode(".jpg", colorized, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if ok:
            with self.depth_ready:
                self.latest_depth_jpeg = jpeg.tobytes()
                self.depth_ready.notify_all()

        if self._recording:
            msg = self._bridge.cv2_to_imgmsg(depth_mm, encoding="16UC1")
            msg.header.stamp = stamp.to_msg()
            self._bag.register_topic(self.depth_topic, "sensor_msgs/msg/Image")
            self._bag.write(self.depth_topic, msg, stamp.nanoseconds)
