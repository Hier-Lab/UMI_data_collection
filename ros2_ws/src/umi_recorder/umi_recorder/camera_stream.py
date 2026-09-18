"""Single RealSense camera capture thread: pyrealsense2 pipeline (color+depth
aligned to color, no point cloud) feeding a live MJPEG buffer and, when the
owning node is recording, a ROS 2 bag via a shared EpisodeBagWriter.
"""

from __future__ import annotations

import threading

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy


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
    ):
        self.name = name
        self.serial = serial
        self.color_topic = f"/{name}/color/image_raw"
        self.depth_topic = f"/{name}/aligned_depth_to_color/image_raw"

        self._width = width
        self._height = height
        self._fps = fps
        self.max_depth_m = max_depth_m
        self._bridge = bridge
        self._bag = bag
        self._logger = logger
        self._clock = clock

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
        self._wait_for_device()
        if self._stop.is_set():
            return

        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(self.serial)
        config.enable_stream(rs.stream.color, self._width, self._height, rs.format.bgr8, self._fps)
        config.enable_stream(rs.stream.depth, self._width, self._height, rs.format.z16, self._fps)
        align = rs.align(rs.stream.color)

        profile = pipeline.start(config)
        # Depth scale (meters per raw depth unit) varies by camera model - e.g.
        # ~0.001 (1mm/unit) on a D435i vs ~0.0001 (0.1mm/unit) on a D405.
        # Treating raw units as millimeters unconditionally silently produced
        # a 10x-wrong depth reading on the D405 (everything looked "far").
        depth_scale_m = profile.get_device().first_depth_sensor().get_depth_scale()
        self._logger.info(
            f"[{self.name}] pyrealsense2 pipeline started ({self._width}x{self._height}"
            f"@{self._fps}, serial={self.serial}, depth_scale={depth_scale_m}m/unit)"
        )
        try:
            while not self._stop.is_set() and rclpy.ok():
                frames = pipeline.wait_for_frames(timeout_ms=1000)
                frames = align.process(frames)
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue

                color = np.asanyarray(color_frame.get_data())
                depth_raw = np.asanyarray(depth_frame.get_data())
                # Normalize to millimeters regardless of camera model, so
                # colorization and recorded bags are consistent across cameras.
                depth_mm = (depth_raw.astype(np.float32) * depth_scale_m * 1000.0).astype(np.uint16)
                stamp = self._clock.now()
                self._handle_color(color, stamp)
                self._handle_depth(depth_mm, stamp)
        except Exception:
            self._logger.exception(f"[{self.name}] pyrealsense2 capture loop stopped")
        finally:
            pipeline.stop()

    def _handle_color(self, color_bgr: np.ndarray, stamp) -> None:
        ok, jpeg = cv2.imencode(".jpg", color_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if ok:
            with self.color_ready:
                self.latest_color_jpeg = jpeg.tobytes()
                self.color_ready.notify_all()

        if self._recording:
            msg = self._bridge.cv2_to_imgmsg(color_bgr, encoding="bgr8")
            msg.header.stamp = stamp.to_msg()
            self._bag.register_topic(self.color_topic, "sensor_msgs/msg/Image")
            self._bag.write(self.color_topic, msg, stamp.nanoseconds)

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
