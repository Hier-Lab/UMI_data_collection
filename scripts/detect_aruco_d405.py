#!/usr/bin/env python3
"""Open a RealSense D405, detect ArUco markers in the color stream, and show
the result live with imshow. Standalone debug script - no ROS involved.

Usage: python3 detect_aruco_d405.py [--serial SERIAL] [--dict DICT_NAME]
"""

import argparse

import cv2
import cv2.aruco as aruco
import numpy as np
import pyrealsense2 as rs

ARUCO_DICTS = {
    "4x4_50": aruco.DICT_4X4_50,
    "5x5_100": aruco.DICT_5X5_100,
    "6x6_250": aruco.DICT_6X6_250,
    "original": aruco.DICT_ARUCO_ORIGINAL,
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", default=None, help="D405 serial number (default: first device found)")
    parser.add_argument("--dict", default="4x4_50", choices=sorted(ARUCO_DICTS), help="ArUco dictionary to detect")
    args = parser.parse_args()

    aruco_dict = aruco.Dictionary_get(ARUCO_DICTS[args.dict])
    aruco_params = aruco.DetectorParameters_create()

    pipeline = rs.pipeline()
    config = rs.config()
    if args.serial:
        config.enable_device(args.serial)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    print("Streaming - press 'q' to quit.")
    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            image = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            corners, ids, _rejected = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
            if ids is not None:
                aruco.drawDetectedMarkers(image, corners, ids)

            cv2.imshow("D405 - ArUco detection", image)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
