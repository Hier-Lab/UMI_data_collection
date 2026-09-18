"""Runs inside rgbd_odometry's dedicated venv (modern numpy + open3d), never
imported by ROS code directly. Takes color/depth/intrinsics as an .npz file
and writes back a trajectory .npz, so the numpy ABI open3d needs never has
to coexist with the numpy ABI rclpy/cv_bridge/rosbag2_py need.

Input .npz arrays:
  color: (N, H, W, 3) uint8, RGB order
  depth_mm: (N, H, W) uint16, millimeters
  stamps_ns: (N,) int64
  fx, fy, cx, cy: scalars
  width, height: scalars
  depth_trunc_m: scalar

Output .npz arrays:
  stamps_ns: (M,) int64, M = N - 1 (poses are relative to the first frame,
             which is not itself included - the caller emits identity for it)
  poses: (M, 4, 4) float64
"""

from __future__ import annotations

import sys

import numpy as np
import open3d as o3d


def main() -> int:
    in_path, out_path = sys.argv[1], sys.argv[2]
    data = np.load(in_path)

    color = data["color"]
    depth_mm = data["depth_mm"]
    stamps_ns = data["stamps_ns"]
    fx, fy, cx, cy = float(data["fx"]), float(data["fy"]), float(data["cx"]), float(data["cy"])
    width, height = int(data["width"]), int(data["height"])
    depth_trunc_m = float(data["depth_trunc_m"])

    intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
    option = o3d.pipelines.odometry.OdometryOption(depth_max=depth_trunc_m)

    out_stamps = []
    out_poses = []
    current_pose = np.eye(4)
    previous_rgbd = None

    for i in range(color.shape[0]):
        color_img = o3d.geometry.Image(np.ascontiguousarray(color[i]))
        depth_img = o3d.geometry.Image(np.ascontiguousarray(depth_mm[i]))
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_img,
            depth_img,
            depth_scale=1000.0,  # depth_mm is already normalized to millimeters
            depth_trunc=depth_trunc_m,
            convert_rgb_to_intensity=False,
        )

        if previous_rgbd is not None:
            success, trans, _info = o3d.pipelines.odometry.compute_rgbd_odometry(
                previous_rgbd,
                rgbd,
                intrinsics,
                np.eye(4),
                o3d.pipelines.odometry.RGBDOdometryJacobianFromHybridTerm(),
                option,
            )
            if success:
                current_pose = current_pose @ trans
            out_stamps.append(int(stamps_ns[i]))
            out_poses.append(current_pose.copy())

        previous_rgbd = rgbd

    np.savez(
        out_path,
        stamps_ns=np.array(out_stamps, dtype=np.int64),
        poses=np.array(out_poses, dtype=np.float64) if out_poses else np.zeros((0, 4, 4)),
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
