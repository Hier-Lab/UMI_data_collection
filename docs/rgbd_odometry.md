# `rgbd_odometry` post-processing

**`rgbd_odometry`** (Python, `ament_python`) — a standalone post-processing CLI (`ros2 run rgbd_odometry compute_odometry --bag <path> --camera <name>`), not part of the live recording pipeline. It reads one camera's `color/image_raw`, `aligned_depth_to_color/image_raw`, and `color/camera_info` from an already-recorded episode bag, computes frame-to-frame RGBD odometry with Open3D (hybrid Jacobian term — combines the geometric point-to-plane and photometric color constraints), and writes the resulting trajectory back into that same bag as a new `/<camera>/visual_odom` (`geometry_msgs/PoseStamped`) topic — one pose per color frame after the first (which gets an identity pose at the same timestamp).

```bash
ros2 run rgbd_odometry compute_odometry --bag ~/umi_episodes/episode_20260101_120000 --camera wrist --depth-trunc 0.5
```

## Why a subprocess + dedicated venv

`.venv/` (gitignored, not checked in): `open3d` 0.20 needs a numpy ABI incompatible with the numpy this ROS 2 Humble environment pins for `rclpy`/`cv_bridge`/`rosbag2_py` — `o3d.geometry.Image(numpy_array)` fails outright (`RuntimeError: Image can only be initialized from buffer of uint8, uint16, or float!`) even for a correctly-typed, contiguous, freshly-copied array, confirmed not to be an array-property issue.

`compute_odometry.py` (system Python, has ROS access) extracts and pairs color/depth/intrinsics from the bag, writes them to a temporary `.npz`, and invokes `_venv_odometry.py` as a subprocess inside `.venv` (modern numpy + open3d) via `subprocess.run`; the venv script does the actual Open3D math and writes a trajectory `.npz` back.

Set up the venv with:

```bash
python3 -m pip install --user virtualenv   # if `python3 -m venv` isn't available
virtualenv ros2_ws/src/rgbd_odometry/.venv
ros2_ws/src/rgbd_odometry/.venv/bin/pip install open3d numpy
```

## Other notes

- `rotation_matrix_to_quaternion()` in `compute_odometry.py` is a hand-rolled Shepperd's-method matrix→quaternion conversion, not `scipy.spatial.transform.Rotation` — deliberately, to avoid pulling another numpy-ABI-sensitive dependency into the system-Python side of this split.
- `bag_io.py`'s `AugmentedBagWriter` exists because `rosbag2_py.SequentialWriter` is write-once — there's no API to append a topic to an already-closed bag. It copies every message from the source bag into a new temp bag (preserving original topics/types), lets the caller `write_extra()` the new topic's messages interleaved, then `finalize()` closes the temp bag and atomically replaces the source bag directory with it. This means running `compute_odometry` twice on the same bag/camera is safe (it re-reads whatever is currently there and appends `visual_odom` again) but destructive if interrupted mid-`finalize()` — the source is deleted before the temp bag is moved into place.
- Requires the bag to already have `/<camera>/color/camera_info` (added to `umi_recorder` for exactly this reason); a bag recorded before that change has no way to recover real intrinsics and `compute_odometry` will refuse to run against it with a clear error.
- Verified end-to-end against a real 5-second D405 recording (151 color / 150 depth frames): ran successfully, wrote 151 `visual_odom` poses, and the resulting positions/quaternions are physically plausible (near-identity, matching the camera being essentially stationary during that test) with no NaNs or discontinuities.
