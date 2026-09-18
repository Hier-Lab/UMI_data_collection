# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Goal

A data collection UI for UMI hand-grippers that records human demonstrations to train a VLA (vision-language-action) model. Each demonstration is a synchronized recording of hand trajectory (mocap) and RGB-D video (RealSense), saved as a ROS 2 bag.

## Commands

First-time setup:

```bash
# mocap_bridge pulls in mocap_subscriber as a git submodule
git submodule update --init --recursive

# umi_recorder captures RealSense frames directly via pyrealsense2
# (not through the realsense2_camera ROS node) - install the Python bindings:
pip3 install pyrealsense2

# rgbd_odometry needs open3d + a modern numpy, which conflict with the
# numpy this ROS 2 Humble environment pins - they run in a dedicated venv:
python3 -m pip install --user virtualenv   # if `python3 -m venv` isn't available
virtualenv ros2_ws/src/rgbd_odometry/.venv
ros2_ws/src/rgbd_odometry/.venv/bin/pip install open3d numpy
```

Build (run from `ros2_ws/`):

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Launch everything (mocap bridge + recorder web UI, which captures the RealSense camera directly):

```bash
ros2 launch umi_recorder umi_recorder.launch.py
```

Then open `http://<host>:8080` for the Record/Stop UI. Recorded episodes land in `output_dir` (see config) as ROS 2 bags.

All settings (mocap IP/ports, tracked rigid bodies, camera/pose topics, recording output, web host/port) live in one file: [`ros2_ws/src/umi_recorder/config/umi_data_collection.yaml`](ros2_ws/src/umi_recorder/config/umi_data_collection.yaml), with inline comments. Edit it directly, or point the launch file at your own copy without touching the installed one:

```bash
ros2 launch umi_recorder umi_recorder.launch.py umi_config:=/path/to/my_umi_data_collection.yaml
```

Run nodes individually for debugging (both read the same file, each using only its own top-level section):

```bash
ros2 run mocap_bridge mocap_bridge_node --ros-args --params-file $(ros2 pkg prefix umi_recorder)/share/umi_recorder/config/umi_data_collection.yaml
ros2 run umi_recorder recorder_node --ros-args --params-file $(ros2 pkg prefix umi_recorder)/share/umi_recorder/config/umi_data_collection.yaml
```

Inspect a recorded bag:

```bash
ros2 bag info ~/umi_episodes/episode_20260101_120000
```

Post-process one camera of a recorded episode into RGBD visual odometry (writes `/<camera>/visual_odom` back into that same bag — see [rgbd_odometry](#rgbd_odometry-post-processing) below):

```bash
ros2 run rgbd_odometry compute_odometry --bag ~/umi_episodes/episode_20260101_120000 --camera wrist --depth-trunc 0.5
```

## Architecture

Workspace: `ros2_ws/src/`, three packages.

**`mocap_bridge`** (C++, `ament_cmake`) — wraps the NatNet client from [mocap_subscriber](https://github.com/lgkimjy/mocap_subscriber), pulled in as a git submodule at `third_party/mocap_subscriber/` (built via `add_subdirectory`, so it tracks a pinned commit — bump the submodule and commit the pointer to pick up upstream changes) and publishes `geometry_msgs/PoseStamped` per tracked rigid body on `mocap/<name>/pose`. Builds a `mocap_subscriber::NatNetConfig` directly from ROS params (`server_address`, `local_address`, `use_multicast`, `command_port`, `data_port`, `rigid_body_ids`, `rigid_body_names` — the last two are matched by position, so tracking multiple objects means extending both lists together) rather than reading `mocap_subscriber`'s own file format. Polls `NatNetClient::latestPoseById()` on a timer and skips publishing when `isPoseFresh()` is false rather than publishing stale poses.

**`umi_recorder`** (Python, `ament_python`):
- `camera_stream.py`'s `CameraStream` owns one RealSense camera end-to-end: opens it directly via `pyrealsense2` on its own background thread (`_run`), bypassing the `realsense2_camera` ROS node — that ROS wrapper was measured to degrade from ~30fps to ~5-8fps within seconds under this exact workload (config, resolution, CPU near-idle), while a raw `pyrealsense2`/`librealsense2` pipeline with the identical stream config sustains a steady ~30fps indefinitely. Uses `rs.align(rs.stream.color)` to align depth to color; no point cloud is computed. Before `pipeline.start()`, `_wait_for_device()` polls `rs.context().query_devices()` for the configured serial and logs a warning + retries every 2s if it's missing, rather than letting `pipeline.start()` block forever with zero feedback (a real failure mode if a serial is wrong or a camera is unplugged). Immediately after `pipeline.start()`, reads `first_depth_sensor().get_depth_scale()` and converts every depth frame to millimeters using that scale before colorizing or recording — depth scale (meters/raw-unit) is camera-model-specific (e.g. ~0.001 on a D435i, ~0.0001 on a D405) and assuming millimeters unconditionally silently produced 10x-wrong depth on a D405 (verified: near objects read as "far" and colorized almost entirely dark/clipped until this was fixed). Every depth frame also runs through librealsense's standard denoising pipeline — `disparity_transform(True)` → `spatial_filter()` → `temporal_filter()` → `disparity_transform(False)` — applied to both the live preview and the recorded bag (deliberately no hole-filling filter: that would extrapolate depth for pixels with no real return, and a policy shouldn't train on fabricated depth). `temporal_filter` keeps internal state across frames, so one filter chain is created per `CameraStream` and reused for the camera's lifetime, never recreated per-frame.
- `recorder_node.py`'s `RecorderNode` owns up to `MAX_CAMERAS` (4) `CameraStream`s, built from parallel `camera_names`/`camera_serials`/`camera_widths`/`camera_heights`/`camera_fps`/`camera_max_depth_m` ROS params (all matched by position — same convention as `mocap_bridge_node`'s `rigid_body_ids`/`rigid_body_names`), rejecting more than 4 cameras or any length mismatch across those lists at startup with a clear `ValueError`. Resolution/fps/max-depth are per-camera (not shared) specifically to support mixed rigs, e.g. a close-range D405 (usable range roughly 7-50cm) alongside a wider-range D435i (30cm-3m+) with different `max_depth_m` each.
- Also subscribes to the mocap bridge's `PoseStamped` topic over ROS (`pose_topic` param — must match `/mocap/<rigid_body_names[i]>/pose` from `mocap_bridge_node`'s config), using `qos_profile_sensor_data` (best-effort) to match `mocap_bridge_node`'s `SensorDataQoS` publisher — a reliable subscriber (rclpy's default from a plain integer queue depth) never connects to a best-effort publisher in DDS, so this must stay best-effort or the topic silently stops delivering messages with no error.
- Runs a Flask app in a background thread (started from `Node.__init__`) alongside a `MultiThreadedExecutor` on the main thread for the pose subscription. `/cameras` lists configured camera names; `/video_feed/<name>` and `/depth_feed/<name>` are MJPEG generators, one pair per camera, each blocking on that `CameraStream`'s own `threading.Condition` (`color_ready`/`depth_ready`) instead of polling on a fixed timer — each new frame calls `notify_all()`, so a stream pushes a frame exactly when one arrives. `/depth_feed/<name>` colorizes with `cv2.COLORMAP_JET`, clipped/normalized against `max_depth_m` (near = bright, beyond `max_depth_m` or no-return = dark/black) — see `CameraStream._handle_depth`.
- `/trajectory` returns the last `MAX_TRAJECTORY_POINTS` (20) pose points as JSON `{x,y,z,qx,qy,qz,qw}`, rendered in `templates/index.html` via Three.js: a 3D trail with a small dot per sample (visible even for sub-millimeter motion) plus an `AxesHelper` orientation gizmo at the current pose, rotated by its quaternion — not a flat 2D projection.
- `/config` returns `{name: {max_depth_m}}` per camera so the frontend can label each depth panel's range without hardcoding it. The frontend fetches `/config` then `/cameras` on load and renders one color+depth panel pair per name (`loadCameraPanels()` in `index.html`), so panel count and per-panel labels follow config with no template changes needed.
- Record/Stop (`/record`, `/stop` POST endpoints) toggle recording; `RecorderNode.start_recording`/`stop_recording` call `CameraStream.set_recording()` on every configured camera. Each `CameraStream` writes its own topics into the shared `EpisodeBagWriter` (`bag_writer.py`, a thin `rosbag2_py.SequentialWriter` wrapper) — namespaced by camera name, e.g. `/<name>/color/image_raw`, `/<name>/aligned_depth_to_color/image_raw` — alongside the pose topic, wrapped into `sensor_msgs/Image` via `cv_bridge.cv2_to_imgmsg` (raw depth in millimeters, not the colorized visualization). Each camera also writes a `/<name>/color/camera_info` `sensor_msgs/CameraInfo` message once per recording (real intrinsics queried from the color stream's `VideoStreamProfile` at pipeline start — `CameraStream._build_camera_info`), so a bag is self-contained for anything needing real fx/fy/cx/cy downstream (e.g. RGBD odometry). One bag per episode is fully self-contained and time-synchronized on ROS-clock timestamps across every camera plus mocap.
- `CameraStream._run` is a two-level retry loop: the outer loop calls `_wait_for_device()` then `_run_pipeline()`, and any exception from `_run_pipeline()` (device unplugged, driver error) is caught, logged, and retried after a 2s backoff rather than permanently ending that camera's thread. Inside `_run_pipeline()`, a single `wait_for_frames()` timeout (`RuntimeError`) is treated as transient and the loop just continues — only an exception outside that inner try/except tears the pipeline down for a full reconnect. This matters because `CameraStream`s are fully independent: one camera failing (wrong/missing serial, mid-session disconnect) must never affect any other configured camera, the web UI, or the pose subscription — verified by running with one real camera + one nonexistent serial simultaneously (the real one streamed at a steady 30fps the whole time while the missing one just logged and waited).

Both nodes' parameters live together in one file, `umi_recorder/config/umi_data_collection.yaml`, as separate `mocap_bridge_node:`/`umi_recorder_node:` top-level sections (ROS 2's standard multi-node YAML params format — each node only reads its own section, so passing the whole file to both is intentional, not a bug). `launch/umi_recorder.launch.py` brings up only `mocap_bridge_node` and `recorder_node` (no `realsense2_camera` node — the camera is owned exclusively by `recorder_node`'s pyrealsense2 pipeline, and only one process can hold the USB device at a time), and exposes the config file as a single launch argument (`umi_config`).

### `rgbd_odometry` post-processing

**`rgbd_odometry`** (Python, `ament_python`) — a standalone post-processing CLI (`ros2 run rgbd_odometry compute_odometry --bag <path> --camera <name>`), not part of the live recording pipeline. It reads one camera's `color/image_raw`, `aligned_depth_to_color/image_raw`, and `color/camera_info` from an already-recorded episode bag, computes frame-to-frame RGBD odometry with Open3D (hybrid Jacobian term — combines the geometric point-to-plane and photometric color constraints), and writes the resulting trajectory back into that same bag as a new `/<camera>/visual_odom` (`geometry_msgs/PoseStamped`) topic — one pose per color frame after the first (which gets an identity pose at the same timestamp).

- **Why a subprocess + dedicated venv** (`.venv/`, gitignored, not checked in): `open3d` 0.20 needs a numpy ABI incompatible with the numpy this ROS 2 Humble environment pins for `rclpy`/`cv_bridge`/`rosbag2_py` — `o3d.geometry.Image(numpy_array)` fails outright (`RuntimeError: Image can only be initialized from buffer of uint8, uint16, or float!`) even for a correctly-typed, contiguous, freshly-copied array, confirmed not to be an array-property issue. `compute_odometry.py` (system Python, has ROS access) extracts and pairs color/depth/intrinsics from the bag, writes them to a temporary `.npz`, and invokes `_venv_odometry.py` as a subprocess inside `.venv` (modern numpy + open3d) via `subprocess.run`; the venv script does the actual Open3D math and writes a trajectory `.npz` back. Set up the venv with `virtualenv .venv && .venv/bin/pip install open3d numpy` (needs `pip install --user virtualenv` first if `python3 -m venv` isn't available, which it wasn't on this platform — no `python3-venv` package and no sudo to install it).
- `rotation_matrix_to_quaternion()` in `compute_odometry.py` is a hand-rolled Shepperd's-method matrix→quaternion conversion, not `scipy.spatial.transform.Rotation` — deliberately, to avoid pulling another numpy-ABI-sensitive dependency into the system-Python side of this split.
- `bag_io.py`'s `AugmentedBagWriter` exists because `rosbag2_py.SequentialWriter` is write-once — there's no API to append a topic to an already-closed bag. It copies every message from the source bag into a new temp bag (preserving original topics/types), lets the caller `write_extra()` the new topic's messages interleaved, then `finalize()` closes the temp bag and atomically replaces the source bag directory with it. This means running `compute_odometry` twice on the same bag/camera is safe (it re-reads whatever is currently there and appends `visual_odom` again) but destructive if interrupted mid-`finalize()` — the source is deleted before the temp bag is moved into place.
- Requires the bag to already have `/<camera>/color/camera_info` (added to `umi_recorder` for exactly this reason — see above); a bag recorded before that change has no way to recover real intrinsics and `compute_odometry` will refuse to run against it with a clear error.
- Verified end-to-end against a real 5-second D405 recording (151 color / 150 depth frames): ran successfully, wrote 151 `visual_odom` poses, and the resulting positions/quaternions are physically plausible (near-identity, matching the camera being essentially stationary during that test) with no NaNs or discontinuities.

## Notes

- Quaternion order from the mocap client is `(w, x, y, z)` (Eigen convention) — `mocap_bridge_node.cpp`'s `toPoseStamped` maps this correctly into `geometry_msgs/Quaternion` (which is `x,y,z,w`); double-check this mapping first if orientation looks wrong downstream.
- If Motive is set to `Z Up`, do not apply an additional coordinate transform in `mocap_bridge` or `umi_recorder`.
- The bag writer only registers a topic the first time a message for it arrives after `open()` — if a topic never publishes during an episode (e.g. mocap dropout), it simply won't appear in that bag.
- `mocap_bridge_node`'s `rigid_body_names` and `umi_recorder_node`'s `pose_topic` are not automatically linked — if you rename the tracked rigid body or change its id, update `pose_topic` (`/mocap/<name>/pose`) to match, or `umi_recorder_node` will never receive pose data.
- `templates/index.html` loads Three.js from cdnjs (pinned to r128) — the recorder's web UI needs internet access on first load (browser-cached afterward) even though everything else runs fully offline/local.
- The `realsense2_camera` ROS node was dropped from this project after measuring it degrade from ~30fps to ~5-8fps within ~15-20 seconds on this hardware (config: 640x480@30 color+depth+align), even running alone with the CPU at ~97% idle — while a raw `pyrealsense2`/`librealsense2` pipeline with the identical config sustains a steady ~30fps indefinitely. The camera and USB link (confirmed native USB 3.0, `lsusb -t` shows `5000M`) were never the bottleneck; the ROS wrapper's message publishing path was. If frame rate regresses again, benchmark with a minimal `rs2::pipeline`/`pyrealsense2` script first to rule the SDK in or out before suspecting `recorder_node`.
- Only one process can hold a given RealSense camera at a time (`rs2::backend_error ... Device or resource busy`). Before running `recorder_node` standalone for debugging, make sure no other `realsense2_camera_node`, `rs_bench`-style script, or leftover `recorder_node` instance still has that serial open.
- Find each camera's serial number for `camera_serials` with `rs-enumerate-devices` (from `librealsense2-utils`, or `/opt/ros/humble/bin/rs-enumerate-devices` if only the ROS package is installed) — look for the `Serial Number` field under each device's info block. A `CameraStream` for a serial that never appears logs one warning and then retries every 2s indefinitely rather than blocking `recorder_node` startup or the web UI.
- `package.xml` is parsed by `colcon-ros`, and if it fails to parse as valid XML, colcon silently falls back to treating the package as generic Python (`python`) rather than `ros.ament_python` — the build still "succeeds" with no visible error, but the package never gets registered in `AMENT_PREFIX_PATH`, so `ros2 run`/`ros2 pkg prefix`/`get_package_share_directory` all fail with "package not found". A `<!-- comment -->` containing a bare `--` (e.g. a CLI flag like `--user` in explanatory text) is invalid XML and will trigger this silently — check `colcon --log-level debug build` output for a "Failed to parse potential ROS package manifest" line if a package mysteriously can't be found after a build that reported success.
- The D405 has no `emitter_enabled`/`laser_power` options on its depth sensor (`depth_sensor.supports(...)` returns `False` for both) — unlike the D435i, it's a passive/short-range stereo design without a toggleable IR laser projector, so there is no IR emitter to turn on for it. Don't try to wire up emitter control for a D405 entry in `camera_serials`; check `depth_sensor.supports(rs.option.emitter_enabled)` per-device first if this ever needs to support a camera that does have one.
- `RcutilsLogger` (the object `Node.get_logger()` returns) has no `.exception()` method — that's plain Python `logging.Logger` API. Calling it crashes with `AttributeError` inside whatever `except` block called it, which is especially bad news in an exception handler meant to keep a camera thread alive: the intended error path itself throws a new, unhandled exception, silently killing the thread instead of logging and continuing. Use `logger.error(f"...: {traceback.format_exc()}")` instead.
- Never run two processes (or two `CameraStream`s) against the same camera serial at once. Beyond the expected `rs2::backend_error ... Device or resource busy`, we observed the D405 get stuck after such a collision — `pipeline.start()` succeeded but `wait_for_frames()` timed out indefinitely afterward, even from a single clean process with no other contender running. `device.hardware_reset()` (via `rs.context().query_devices()[0].hardware_reset()`, or physically unplug/replug) recovered it.
