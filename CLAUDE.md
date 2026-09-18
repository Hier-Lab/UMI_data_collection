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

## Architecture

Workspace: `ros2_ws/src/`, two packages.

**`mocap_bridge`** (C++, `ament_cmake`) — wraps the NatNet client from [mocap_subscriber](https://github.com/lgkimjy/mocap_subscriber), pulled in as a git submodule at `third_party/mocap_subscriber/` (built via `add_subdirectory`, so it tracks a pinned commit — bump the submodule and commit the pointer to pick up upstream changes) and publishes `geometry_msgs/PoseStamped` per tracked rigid body on `mocap/<name>/pose`. Builds a `mocap_subscriber::NatNetConfig` directly from ROS params (`server_address`, `local_address`, `use_multicast`, `command_port`, `data_port`, `rigid_body_ids`, `rigid_body_names` — the last two are matched by position, so tracking multiple objects means extending both lists together) rather than reading `mocap_subscriber`'s own file format. Polls `NatNetClient::latestPoseById()` on a timer and skips publishing when `isPoseFresh()` is false rather than publishing stale poses.

**`umi_recorder`** (Python, `ament_python`) — the recorder + web UI, all in `recorder_node.py`:
- Captures color+depth directly from the camera via `pyrealsense2` on a dedicated background thread (`_camera_loop`), not through the `realsense2_camera` ROS node — that ROS wrapper was measured to degrade from ~30fps to ~5-8fps within seconds under this exact workload (config, resolution, CPU near-idle), while a raw `pyrealsense2`/`librealsense2` pipeline with the identical stream config sustains a steady ~30fps indefinitely. Uses `rs.align(rs.stream.color)` to align depth to color; no point cloud is computed. `camera_width`/`camera_height`/`camera_fps` are ROS params (default 640x480x30).
- Subscribes only to the mocap bridge's `PoseStamped` topic over ROS (`pose_topic` param — must match `/mocap/<rigid_body_names[i]>/pose` from `mocap_bridge_node`'s config), using `qos_profile_sensor_data` (best-effort) to match `mocap_bridge_node`'s `SensorDataQoS` publisher — a reliable subscriber (rclpy's default from a plain integer queue depth) never connects to a best-effort publisher in DDS, so this must stay best-effort or the topic silently stops delivering messages with no error.
- Runs a Flask app in a background thread (started from `Node.__init__`) alongside a `MultiThreadedExecutor` on the main thread for the pose subscription — all share `RecorderNode` state (latest color/depth JPEG, trajectory buffer, recording flag) behind a lock.
- `/video_feed` and `/depth_feed` are MJPEG generators that block on a `threading.Condition` (`_color_ready`/`_depth_ready`) instead of polling on a fixed timer — each new frame calls `notify_all()`, so a stream pushes a frame exactly when one arrives rather than re-sending/re-locking 30x/sec regardless of whether anything changed. `/depth_feed` colorizes with `cv2.COLORMAP_JET`, clipped/normalized against `max_depth_m` (near = bright, beyond `max_depth_m` or no-return = dark/black) — see `_handle_depth`.
- `/trajectory` returns the last `MAX_TRAJECTORY_POINTS` (20) pose points as JSON `{x,y,z}`, rendered as a 3D trail in `templates/index.html` via Three.js (orbit camera, grid, axes — not a flat 2D projection), plus a marker at the current pose.
- `/config` exposes `max_depth_m` so the frontend can label the depth panel's range without hardcoding it.
- Record/Stop (`/record`, `/stop` POST endpoints) toggle `EpisodeBagWriter` (`bag_writer.py`), a thin `rosbag2_py.SequentialWriter` wrapper — one bag per episode, topics registered lazily on first message once recording starts. Color/depth frames captured from `pyrealsense2` are wrapped into `sensor_msgs/Image` via `cv_bridge.cv2_to_imgmsg` before being written (raw depth in millimeters, not the colorized visualization), alongside the pose topic, so an episode is fully self-contained and time-synchronized on ROS-clock timestamps.

Both nodes' parameters live together in one file, `umi_recorder/config/umi_data_collection.yaml`, as separate `mocap_bridge_node:`/`umi_recorder_node:` top-level sections (ROS 2's standard multi-node YAML params format — each node only reads its own section, so passing the whole file to both is intentional, not a bug). `launch/umi_recorder.launch.py` brings up only `mocap_bridge_node` and `recorder_node` (no `realsense2_camera` node — the camera is owned exclusively by `recorder_node`'s pyrealsense2 pipeline, and only one process can hold the USB device at a time), and exposes the config file as a single launch argument (`umi_config`).

## Notes

- Quaternion order from the mocap client is `(w, x, y, z)` (Eigen convention) — `mocap_bridge_node.cpp`'s `toPoseStamped` maps this correctly into `geometry_msgs/Quaternion` (which is `x,y,z,w`); double-check this mapping first if orientation looks wrong downstream.
- If Motive is set to `Z Up`, do not apply an additional coordinate transform in `mocap_bridge` or `umi_recorder`.
- The bag writer only registers a topic the first time a message for it arrives after `open()` — if a topic never publishes during an episode (e.g. mocap dropout), it simply won't appear in that bag.
- `mocap_bridge_node`'s `rigid_body_names` and `umi_recorder_node`'s `pose_topic` are not automatically linked — if you rename the tracked rigid body or change its id, update `pose_topic` (`/mocap/<name>/pose`) to match, or `umi_recorder_node` will never receive pose data.
- `templates/index.html` loads Three.js from cdnjs (pinned to r128) — the recorder's web UI needs internet access on first load (browser-cached afterward) even though everything else runs fully offline/local.
- The `realsense2_camera` ROS node was dropped from this project after measuring it degrade from ~30fps to ~5-8fps within ~15-20 seconds on this hardware (config: 640x480@30 color+depth+align), even running alone with the CPU at ~97% idle — while a raw `pyrealsense2`/`librealsense2` pipeline with the identical config sustains a steady ~30fps indefinitely. The camera and USB link (confirmed native USB 3.0, `lsusb -t` shows `5000M`) were never the bottleneck; the ROS wrapper's message publishing path was. If frame rate regresses again, benchmark with a minimal `rs2::pipeline`/`pyrealsense2` script first to rule the SDK in or out before suspecting `recorder_node`.
- Only one process can hold the RealSense camera at a time (`rs2::backend_error ... Device or resource busy`). Before running `recorder_node` standalone for debugging, make sure no other `realsense2_camera_node` or test script still has the camera open.
- `package.xml` is parsed by `colcon-ros`, and if it fails to parse as valid XML, colcon silently falls back to treating the package as generic Python (`python`) rather than `ros.ament_python` — the build still "succeeds" with no visible error, but the package never gets registered in `AMENT_PREFIX_PATH`, so `ros2 run`/`ros2 pkg prefix`/`get_package_share_directory` all fail with "package not found". A `<!-- comment -->` containing a bare `--` (e.g. a CLI flag like `--user` in explanatory text) is invalid XML and will trigger this silently — check `colcon --log-level debug build` output for a "Failed to parse potential ROS package manifest" line if a package mysteriously can't be found after a build that reported success.
