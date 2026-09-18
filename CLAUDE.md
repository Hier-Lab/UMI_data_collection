# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Goal

A data collection UI for UMI hand-grippers that records human demonstrations to train a VLA (vision-language-action) model. Each demonstration is a synchronized recording of hand trajectory (mocap) and RGB-D video (RealSense), saved as a ROS 2 bag.

## Commands

First-time setup — `mocap_bridge` pulls in `mocap_subscriber` as a git submodule:

```bash
git submodule update --init --recursive
```

Build (run from `ros2_ws/`):

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Launch everything (RealSense camera + mocap bridge + recorder web UI):

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
- Subscribes to RealSense color/depth (`sensor_msgs/Image`, default topics from `realsense2_camera` with `align_depth` enabled) and to the mocap bridge's `PoseStamped` topic (`pose_topic` param — must match `/mocap/<rigid_body_names[i]>/pose` from `mocap_bridge_node`'s config). All three subscriptions use `qos_profile_sensor_data` (best-effort) to match `mocap_bridge_node`'s `SensorDataQoS` publisher and `realsense2_camera`'s publishers — a reliable subscriber (rclpy's default from a plain integer queue depth) never connects to a best-effort publisher in DDS, so this must stay best-effort or the corresponding topic silently stops delivering messages with no error.
- Runs a Flask app in a background thread (started from `Node.__init__`) alongside `rclpy.spin()` on the main thread — the two share `RecorderNode` state (latest color/depth JPEG, trajectory buffer, recording flag) behind a lock.
- `/video_feed` serves an MJPEG stream re-encoded from the latest color frame. `/depth_feed` serves the depth stream colorized with `cv2.COLORMAP_JET`, clipped/normalized against `max_depth_m` (near = bright, beyond `max_depth_m` or no-return = dark/black) — see `_on_depth`.
- `/trajectory` returns the last `MAX_TRAJECTORY_POINTS` (20) pose points as JSON `{x,y,z}`, rendered as a 3D trail in `templates/index.html` via Three.js (orbit camera, grid, axes — not a flat 2D projection), plus a marker at the current pose.
- `/config` exposes `max_depth_m` so the frontend can label the depth panel's range without hardcoding it.
- Record/Stop (`/record`, `/stop` POST endpoints) toggle `EpisodeBagWriter` (`bag_writer.py`), a thin `rosbag2_py.SequentialWriter` wrapper — one bag per episode, topics registered lazily on first message once recording starts. All three subscribed topics (color, depth, pose) are written into the same bag (raw depth, not the colorized visualization) so an episode is fully self-contained and time-synchronized via ROS message timestamps.

Both nodes' parameters live together in one file, `umi_recorder/config/umi_data_collection.yaml`, as separate `mocap_bridge_node:`/`umi_recorder_node:` top-level sections (ROS 2's standard multi-node YAML params format — each node only reads its own section, so passing the whole file to both is intentional, not a bug). `launch/umi_recorder.launch.py` brings up `realsense2_camera`'s `rs_launch.py` (640x480@30, depth aligned to color) plus both nodes, and exposes the config file as a single launch argument (`umi_config` — named to avoid colliding with `rs_launch.py`'s own `config` argument for a RealSense device JSON) so a deployment-specific setup doesn't require editing the installed package file.

## Notes

- Quaternion order from the mocap client is `(w, x, y, z)` (Eigen convention) — `mocap_bridge_node.cpp`'s `toPoseStamped` maps this correctly into `geometry_msgs/Quaternion` (which is `x,y,z,w`); double-check this mapping first if orientation looks wrong downstream.
- If Motive is set to `Z Up`, do not apply an additional coordinate transform in `mocap_bridge` or `umi_recorder`.
- The bag writer only registers a topic the first time a message for it arrives after `open()` — if a topic never publishes during an episode (e.g. mocap dropout), it simply won't appear in that bag.
- `mocap_bridge_node`'s `rigid_body_names` and `umi_recorder_node`'s `pose_topic` are not automatically linked — if you rename the tracked rigid body or change its id, update `pose_topic` (`/mocap/<name>/pose`) to match, or `umi_recorder_node` will never receive pose data.
- `templates/index.html` loads Three.js from cdnjs (pinned to r128) — the recorder's web UI needs internet access on first load (browser-cached afterward) even though everything else runs fully offline/local.
