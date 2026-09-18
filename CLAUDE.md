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

Then open `http://<host>:8080` for the Record/Stop UI. Recorded episodes land in `~/umi_episodes/episode_<timestamp>/` as ROS 2 bags (override with the `output_dir` param on `recorder_node`).

To point at a different config (e.g. testing against a different Motive server or a different camera topic set), override the launch args instead of editing installed files:

```bash
ros2 launch umi_recorder umi_recorder.launch.py \
  natnet_config:=/path/to/my_natnet.conf \
  mocap_bridge_params:=/path/to/my_mocap_bridge.yaml \
  umi_recorder_params:=/path/to/my_umi_recorder.yaml
```

Run nodes individually for debugging:

```bash
ros2 run mocap_bridge mocap_bridge_node --ros-args -p config_path:=$(ros2 pkg prefix mocap_bridge)/share/mocap_bridge/config/natnet.conf
ros2 run umi_recorder recorder_node
```

Inspect a recorded bag:

```bash
ros2 bag info ~/umi_episodes/episode_20260101_120000
```

## Architecture

Workspace: `ros2_ws/src/`, two packages.

**`mocap_bridge`** (C++, `ament_cmake`) — wraps the NatNet client from [mocap_subscriber](https://github.com/lgkimjy/mocap_subscriber), pulled in as a git submodule at `third_party/mocap_subscriber/` (built via `add_subdirectory`, so it tracks a pinned commit — bump the submodule and commit the pointer to pick up upstream changes) and publishes `geometry_msgs/PoseStamped` per tracked rigid body on `mocap/<name>/pose`. Polls `NatNetClient::latestPoseById()` on a timer and skips publishing when `isPoseFresh()` is false rather than publishing stale poses.
  - `config/natnet.conf` — mocap connection settings in `mocap_subscriber`'s native key=value format: `server_address`/`local_address` (Motive PC vs. this Linux PC IP), `use_multicast`, `command_port`/`data_port`, and `rigid_body_id`/`rigid_body_name` (comma-separated, matched by position — add more objects by extending both lists). Loaded via the `config_path` ROS param.
  - `config/mocap_bridge.yaml` — ROS params for the node itself: `frame_id`, `publish_rate_hz` (default 120), `pose_max_age_ms` (default 250).

**`umi_recorder`** (Python, `ament_python`) — the recorder + web UI, all in `recorder_node.py`:
- Subscribes to RealSense color/depth (`sensor_msgs/Image`, default topics from `realsense2_camera` with `align_depth` enabled) and to the mocap bridge's `PoseStamped` topic.
- Runs a Flask app in a background thread (started from `Node.__init__`) alongside `rclpy.spin()` on the main thread — the two share `RecorderNode` state (latest JPEG frame, trajectory buffer, recording flag) behind a lock.
- `/video_feed` serves an MJPEG stream re-encoded from the latest color frame; `/trajectory` returns the last 500 pose points as JSON for the canvas plot in `templates/index.html`.
- Record/Stop (`/record`, `/stop` POST endpoints) toggle `EpisodeBagWriter` (`bag_writer.py`), a thin `rosbag2_py.SequentialWriter` wrapper — one bag per episode, topics registered lazily on first message once recording starts. All three subscribed topics (color, depth, pose) are written into the same bag so an episode is fully self-contained and time-synchronized via ROS message timestamps.
  - `config/umi_recorder.yaml` — ROS params: `color_topic`, `depth_topic`, `pose_topic` (must match the name mocap_bridge publishes, i.e. `/mocap/<rigid_body_name>/pose`), `output_dir`, `web_host`, `web_port`.

`launch/umi_recorder.launch.py` brings up `realsense2_camera`'s `rs_launch.py` (640x480@30, depth aligned to color), `mocap_bridge_node`, and `recorder_node` together, and exposes the three config files above as launch arguments (`natnet_config`, `mocap_bridge_params`, `umi_recorder_params`) so a deployment-specific config doesn't require editing installed package files.

## Notes

- Quaternion order from the mocap client is `(w, x, y, z)` (Eigen convention) — `mocap_bridge_node.cpp`'s `toPoseStamped` maps this correctly into `geometry_msgs/Quaternion` (which is `x,y,z,w`); double-check this mapping first if orientation looks wrong downstream.
- If Motive is set to `Z Up`, do not apply an additional coordinate transform in `mocap_bridge` or `umi_recorder`.
- The bag writer only registers a topic the first time a message for it arrives after `open()` — if a topic never publishes during an episode (e.g. mocap dropout), it simply won't appear in that bag.
- `natnet.conf`'s `rigid_body_name` and `umi_recorder.yaml`'s `pose_topic` are not automatically linked — if you rename the tracked rigid body or change its id, update `pose_topic` (`/mocap/<name>/pose`) to match, or `umi_recorder_node` will never receive pose data.
