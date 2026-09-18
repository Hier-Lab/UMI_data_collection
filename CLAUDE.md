# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Goal

A data collection UI for UMI hand-grippers that records human demonstrations to train a VLA (vision-language-action) model. Each demonstration is a synchronized recording of hand trajectory (mocap) and RGB-D video (RealSense), saved as a ROS 2 bag.

## Commands

Build (run from `ros2_ws/`):

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Launch everything (RealSense camera + mocap bridge + recorder web UI):

```bash
ros2 launch umi_recorder umi_recorder.launch.py
```

Then open `http://<host>:8080` for the Record/Stop UI. Recorded episodes land in `~/umi_episodes/episode_<timestamp>/` as ROS 2 bags (override with the `output_dir` param on `recorder_node`).

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

**`mocap_bridge`** (C++, `ament_cmake`) — wraps the vendored NatNet client from `/home/hier-tony/Projects/mocap_subscriber` (copied into `third_party/MoCapParser/`, not a live dependency — resync manually if the upstream client changes) and publishes `geometry_msgs/PoseStamped` per tracked rigid body on `mocap/<name>/pose`. Polls `NatNetClient::latestPoseById()` on a timer (`publish_rate_hz` param, default 120 Hz) and skips publishing when `isPoseFresh()` is false rather than publishing stale poses. Config (server/local IP, rigid body IDs/names) lives in `config/natnet.conf`, same format as upstream.

**`umi_recorder`** (Python, `ament_python`) — the recorder + web UI, all in `recorder_node.py`:
- Subscribes to RealSense color/depth (`sensor_msgs/Image`, default topics from `realsense2_camera` with `align_depth` enabled) and to the mocap bridge's `PoseStamped` topic.
- Runs a Flask app in a background thread (started from `Node.__init__`) alongside `rclpy.spin()` on the main thread — the two share `RecorderNode` state (latest JPEG frame, trajectory buffer, recording flag) behind a lock.
- `/video_feed` serves an MJPEG stream re-encoded from the latest color frame; `/trajectory` returns the last 500 pose points as JSON for the canvas plot in `templates/index.html`.
- Record/Stop (`/record`, `/stop` POST endpoints) toggle `EpisodeBagWriter` (`bag_writer.py`), a thin `rosbag2_py.SequentialWriter` wrapper — one bag per episode, topics registered lazily on first message once recording starts. All three subscribed topics (color, depth, pose) are written into the same bag so an episode is fully self-contained and time-synchronized via ROS message timestamps.

`launch/umi_recorder.launch.py` brings up `realsense2_camera`'s `rs_launch.py` (640x480@30, depth aligned to color), `mocap_bridge_node`, and `recorder_node` together.

## Notes

- Quaternion order from the mocap client is `(w, x, y, z)` (Eigen convention) — `mocap_bridge_node.cpp`'s `toPoseStamped` maps this correctly into `geometry_msgs/Quaternion` (which is `x,y,z,w`); double-check this mapping first if orientation looks wrong downstream.
- If Motive is set to `Z Up`, do not apply an additional coordinate transform in `mocap_bridge` or `umi_recorder`.
- The bag writer only registers a topic the first time a message for it arrives after `open()` — if a topic never publishes during an episode (e.g. mocap dropout), it simply won't appear in that bag.
