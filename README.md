# UMI Data Collection

Records synchronized human demonstrations (hand trajectory + RGB-D video) for training a VLA (vision-language-action) model with UMI hand-grippers. Each demonstration is saved as a ROS 2 bag containing mocap poses and one or more RealSense camera streams.

## Packages

- **`mocap_bridge`** (C++) — publishes `geometry_msgs/PoseStamped` per tracked rigid body from an OptiTrack/Motive NatNet stream.
- **`umi_recorder`** (Python) — captures up to 4 RealSense cameras directly via `pyrealsense2`, subscribes to mocap poses, and serves a web UI (Record/Stop, live video + depth + 3D trajectory) that writes everything into one ROS 2 bag per episode. Optionally also owns one exocentric camera (e.g. a D435i facing the human operator).
- **`exo_pose_bridge`** (Python) — runs MediaPipe Pose on the exocentric camera's live feed to track human torso motion (for mapping to linear base motion), publishing a normalized torso position and a skeleton-overlay feed shown in the web UI.
- **`rgbd_odometry`** (Python) — post-process CLI that computes RGBD visual odometry (Open3D) for one camera of a recorded episode and writes it back into that bag as a new topic.

## Setup

```bash
git submodule update --init --recursive   # mocap_bridge's NatNet client
pip3 install pyrealsense2

# rgbd_odometry needs open3d + a modern numpy, which conflict with this
# ROS 2 Humble environment's pinned numpy - see docs/rgbd_odometry.md
python3 -m pip install --user virtualenv
virtualenv ros2_ws/src/rgbd_odometry/.venv
ros2_ws/src/rgbd_odometry/.venv/bin/pip install open3d numpy

# exo_pose_bridge (optional - only needed if you use an exocentric camera)
# needs mediapipe, which has the same numpy conflict - see docs/architecture.md
virtualenv --system-site-packages ros2_ws/src/exo_pose_bridge/.venv
ros2_ws/src/exo_pose_bridge/.venv/bin/pip install mediapipe
ros2_ws/src/exo_pose_bridge/.venv/bin/pip install --ignore-installed matplotlib
mkdir -p ros2_ws/src/exo_pose_bridge/models
curl -fsSL -o ros2_ws/src/exo_pose_bridge/models/pose_landmarker_lite.task \
  "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_lite/float16/latest/pose_landmarker_lite.task"
```

## Build

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch umi_recorder umi_recorder.launch.py
```

Open `http://<host>:8080` for the Record/Stop UI. Episodes land in `output_dir` (see config) as ROS 2 bags.

Post-process one camera's episode into visual odometry:

```bash
ros2 run rgbd_odometry compute_odometry --bag ~/umi_episodes/episode_<timestamp> --camera <name> --depth-trunc 0.5
```

## Configuration

All settings (mocap connection, tracked rigid bodies, per-camera resolution/fps/serial/max-depth, output dir, web host/port) live in one file: [`ros2_ws/src/umi_recorder/config/umi_data_collection.yaml`](ros2_ws/src/umi_recorder/config/umi_data_collection.yaml), with inline comments.

## Documentation

See [`docs/`](docs/) for architecture details, hardware notes, and debugging gotchas:

- [docs/architecture.md](docs/architecture.md) — how each package works
- [docs/rgbd_odometry.md](docs/rgbd_odometry.md) — the post-process odometry pipeline
- [docs/gotchas.md](docs/gotchas.md) — hardware quirks and debugging lessons learned
