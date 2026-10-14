import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        "umi_config",
        default_value=get_package_share_directory("umi_recorder")
        + "/config/umi_data_collection.yaml",
        description="Path to the umi_data_collection.yaml config (mocap connection, "
        "tracked rigid bodies, topics, recording output, web UI host/port)",
    )

    # RealSense color/depth are captured directly via pyrealsense2 inside
    # umi_recorder_node (see recorder_node.py) rather than through the
    # realsense2_camera ROS node, which was measured to degrade from ~30fps
    # to ~5-8fps within seconds under this workload.

    mocap_bridge_node = Node(
        package="mocap_bridge",
        executable="mocap_bridge_node",
        name="mocap_bridge_node",
        parameters=[LaunchConfiguration("umi_config")],
        output="screen",
    )

    recorder_node = Node(
        package="umi_recorder",
        executable="recorder_node",
        name="umi_recorder_node",
        parameters=[LaunchConfiguration("umi_config")],
        output="screen",
    )

    # exo_pose_bridge is not run via `ros2 run` / launch_ros's Node action:
    # it needs mediapipe's own numpy/opencv, which conflict with this ROS 2
    # Humble environment's pinned numpy (same problem rgbd_odometry has with
    # open3d - see that package's docs). It lives in a dedicated venv
    # (ros2_ws/src/exo_pose_bridge/.venv/, built with --system-site-packages
    # so it can still see rclpy/sensor_msgs/geometry_msgs) that this launch
    # file invokes directly. If exo_camera_name is empty in umi_config, the
    # node starts but does nothing (see pose_node.py).
    #
    # This launch file lives at ros2_ws/src/umi_recorder/launch/ - a
    # source-tree path, not an installed share/ path - specifically so this
    # relative lookup of exo_pose_bridge's sibling source package works.
    exo_pose_bridge_src = (Path(__file__).resolve().parents[2] / "exo_pose_bridge").resolve()
    exo_pose_bridge_venv_python = exo_pose_bridge_src / ".venv" / "bin" / "python3"

    # additional_env's PYTHONPATH would otherwise replace (not extend) the
    # PYTHONPATH that sourcing ROS's setup.bash put in this process's own
    # environment (which rclpy needs) - prepend to it explicitly instead of
    # relying on additional_env's merge behavior.
    exo_pose_bridge_pythonpath = os.pathsep.join(
        filter(None, [str(exo_pose_bridge_src), os.environ.get("PYTHONPATH", "")])
    )

    exo_pose_bridge_node = ExecuteProcess(
        cmd=[
            str(exo_pose_bridge_venv_python),
            "-m",
            "exo_pose_bridge.pose_node",
            "--ros-args",
            "--params-file",
            LaunchConfiguration("umi_config"),
        ],
        additional_env={"PYTHONPATH": exo_pose_bridge_pythonpath},
        output="screen",
    )

    return LaunchDescription(
        [
            config_arg,
            mocap_bridge_node,
            recorder_node,
            exo_pose_bridge_node,
        ]
    )
