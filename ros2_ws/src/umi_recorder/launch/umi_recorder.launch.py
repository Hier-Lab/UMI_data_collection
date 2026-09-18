from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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

    return LaunchDescription(
        [
            config_arg,
            mocap_bridge_node,
            recorder_node,
        ]
    )
