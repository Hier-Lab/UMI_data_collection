from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("realsense2_camera"), "/launch/rs_launch.py"]
        ),
        launch_arguments={
            "rgb_camera.color_profile": "640x480x30",
            "depth_module.depth_profile": "640x480x30",
            "align_depth.enable": "true",
        }.items(),
    )

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
            realsense_launch,
            mocap_bridge_node,
            recorder_node,
        ]
    )
