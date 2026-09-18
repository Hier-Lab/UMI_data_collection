from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
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
        parameters=[
            {
                "config_path": get_package_share_directory("mocap_bridge")
                + "/config/natnet.conf"
            }
        ],
        output="screen",
    )

    recorder_node = Node(
        package="umi_recorder",
        executable="recorder_node",
        name="umi_recorder_node",
        output="screen",
    )

    return LaunchDescription([realsense_launch, mocap_bridge_node, recorder_node])
