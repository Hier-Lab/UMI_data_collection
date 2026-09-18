from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    mocap_bridge_share = get_package_share_directory("mocap_bridge")
    umi_recorder_share = get_package_share_directory("umi_recorder")

    natnet_config_arg = DeclareLaunchArgument(
        "natnet_config",
        default_value=mocap_bridge_share + "/config/natnet.conf",
        description="Path to the mocap (NatNet) connection config: server/local IP, rigid body ids/names",
    )
    mocap_bridge_params_arg = DeclareLaunchArgument(
        "mocap_bridge_params",
        default_value=mocap_bridge_share + "/config/mocap_bridge.yaml",
        description="Path to mocap_bridge_node ROS parameters (frame_id, publish rate, pose max age)",
    )
    umi_recorder_params_arg = DeclareLaunchArgument(
        "umi_recorder_params",
        default_value=umi_recorder_share + "/config/umi_recorder.yaml",
        description="Path to umi_recorder_node ROS parameters (topics, output dir, web host/port)",
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
        parameters=[
            LaunchConfiguration("mocap_bridge_params"),
            {"config_path": LaunchConfiguration("natnet_config")},
        ],
        output="screen",
    )

    recorder_node = Node(
        package="umi_recorder",
        executable="recorder_node",
        name="umi_recorder_node",
        parameters=[LaunchConfiguration("umi_recorder_params")],
        output="screen",
    )

    return LaunchDescription(
        [
            natnet_config_arg,
            mocap_bridge_params_arg,
            umi_recorder_params_arg,
            realsense_launch,
            mocap_bridge_node,
            recorder_node,
        ]
    )
