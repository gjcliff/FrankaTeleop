from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EqualsSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_realsense",
                default_value="true",
                description="Use the Realsense Camera. If 'false', will attempt to "
                "use usb camera or built in webcam",
            ),
            DeclareLaunchArgument(
                "camera_name",
                default_value="realsense",
                description="The name of the camera",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("realsense2_camera"),
                            "launch",
                            "rs_launch.py",
                        ]
                    )
                ),
                condition=IfCondition(
                    EqualsSubstitution(LaunchConfiguration("use_realsense"), "true")
                ),
                launch_arguments={
                    "camera_name": LaunchConfiguration("camera_name"),
                    "camera_namespace": "",
                    "enable_color": "true",
                    "enable_depth": "true",
                    "align_depth.enable": "true",
                    "pointcloud.enable": "true",
                    "pointcloud.stream_filter": "2",  # color
                    "pointcloud.stream_index_filter": "0",
                    "decimation_filter.enable": "true",
                    "spatial_filter.enable": "true",
                    "temporal_filter.enable": "true",
                    "hole_filling_filter.enable": "true",
                    "json_file_path": (
                        get_package_share_directory("handcv") + "/config/advanced.json"
                    ),
                }.items(),
            ),
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                condition=UnlessCondition(LaunchConfiguration("use_realsense")),
                arguments=["-p framerate:=30.0 -p pixel_format:=rgb8"],
            ),
            Node(
                package="handcv",
                executable="handcv",
                parameters=[{"camera_name": LaunchConfiguration("camera_name")}],
            ),
        ]
    )
