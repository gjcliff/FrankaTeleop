from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import OpaqueFunction, DeclareLaunchArgument, Shutdown, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, EqualsSubstitution
from launch.conditions import IfCondition

from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz", default_value="true",
            description="to use rviz, or not to use rviz, that is the question."
        ),
        DeclareLaunchArgument(
            "use_realsense", default_value="true",
            description="Use the Realsense Camera. If 'false', will attempt to use usb camera or built in webcam"
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch',
                    'rs_launch.py'
                ])
            ),
            condition=IfCondition(EqualsSubstitution(
                LaunchConfiguration("use_realsense"), "true")),
            launch_arguments={
                "align_depth.enable": "true",
                "pointcloud.enable": "true",
                "json_file_path": get_package_share_directory("handcv") + "/config/high_density_preset.json",
                "pointcloud.enable": "true",
                }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            on_exit=Shutdown(),  # when rviz closes, shutdown all nodes
            condition=IfCondition(EqualsSubstitution(
                LaunchConfiguration("use_rviz"), "true")),
            arguments=["-d",
                       PathJoinSubstitution(
                           [FindPackageShare(
                               "handcv"), "config", "handcv.rviz"])],
        ),
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            condition=IfCondition(EqualsSubstitution(
                LaunchConfiguration("use_realsense"), "false")),
            arguments=["-p framerate:=30.0 -p pixel_format:=yuyv"]
        ),
        Node(
            package="handcv",
            executable="handcv",
        ),
        # Node(
        #     package="rqt_image_view",
        #     executable="rqt_image_view",
        #     arguments=["/camera/color/image_raw"],
        # )

    ])
