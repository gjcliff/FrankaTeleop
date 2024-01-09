from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, EqualsSubstitution
from launch.conditions import IfCondition


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz", default_value="true",
            description="use rviz or not"
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
                               "handcv"), "config", "handcv.rviz"]
                       )]
        ),
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            arguments=["-p", " ", "framerate:=30.0", " ",
                       "-p", " ", "pixel_format:=yuyv"]
        )

    ])
