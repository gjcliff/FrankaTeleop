import os 
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, Shutdown, DeclareLaunchArgument, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable, Command, AndSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(name="use_fake_hardware", default_value="true",
                                  description="whether or not to use fake hardware."),
            DeclareLaunchArgument(name="use_rviz", default_value="true",
                                  description="whether or not to use rviz."),
            DeclareLaunchArgument(name="robot_ip", default_value="dont-care",
                                  description="IP address of the robot"),
            DeclareLaunchArgument(name="use_realsense", default_value="true",
                                  description="whether or not to use realsense camera."),
            DeclareLaunchArgument(name="run_franka_teleop", default_value="true",
                                  description="whether or not to run franka teleop."),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution(
                    [FindPackageShare('franka_teleop'), 'launch', 'franka_msr.launch.py'])]),
                launch_arguments={'robot_ip': LaunchConfiguration("robot_ip"),
                                  'use_fake_hardware': LaunchConfiguration("use_fake_hardware"),
                                  'use_rviz': 'false'}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution(
                    [FindPackageShare('franka_teleop'), 'launch', 'franka_rviz.launch.py'])]),
                launch_arguments={'robot_ip': LaunchConfiguration("robot_ip"),
                                  'use_fake_hardware': LaunchConfiguration("use_fake_hardware"),
                                  'use_rviz': 'true'}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution(
                    [FindPackageShare('handcv'), 'launch', 'camera.launch.py'])]),
                launch_arguments={'use_realsense': LaunchConfiguration("use_realsense")}.items(),
            ),
            Node(
                package="cv_franka_bridge",
                executable="cv_franka_bridge",
                output="screen"
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'camera_link']
            ),
            # SetLaunchConfiguration(
            #     "robot_ip", PythonExpression(["'\"dont-care\" if ", LaunchConfiguration("use_fake_hardware"), " == \"true\" else \"panda0.robot\"'"])),
        ]
    )