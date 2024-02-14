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
    moveit_config_fake = (
        MoveItConfigsBuilder("numsr_franka")
        .robot_description(file_path="config/panda_arm_fake.urdf.xacro")
        .robot_description_semantic(file_path="config/panda_arm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/panda_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines("ompl", ["ompl"])
        .moveit_cpp(
            file_path=get_package_share_directory("numsr_franka_moveit_config")
            + "/config/moveit_cpp.yaml"
        )
        .to_moveit_configs()
    )
    moveit_config_real = (
        MoveItConfigsBuilder("numsr_franka")
        .robot_description(file_path="config/panda_arm_real.urdf.xacro")
        .robot_description_semantic(file_path="config/panda_arm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/panda_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines("ompl", ["ompl"])
        .moveit_cpp(
            file_path=get_package_share_directory("numsr_franka_moveit_config")
            + "/config/moveit_cpp.yaml"
        )
        .to_moveit_configs()
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(name="use_fake_hardware", default_value="false",
                                  description="whether or not to use fake hardware."),
            DeclareLaunchArgument(name="use_rviz", default_value="true",
                                  description="whether or not to use rviz."),
            DeclareLaunchArgument(name="robot_ip", default_value="panda0.robot",
                                  description="IP address of the robot"),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                condition=IfCondition(AndSubstitution(LaunchConfiguration("use_fake_hardware"), LaunchConfiguration("use_rviz"))),
                on_exit=Shutdown(),
                output="log",
                arguments=["-d", PathJoinSubstitution([
                    FindPackageShare("franka_teleop"), "config", "moveit.rviz"
                ])],
                parameters=[
                    moveit_config_fake.robot_description,
                    moveit_config_fake.robot_description_semantic,
                    moveit_config_fake.robot_description_kinematics,
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                condition=UnlessCondition(AndSubstitution(LaunchConfiguration("use_fake_hardware"), LaunchConfiguration("use_rviz"))),
                on_exit=Shutdown(),
                output="log",
                arguments=["-d", PathJoinSubstitution([
                    FindPackageShare("franka_teleop"), "config", "moveit.rviz"
                ])],
                parameters=[
                    moveit_config_real.robot_description,
                    moveit_config_real.robot_description_semantic,
                    moveit_config_real.robot_description_kinematics,
                ],
            ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource([PathJoinSubstitution(
            #         [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
            #     launch_arguments={'robot_ip': LaunchConfiguration("robot_ip"),
            #                       'use_fake_hardware': LaunchConfiguration("use_fake_hardware")}.items(),
            # ),
        ]
    )
