import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, Shutdown, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable, Command
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("numsr_franka")
        .robot_description(file_path="config/panda.urdf.xacro")
        .trajectory_execution(file_path="config/panda_controllers.yaml")
        .planning_pipelines("ompl", ["ompl"])
        .moveit_cpp(
            file_path=get_package_share_directory("numsr_franka_moveit_config")
            + "/config/moveit_cpp.yaml"
        )
        .to_moveit_configs()
    )

    # Load controllers
    # panda_controller = PythonExpression(["'\"panda_mock_controllers\" if ", LaunchConfiguration("panda_controllers"), " else \"panda_ros_controllers\"'"])
    load_controllers = []
    for controller in [
        "panda_arm_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(
                    controller)],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(name="use_fake_hardware", default_value="true",
                                  description="whether or not to use fake hardware."),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                condition=IfCondition(
                    LaunchConfiguration("use_fake_hardware")),
                parameters=[moveit_config.robot_description, PathJoinSubstitution([
                    FindPackageShare(
                        "numsr_franka_moveit_config"), "config", "panda_mock_controllers.yaml"
                ])],
                output="both",
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                condition=UnlessCondition(
                    LaunchConfiguration("use_fake_hardware")),
                parameters=[moveit_config.robot_description, PathJoinSubstitution([
                    FindPackageShare(
                        "numsr_franka_moveit_config"), "config", "panda_ros_controllers.yaml"
                ])],
                output="both",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_publisher",
                on_exit=Shutdown(),
                output="log",
                arguments=["--frame-id", "world",
                           "--child-frame-id", "panda_link0"],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="both",
                parameters=[moveit_config.robot_description],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                on_exit=Shutdown(),
                output="log",
                arguments=["-d", PathJoinSubstitution([
                    FindPackageShare("franka_teleop"), "config", "moveit.rviz"
                ])],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                ],
            ),
            Node(
                package="franka_teleop",
                executable="franka_teleop",
                on_exit=Shutdown(),
                output="screen",
                parameters=[moveit_config.to_dict()],
            ),
        ]
        + load_controllers
    )
