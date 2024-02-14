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

    # Load controllers
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
            DeclareLaunchArgument(name="use_rviz", default_value="true",
                                  description="whether or not to use rviz."),
            DeclareLaunchArgument(name="robot_ip", default_value="dont-care",
                                  description="IP address of the robot"),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                condition=IfCondition(
                    LaunchConfiguration("use_fake_hardware")),
                remappings=[('joint_states', 'franka/joint_states')],
                parameters=[moveit_config_fake.robot_description, PathJoinSubstitution([
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
                remappings=[('joint_states', 'franka/joint_states')],
                parameters=[moveit_config_real.robot_description, PathJoinSubstitution([
                    FindPackageShare(
                        "numsr_franka_moveit_config"), "config", "panda_ros_controllers.yaml"
                ])],
                output="both",
            ),
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                parameters=[
                    {'source_list': ['franka/joint_states', 'panda_gripper/joint_states'], 'rate': 30}],
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
                condition=IfCondition(LaunchConfiguration("use_fake_hardware")),
                output="both",
                parameters=[moveit_config_fake.robot_description],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                condition=UnlessCondition(LaunchConfiguration("use_fake_hardware")),
                output="both",
                parameters=[moveit_config_real.robot_description],
            ),
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution(
                    [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
                launch_arguments={'robot_ip': LaunchConfiguration("robot_ip"),
                                  'use_fake_hardware': LaunchConfiguration("use_fake_hardware")}.items(),
            ),
            Node(
                package="franka_teleop",
                executable="franka_teleop_node",
                condition=IfCondition(LaunchConfiguration("use_fake_hardware")),
                on_exit=Shutdown(),
                output="screen",
                parameters=[moveit_config_fake.to_dict()],
            ),
            Node(
                package="franka_teleop",
                executable="franka_teleop_node",
                condition=UnlessCondition(LaunchConfiguration("use_fake_hardware")),
                on_exit=Shutdown(),
                output="screen",
                parameters=[moveit_config_real.to_dict()],
            ),
        ]
        + load_controllers
    )
