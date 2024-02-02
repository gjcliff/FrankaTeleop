import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, Shutdown, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable, Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("msr_franka")
        .robot_description(file_path="config/panda.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines("ompl", ["ompl"])
        .moveit_cpp(
            file_path=get_package_share_directory("franka_teleop")
            + "/config/moveit_cpp.yaml"
        )
        .to_moveit_configs()
    )
    # MoveItCpp demo executable
    moveit_cpp_node = Node(
        package="franka_teleop",
        executable="franka_teleop",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("franka_teleop")
        + "/launch/moveit_cpp_tutorial.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        on_exit=Shutdown(),
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "panda_link0"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    DeclareLaunchArgument(name="use_fake_hardware", default="true",
                          description="whether or not to use fake hardware.")



    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("msr_franka_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        condition=IfCondition(LaunchConfiguration("use_fake_hardware")),
        parameters=[moveit_config.robot_description, PathJoinSubstitution()],
        output="both",
    )

    # DeclareLaunchArgument(
    #     "panda_controllers", default_value="panda_mock_controllers",
    #     description="which panda_controllers file to use, panda_ros_controllers or\
    #         panda_mock_controllers."
    # )

    # Load controllers
    # panda_controller = PythonExpression(["'\"panda_mock_controllers\" if ", LaunchConfiguration("panda_controllers"), " else \"panda_ros_controllers\"'"])
    load_controllers = []
    for controller in [
        "panda_arm_controller",
        "panda_hand_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription(
        [
            static_tf,
            robot_state_publisher,
            rviz_node,
            moveit_cpp_node,
            ros2_control_node,
        ]
        + load_controllers
    )