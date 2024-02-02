from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import (
    DeclareLaunchArgument,
    Shutdown,
    ExecuteProcess
)
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
    Command,
    FindExecutable,
)
from launch.conditions import IfCondition, UnlessCondition

from ament_index_python import get_package_share_directory
import yaml
import os


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    load_controllers = []
    for controller in ['panda_arm_controller', 'joint_state_broadcaster']:
        load_controllers += [
            ExecuteProcess(
                cmd=['ros2 run controller_manager spawner {}'.format(
                    controller)],
                shell=True,
                output='screen',
            )
        ]
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="true",
                description="Use the real franka robot or not",
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value="dont-care",
                description="The ip of the robot. If using fake hardware, value is dont-care.\
                if using real hardware, value should be panda0.robot",
            ),
            DeclareLaunchArgument(
                "fake_sensor_commands",
                default_value="false",
                description="fake sensor commands. only valid when the fake_sensor_commands parameter is true",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="use rviz or not"
            ),
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                remappings=[('joint_states', 'franka/joint_states')],
                output={
                    'stdout': 'screen',
                    'stderr': 'screen',
                },
                on_exit=Shutdown(),
                condition=UnlessCondition(
                    LaunchConfiguration("use_fake_hardware")),
                parameters=[
                    # {  # robot description parameter
                    #     "robot_description": Command(
                    #         [
                    #             FindExecutable(name="xacro"),
                    #             " ",
                    #             PathJoinSubstitution(
                    #                 [
                    #                     FindPackageShare("franka_description"),
                    #                     "robots",
                    #                     "panda_arm.urdf.xacro",
                    #                 ]
                    #             ),
                    #             " hand:=true",
                    #             " robot_ip:=",
                    #             LaunchConfiguration("robot_ip"),
                    #             " use_fake_hardware:=",
                    #             LaunchConfiguration("use_fake_hardware"),
                    #             " fake_sensor_commands:=",
                    #             LaunchConfiguration("fake_sensor_commands"),
                    #         ]
                    #     )
                    # },
                    PathJoinSubstitution(
                        [
                            FindPackageShare("franka_moveit_config"),
                            "config",
                            "panda_ros_controllers.yaml"
                        ]
                    )
                ],
            ),
            Node(
                package='controller_manager',
                executable='ros2_control_node',

                remappings=[('joint_states', 'franka/joint_states')],
                output={
                    'stdout': 'screen',
                    'stderr': 'screen',
                },
                on_exit=Shutdown(),
                condition=IfCondition(
                    LaunchConfiguration("use_fake_hardware")),
                parameters=[
                    {  # robot description parameter
                        "robot_description": Command(
                            [
                                FindExecutable(name="xacro"),
                                " ",
                                PathJoinSubstitution(
                                    [
                                        FindPackageShare("franka_description"),
                                        "robots",
                                        "panda_arm.urdf.xacro",
                                    ]
                                ),
                                " hand:=true",
                                " robot_ip:=",
                                LaunchConfiguration("robot_ip"),
                                " use_fake_hardware:=",
                                LaunchConfiguration("use_fake_hardware"),
                                " fake_sensor_commands:=",
                                LaunchConfiguration("fake_sensor_commands"),
                            ]
                        )
                    },
                    PathJoinSubstitution(
                        [
                            FindPackageShare("franka_moveit_config"),
                            "config",
                            "panda_mock_controllers.yaml"
                        ]
                    )

                ],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                parameters=[
                    {'source_list': ['franka/joint_states',
                                     'panda_gripper/joint_states']}
                ]
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="both",
                parameters=[
                    {  # robot description parameter
                        "robot_description": Command(
                            [
                                FindExecutable(name="xacro"),
                                " ",
                                PathJoinSubstitution(
                                    [
                                        FindPackageShare("franka_description"),
                                        "robots",
                                        "panda_arm.urdf.xacro",
                                    ]
                                ),
                                " hand:=true",
                                " robot_ip:=",
                                LaunchConfiguration("robot_ip"),
                                " use_fake_hardware:=",
                                LaunchConfiguration("use_fake_hardware"),
                                " fake_sensor_commands:=",
                                LaunchConfiguration("fake_sensor_commands"),
                            ]
                        )
                    },
                ]
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='log',
                arguments=['-d', PathJoinSubstitution(
                    [
                        FindPackageShare(
                            "franka_moveit_config"), "rviz", "moveit.rviz"
                    ]
                )],
                # did I actually implement this?
                condition=IfCondition(LaunchConfiguration("use_rviz")),
                parameters=[
                    {  # robot description parameter
                        "robot_description": Command(
                            [
                                FindExecutable(name="xacro"),
                                " ",
                                PathJoinSubstitution(
                                    [
                                        FindPackageShare("franka_description"),
                                        "robots",
                                        "panda_arm.urdf.xacro",
                                    ]
                                ),
                                " hand:=true",
                                " robot_ip:=",
                                LaunchConfiguration("robot_ip"),
                                " use_fake_hardware:=",
                                LaunchConfiguration("use_fake_hardware"),
                                " fake_sensor_commands:=",
                                LaunchConfiguration("fake_sensor_commands"),
                            ]
                        )
                    },
                    {  # robot description semantic parameter
                        "robot_description_semantic": Command(
                            [
                                FindExecutable(name="xacro"),
                                " ",
                                PathJoinSubstitution(
                                    [
                                        FindPackageShare(
                                            "franka_moveit_config"),
                                        "srdf",
                                        "panda_arm.srdf.xacro",
                                    ]
                                ),
                                " hand:=true",
                            ]
                        )
                    },
                    {  # robot description kinematics parameter
                        "robot_description_kinematics": {
                            "panda_arm": {
                                "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
                                "kinematics_solver_search_resolution": 0.005,
                                "kinematics_solver_timeout": 0.05,
                            },
                            "panda_manipulator": {
                                "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
                                "kinematics_solver_search_resolution": 0.005,
                                "kinematics_solver_timeout": 0.05,
                            },
                        },
                    },
                    # {  # planning scene monitor parameters
                    #     "publish_planning_scene": True,
                    #     "publish_geometry_updates": True,
                    #     "publish_state_updates": True,
                    #     "publish_transforms_updates": True,
                    # },
                    # # Planning Functionality
                    # {
                    #     "planning_pipelines": {"pipeline_names": ["ompl"]},
                    # },
                    # {
                    #     "planning_scene_monitor_options": {
                    #         "name": "planning_scene_monitor",
                    #         "robot_description": "robot_description",
                    #         "joint_state_topic": "/joint_states",
                    #         "attached_collision_object_topic": "/moveit_cpp/planning_scene_monitor",
                    #         "publish_planning_scene_topic": "/moveit_cpp/publish_planning_scene",
                    #         "monitored_planning_scene_topic": "/moveit_cpp/monitored_planning_scene",
                    #         "wait_for_initial_state_timeout": 10.0,
                    #     }
                    # }

                ]
            ),
            # Node(
            #     package="franka_teleop",
            #     executable="franka_teleop",
            #     output="screen",
            #     parameters=[
            #         {  # robot description parameter
            #             "robot_description": Command(
            #                 [
            #                     FindExecutable(name="xacro"),
            #                     " ",
            #                     PathJoinSubstitution(
            #                         [
            #                             FindPackageShare("franka_description"),
            #                             "robots",
            #                             "panda_arm.urdf.xacro",
            #                         ]
            #                     ),
            #                     " hand:=true",
            #                     " robot_ip:=",
            #                     LaunchConfiguration("robot_ip"),
            #                     " use_fake_hardware:=",
            #                     LaunchConfiguration("use_fake_hardware"),
            #                     " fake_sensor_commands:=",
            #                     LaunchConfiguration("fake_sensor_commands"),
            #                 ]
            #             )
            #         },
            #         {  # robot description semantic parameter
            #             "robot_description_semantic": Command(
            #                 [
            #                     FindExecutable(name="xacro"),
            #                     " ",
            #                     PathJoinSubstitution(
            #                         [
            #                             FindPackageShare(
            #                                 "franka_moveit_config"),
            #                             "srdf",
            #                             "panda_arm.srdf.xacro",
            #                         ]
            #                     ),
            #                     " hand:=true",
            #                 ]
            #             )
            #         },
            #         {  # robot description kinematics parameter
            #             "robot_description_kinematics": {
            #                 "panda_arm": {
            #                     "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
            #                     "kinematics_solver_search_resolution": 0.005,
            #                     "kinematics_solver_timeout": 0.05,
            #                 },
            #                 "panda_manipulator": {
            #                     "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
            #                     "kinematics_solver_search_resolution": 0.005,
            #                     "kinematics_solver_timeout": 0.05,
            #                 },
            #             },
            #         },
            #         # Trajectory Execution Functionality
            #         {  # moveit_controllers
            #             "moveit_simple_controller_manager": load_yaml(
            #                 'franka_moveit_config', 'config/panda_controllers.yaml'),
            #             "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
            #         },
            #         {  # trajectory execution
            #             "moveit_manage_controllers": True,
            #             "trajectory_execution.allowed_execution_during_scaling": 1.2,
            #             "trajectory_execution.allowed_goal_duration_margin": 0.5,
            #             "trajectory_execution.allowed_start_tolerance": 0.01,
            #         },
            #         {  # planning scene monitor parameters
            #             "publish_planning_scene": True,
            #             "publish_geometry_updates": True,
            #             "publish_state_updates": True,
            #             "publish_transforms_updates": True,
            #         },
            #         # Planning Functionality
            #         {
            #             "planning_pipelines": {"pipeline_names": ["ompl"]},
            #         },
            #         {
            #             "planning_scene_monitor_options": {
            #                 "name": "planning_scene_monitor",
            #                 "robot_description": "robot_description",
            #                 "joint_state_topic": "/joint_states",
            #                 "attached_collision_object_topic": "/moveit_cpp/planning_scene_monitor",
            #                 "publish_planning_scene_topic": "/moveit_cpp/publish_planning_scene",
            #                 "monitored_planning_scene_topic": "/moveit_cpp/monitored_planning_scene",
            #                 "wait_for_initial_state_timeout": 10.0,
            #             }
            #         },
            #         load_yaml("franka_teleop", f"config/ompl_planning.yaml")
            #         # PathJoinSubstitution(
            #         #     [
            #         #         FindPackageShare("franka_teleop"),
            #         #         "config",
            #         #         "ompl_planning.yaml"
            #         #     ]
            #         # )
            #     ],
            # ),
        ]
        + load_controllers
    )


generate_launch_description()
