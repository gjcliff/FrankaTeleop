from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import (
    DeclareLaunchArgument,
    Shutdown,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
    EqualsSubstitution,
    Command,
    FindExecutable,
    PythonExpression,
)
from launch.conditions import IfCondition
from moveit_configs_utils import MoveItConfigsBuilder

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
            Node(
                package="franka_teleop",
                executable="franka_teleop",
                output="screen",
                parameters=[
                    {
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
                    {
                        "robot_description_semantic": Command(
                            [
                                FindExecutable(name="xacro"),
                                " ",
                                PathJoinSubstitution(
                                    [
                                        FindPackageShare("franka_moveit_config"),
                                        "srdf",
                                        "panda_arm.srdf.xacro",
                                    ]
                                ),
                                " hand:=true",
                            ]
                        )
                    },
                    {
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
                    }
                    # load_yaml('franka_moveit_config', 'config/kinematics.yaml'),
                ],
            ),
        ]
    )
