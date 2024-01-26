from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import (DeclareLaunchArgument, Shutdown, IncludeLaunchDescription,
                            SetLaunchConfiguration)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (PathJoinSubstitution, LaunchConfiguration, EqualsSubstitution,
                                  Command, FindExecutable, PythonExpression)
from launch.conditions import IfCondition
from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python import get_package_share_directory
import yaml
import os

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_fake_hardware", default_value="true",
            description="Use the real franka robot or not"
        ),
        DeclareLaunchArgument(
            "fake_sensor_commands", default_value="false",
            description="fake sensor commands. only valid when the fake_sensor_commands parameter is true"
        ),
        SetLaunchConfiguration(
            "robot_ip",
            PythonExpression(["'\"dont-care\" if ",
                              LaunchConfiguration("use_fake_hardware"),
                              " == \"true\" else \"panda0.robot\"'"])),
        Node(
            package="franka_teleop",
            executable="franka_teleop",
            parameters=[
                {'robot_description':
                 Command([FindExecutable(name='xacro'), ' ',
                          PathJoinSubstitution([FindPackageShare('franka_description'),
                                               'robots', 'panda_arm.urdf.xacro']),
                                               # error here!! launchconfiguration(robotip)
                          ' hand:=true', ' robot_ip:=', LaunchConfiguration("robot_ip"), ' use_fake_hardware:=',
                          LaunchConfiguration("use_fake_hardware"), ' fake_sensor_commands:=', LaunchConfiguration("fake_sensor_commands")])
                },
                {'franka_moveit_config': load_yaml('franka_moveit_config', 'config/kinematics.yaml')},
                {'robot_description_semantic': 
                 PathJoinSubstitution([FindPackageShare('franka_moveit_config'),
                                       'srdf',
                                       'panda_arm.srdf.xacro'])},
            ]
                        
        ),
    ])