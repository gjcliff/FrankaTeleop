import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    OpaqueFunction,
    Shutdown,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.parameter_descriptions import ParameterValue
from launch_param_builder import ParameterBuilder
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable,
    Command,
    AndSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

import yaml


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
    robot_ip_parameter_name = "robot_ip"
    use_fake_hardware_parameter_name = "use_fake_hardware"
    fake_sensor_commands_parameter_name = "fake_sensor_commands"
    load_gripper_parameter_name = "load_gripper"
    ee_id_parameter_name = "ee_id"
    arm_id_parameter_name = "arm_id"

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(
        fake_sensor_commands_parameter_name
    )
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    ee_id = LaunchConfiguration(ee_id_parameter_name)

    # planning_context
    franka_xacro_file = os.path.join(
        get_package_share_directory("franka_description"),
        "robots",
        "fr3",
        "fr3.urdf.xacro",
    )

    robot_description_config = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            franka_xacro_file,
            " hand:=",
            load_gripper,
            " robot_ip:=",
            robot_ip,
            " ee_id:=",
            ee_id,
            " use_fake_hardware:=",
            use_fake_hardware,
            " fake_sensor_commands:=",
            fake_sensor_commands,
            " ros2_control:=true",
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(
            robot_description_config, value_type=str
        )
    }

    franka_semantic_xacro_file = os.path.join(
        get_package_share_directory("franka_description"),
        "robots",
        "fr3",
        "fr3.srdf.xacro",
    )

    robot_description_semantic_config = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            franka_semantic_xacro_file,
            " hand:=",
            load_gripper,
            " ee_id:=",
            ee_id,
        ]
    )

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_config, value_type=str
        )
    }

    kinematics_yaml = load_yaml(
        "franka_fr3_moveit_config", "config/kinematics.yaml"
    )

    kinematics_config = {"robot_description_kinematics": kinematics_yaml}

    joint_limits_yaml = load_yaml(
        "franka_fr3_moveit_config", "config/fr3_joint_limits.yaml"
    )

    joint_limits_config = {"robot_description_planning": joint_limits_yaml}

    # get parameters for the servo node
    servo_params = {
        "moveit_servo": (
            ParameterBuilder("moveit_servo")
            .yaml("config/panda_simulated_config.yaml")
            .to_dict()
        )
    }

    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "fr3_arm"}
    servo_params["moveit_servo"]["move_group_name"] = "fr3_arm"
    servo_params["moveit_servo"][
        "command_out_topic"
    ] = "/fr3_arm_controller/joint_trajectory"

    # this filter parameter should be >1. increase it for greater smoothing but
    # slower motion
    # low_pass_filter_coeff = {"butterworth_filter_coeff": 3.0}

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="use_rviz",
                default_value="true",
                description="whether or not to use rviz.",
            ),
            DeclareLaunchArgument(
                name="arm_id",
                default_value="fr3",
                description="ID of the type of arm used",
            ),
            DeclareLaunchArgument(
                name="robot_ip",
                default_value="dont-care",
                description="IP address of the robot",
            ),
            DeclareLaunchArgument(
                name="use_fake_hardware",
                default_value="true",
                description="whether or not to use fake hardware.",
            ),
            DeclareLaunchArgument(
                name="ee_id",
                default_value="franka_hand",
                description="The end-effector id to use. Available options: "
                "none, franka_hand, cobot_pump",
            ),
            DeclareLaunchArgument(
                name="load_gripper",
                default_value="true",
                description="Whether to load the gripper or not (true or "
                "false)",
            ),
            DeclareLaunchArgument(
                name="load_gripper",
                default_value="true",
                description="Whether to load the gripper or not (true or "
                "false)",
            ),
            DeclareLaunchArgument(
                name="fake_sensor_commands",
                default_value="false",
                description="Fake sensor commands. Only valid when "
                "'fake_sensor_commands' is true",
            ),
            Node(
                package="franka_teleop",
                executable="franka_servo",
                parameters=[
                    servo_params,
                    acceleration_filter_update_period,
                    planning_group_name,
                    # low_pass_filter_coeff,
                    robot_description,
                    robot_description_semantic,
                    kinematics_config,
                    joint_limits_config,
                ],
                output="screen",
            ),
        ]
    )
