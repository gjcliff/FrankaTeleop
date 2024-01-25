from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import (DeclareLaunchArgument, Shutdown, IncludeLaunchDescription,
                            SetLaunchConfiguration)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (PathJoinSubstitution, LaunchConfiguration, EqualsSubstitution,
                                  Command, FindExecutable, PythonExpression)
from launch.conditions import IfCondition

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


ompl_planning_pipeline_config = {
    'move_group': {
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                            'default_planner_request_adapters/ResolveConstraintFrames '
                            'default_planner_request_adapters/FixWorkspaceBounds '
                            'default_planner_request_adapters/FixStartStateBounds '
                            'default_planner_request_adapters/FixStartStateCollision '
                            'default_planner_request_adapters/FixStartStatePathConstraints',
        'start_state_max_bounds_error': 0.1,
    }
}
ompl_planning_yaml = load_yaml(
    'franka_moveit_config', 'config/ompl_planning.yaml'
)
ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "do_rviz", default_value="true",
            description="to use rviz, or not to use rviz, that is the question."
        ),
        DeclareLaunchArgument(
            "use_realsense", default_value="true",
            description="Use the Realsense Camera. If 'false', will attempt to use usb camera or built in webcam"
        ),
        DeclareLaunchArgument(
            "use_fake_hardware", default_value="true",
            description="Use the real franka robot or not"
        ),

        SetLaunchConfiguration(
            "robot_ip", PythonExpression(["'\"dont-care\" if ", LaunchConfiguration("use_fake_hardware"), " == \"true\" else \"panda0.robot\"'"])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch',
                    'rs_launch.py'
                ])
            ),
            condition=IfCondition(EqualsSubstitution(
                LaunchConfiguration("use_realsense"), "true")),
            launch_arguments={
                "align_depth.enable": "true",
                "pointcloud.enable": "true",
                "colorizer.enable": "true",
                "decimation_filter.enable": "true",
                "spatial_filter.enable": "true",
                "temporal_filter.enable": "true",
                "disparity_filter.enable": "true",
                "hole_filling_filter.enable": "true",
                "hdr_merge.enable": "true",
                "json_file_path": get_package_share_directory("handcv") + "/config/high_density_preset.json",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('franka_moveit_config'),
                    'launch',
                    'moveit.launch.py'
                ])
            ),
            condition=IfCondition(EqualsSubstitution(
                LaunchConfiguration("use_fake_hardware"), "true")),
            launch_arguments={
                "robot_ip": "dont-care",
                "use_fake_hardware": "true",
                "use_rviz": "false",
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('franka_moveit_config'),
                    'launch',
                    'rviz.launch.py'
                ])
            ),
            condition=IfCondition(EqualsSubstitution(
                LaunchConfiguration("use_fake_hardware"), "false")),
            launch_arguments={
                "robot_ip": "panda0.robot",
                "use_fake_hardware": "false",
            }.items()
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            on_exit=Shutdown(),  # when rviz closes, shutdown all nodes
            condition=IfCondition(EqualsSubstitution(
                LaunchConfiguration("do_rviz"), "true")),
            arguments=["-d",
                       PathJoinSubstitution(
                           [FindPackageShare(
                               "handcv"), "config", "handcv.rviz"])],
            parameters=[
                # this entire Command() block is for the robot description
                {"robot_description": Command(
                    [FindExecutable(name='xacro'), ' ', PathJoinSubstitution([FindPackageShare("franka_description"), "robots", "panda_arm.urdf.xacro"]), ' hand:=true',
                     ' robot_ip:=', LaunchConfiguration("robot_ip"),
                     ' use_fake_hardware:=', LaunchConfiguration(
                         "use_fake_hardware"),
                     ' fake_sensor_commands:=', 'false'])},
                {"robot_description_semantic": Command(
                    [FindExecutable(name='xacro'), ' ', PathJoinSubstitution(
                        [FindPackageShare("franka_moveit_config"), "srdf", "panda_arm.srdf.xacro"]), " hand:=true"]
                )},
                # OMPL mption planning pipeline config
                ompl_planning_pipeline_config,
                load_yaml('franka_moveit_config', 'config/kinematics.yaml')
            ]
        ),
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            condition=IfCondition(EqualsSubstitution(
                LaunchConfiguration("use_realsense"), "false")),
            arguments=["-p framerate:=30.0 -p pixel_format:=yuyv"]
        ),
        Node(
            package="handcv",
            executable="handcv",
        ),
        # Node(
        #     package="franka_teleop",
        #     executable="franka_teleop"
        # ),
        # Node(
        #     package="rqt_image_view",
        #     executable="rqt_image_view",
        #     arguments=["/camera/color/image_raw"],
        # )

    ])
