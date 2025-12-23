from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controllers_yaml",
                default_value=PathJoinSubstitution(
                    [
                        # FindPackageShare("franka_teleop_bringup"),
                        # "config",
                        # "panda_mock_controllers.yaml",
                        FindPackageShare("franka_bringup"),
                        "config",
                        "controllers.yaml",
                    ]
                ),
                description="Override the default controllers.yaml file.",
            ),
            DeclareLaunchArgument(
                name="arm_id",
                default_value="fr3",
                description="ID of the type of arm used",
            ),
            DeclareLaunchArgument(
                name="use_fake_hardware",
                default_value="true",
                description="whether or not to use fake hardware.",
            ),
            DeclareLaunchArgument(
                name="use_rviz",
                default_value="true",
                description="whether or not to use rviz.",
            ),
            DeclareLaunchArgument(
                name="robot_ip",
                default_value="dont-care",
                description="IP address of the robot",
            ),
            DeclareLaunchArgument(
                name="use_realsense",
                default_value="true",
                description="whether or not to use realsense camera.",
            ),
            DeclareLaunchArgument(
                name="rviz_file",
                default_value="integrate_servo.rviz",
                description="rviz file to use.",
            ),
            DeclareLaunchArgument(
                name="x_limits",
                default_value="0.2,0.6",
                description="x limits for the bounding box of\
                                          the end effector. Format: min,max",
            ),
            DeclareLaunchArgument(
                name="y_limits",
                default_value="-0.25,0.25",
                description="y limits for the bounding box of\
                                          the end effector. Format: min,max",
            ),
            DeclareLaunchArgument(
                name="z_limits",
                default_value="0.1,0.6",
                description="z limits for the bounding box of\
                                          the end effector. Format: min,max",
            ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         [
            #             PathJoinSubstitution(
            #                 [
            #                     FindPackageShare("franka_bringup"),
            #                     "launch",
            #                     "franka.launch.py",
            #                 ]
            #             )
            #         ]
            #     ),
            #     launch_arguments={
            #         "controllers_yaml": LaunchConfiguration(
            #             "controllers_yaml"
            #         ),
            #         "use_fake_hardware": LaunchConfiguration(
            #             "use_fake_hardware"
            #         ),
            #         "robot_ip": LaunchConfiguration("robot_ip"),
            #     }.items(),
            # ),
            # Node(
            #     package="rviz2",
            #     executable="rviz2",
            #     name="rviz2",
            #     arguments=[
            #         "--display-config",
            #         PathJoinSubstitution(
            #             [
            #                 FindPackageShare("franka_description"),
            #                 "rviz",
            #                 "visualize_franka.rviz",
            #             ]
            #         ),
            #     ],
            #     output="screen",
            # ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         [
            #             PathJoinSubstitution(
            #                 [
            #                     FindPackageShare("franka_teleop"),
            #                     "launch",
            #                     "franka_servo.launch.py",
            #                 ]
            #             )
            #         ]
            #     ),
            #     launch_arguments={
            #         "robot_ip": LaunchConfiguration("robot_ip"),
            #         "use_fake_hardware": LaunchConfiguration(
            #             "use_fake_hardware"
            #         ),
            #         "use_rviz": "false",
            #     }.items(),
            # ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("handcv"),
                                "launch",
                                "camera.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "use_realsense": LaunchConfiguration("use_realsense")
                }.items(),
            ),
            # Node(
            #     package="cv_franka_bridge",
            #     executable="cv_franka_bridge",
            #     output="screen",
            #     parameters=[
            #         {"x_limits": LaunchConfiguration("x_limits")},
            #         {"y_limits": LaunchConfiguration("y_limits")},
            #         {"z_limits": LaunchConfiguration("z_limits")},
            #     ],
            # ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--yaw",
                    "-1.5708",
                    "--pitch",
                    "0",
                    "--roll",
                    "-1.5708",
                    "--frame-id",
                    "panda_link0",
                    "--child-frame-id",
                    "camera_link",
                ],
            ),
        ]
    )
