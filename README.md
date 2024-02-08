# Franka Teleoperation
* Graham Clifford
* Winter 2024
# Package List
This repository consists of several ROS packages
- franka_teleop - this custom package contains a moveit_cpp node that is reponsible for planning and executing paths for the franka robot.
- numsr_franka_moveit_config - this custom package contains necessary files for configuration of the franka robot. This file doesn't run any nodes, however it offer configuration files necessary for launching the franka and controlling it both in simulation and the real world.

# Setup
## Necessary Python packages:
* Media Pipe:
```pip install mediapipe```

## Necessary ROS packages:
* RealSense Camera
    * follow setup instructions here: https://github.com/IntelRealSense/realsense-ros/tree/ros2-development
* MoveIt
```console
$ sudo apt install ros-{$ROS_DISTRO}-moveit
```

## Necessary Repositories
* Emika Franka Panda repository
    * TODO