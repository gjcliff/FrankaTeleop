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

## Config Files
### Fake Panda Config Files
* panda_arm_real.urdf
    * urdf for the real panda arm, needs a robot_ip parameter (handled inside the launch file)
* panda_mock_controllers.yaml
### Real Panda Config Files
* panda_arm_fake.urdf
    * urdf for the simulated panda arm.
### Shared Panda Config Files
* panda_controllers.yaml
    * config for the moveit_simple_controller_manager
* panda_arm.srdf
* moveit_cpp.yaml
* kinematics.yaml
* moveit.rviz
### Not Used
* moveit_controllers.yaml
arm_id:=panda robot_ip:=dont-care use_fake_hardware:=true
* 
