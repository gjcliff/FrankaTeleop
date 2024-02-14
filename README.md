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

## Setup Instructions
1. Create a new ros2 workspace ```$ mkdir -p ws/handcv/src```
2. cd into the src/ directory, and clone the repository into your workspace and run ```$ colcon build```
3. Source the install directory with ```$ source install/setup.{your_shell}```
4. Run the following setup code from your workspace directory, courtesy of [Matt Elwin](https://github.com/m-elwin)
```
# Create a new ros 2 workspace
mkdir -p franka/src
cd franka

# Clone the repositories
vcs import --recursive --input \
    https://raw.githubusercontent.com/m-elwin/numsr_patches/iron_patches/iron_patches.repos src

# Install dependencies
rosdep install --from-paths src -r -y

# Add the numsr colcon mixin
colcon mixin add numsr_patches file://$(pwd)/src/numsr_patches/index.yaml
colcon mixin update numsr_patches

# Build the workspace
colcon build --mixin numsr
```
5. Source the install directory of the Franka robot with ```$ source install/setup.{your_shell}```
6. Copy the install directory of the handcv workspace onto the Franka's control pc using
```
$ rsync -av --delete install/ username@ip_address:/home/username/
```
7. ssh into the control pc, and source the install directory that you just copied.
8. To launch the Franka control nodes, run
```
$ ros2 launch franka_teleop franka_msr.launch.py use_fake_hardware:=false robot_ip:={your_robot_ip} use_rviz:=false
```
9. The view the robot in rviz on your own computer, you can run
```
$ ros2 launch franka_teleop franka_rviz.launch.py
```
## Usage
* There are four services currently available for controlling the Franka, more are coming soon
    1. **/plan_path**: Use IK to plan a path for the franka's end-effector to a specific pose
    2. **/execute_path**: Execute the path most recently planned for the Franka. You MUST call /plan_path service first
    3. **/plan_and_execute_path**: Same as /plan_path, but the path is executed right away.
    4. **/ready**: Plans and executes a path to bring the Franka to the ready position










