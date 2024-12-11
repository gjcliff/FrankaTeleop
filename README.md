# Franka Teleoperation

This is a ROS2 Iron package implementing teleoperation on the Emika Franka Panda
7 DOF robot. The package uses Google's MediaPipe hand tracking and gesture
recognition in combination with MoveIt Servo to control the robot's movement.

Check out the post for this project on my portfolio website: [Franka
Teleoperation](https://graham-clifford.com/Robot-Arm-Teleoperation-Through-Computer-Vision-Hand-Tracking/)

## Introduction
This repository consists of several ROS packages
- franka_teleop: this custom package contains a ROS2 node which implements the
MoveIt! Servo package. It's responsible for controlling to the movement of the robot
using linear and angular increments.
- handcv: a Python ROS2 node combining Google Mediapipe gesture recognition and
hand tracking to provide hand waypoints and state information.
- cv_franka_bridge: A Python ROS2 node that subscribes to the information handcv
provides, and processes it into commands for the franka_teleop node.
- numsr_franka_moveit_config: this custom package contains necessary configuration
files for the Franka robot.
- hand_interfaces: A package containing custom ROS2 messages and services.

## How to Run
### Necessary packages:
* Media Pipe:
```sh
pip install mediapipe
```
* RealSense Camera
    * follow setup instructions here: https://github.com/IntelRealSense/realsense-ros/tree/ros2-development
* MoveIt
```sh
sudo apt install ros-{ROS_DISTRO}-moveit
```

### Setup
Your robot may or may not be connected to the open internet. If it is, you can install
moveit_servo and it's dependencies by running:
```sh
sudo apt install ros-<distro>-moveit-servo
```
If you are like me and have a robot not connected to the internet, you'll have to
clone the necessary repos on your host system and then copy them over to your
workspace. Here's how to do that:
#### Install Necessary Packages
```sh
git clone git@github.com:gjcliff/FrankaTeleop.git
```
```sh
git clone git@github.com:ros-planning/moveit2.git
```
```sh
git clone git@github.com:ros2/ros_testing.git
```
* install these to some temporary location on your host machine.

#### Copy all packages over to the Franka control computer
Now we're going to copy these three packages into a ros workspace on the robot's
computer in a specific order. We have to start with the lowest dependencies first
and work up.

SSH into the robot in one terminal, and create the workspace on the robot if you
haven't already:
```sh
# on robot
ssh username@robot
source /opt/ros/<distro>/setup.bash
mkdir -p ~/Documents/ws/src/
cd ~/Documents/ws/
```
ros_testing is a dependency of moveit_servo, so we need to build it first.

Create a new terminal, and follow the next series of commands to copy
ros_testing to the robot and build it:
```sh
# on host
scp ros_testing/ user@robot:/home/user/Documents/ws/src/
```
```sh
# on robot
colcon build
source install/setup.bash
```
Now that's done, we can do the same thing to moveit_servo:
```sh
# on host
scp moveit_servo/* user@robot:/home/Documents/your/directory/here
```
```sh
# on robot
colcon build
source install/setup.bash
```
And finally, we can copy over the FrankaTeleop package:
```sh
# on host
scp FrankaTeleop/* user@robot:/home/Documents/ws/FrankaTeleop
```
```sh
# on robot
colcon build
source install/setup.bash
```
#### Running Commands
**Real Robot**:
Run on robot:
```sh
ros2 launch cv_franka_bridge integrate_servo.launch.py use_fake_hardware:=false use_rviz:=false robot_ip:=panda0.robot use_realsense:=false run_franka_teleop:=true
```

Run on your computer:
```sh
ros2 launch cv_franka_bridge integrate_servo.launch.py use_fake_hardware:=false use_rviz:=true robot_ip:=panda0.robot use_realsense:=true run_franka_teleop:=false
```

**Simulated Robot**:
Run on your computer:
```sh
ros2 launch cv_franka_bridge integrate_servo.launch.py
```

#### How to Control the Robot
Here's a list of the gestures the system recognizes and what they do:
* **Thumbs Up (Start/Stop Tracking)**: This is one of the gestures used to tell the code to start/stop
tracking the position of your right hand. You also use this gesture to adjust
your hand in the camera frame without moving the robot. While the camera sees
you giving a thumbs up the robot won't move, but once you release your hand
the robot will start tracking your hand.
* **Thumbs Down (Shutdown)**: This gesture is used to tell the code to stop tracking your hand,
and to end the program. You will not be able to give the thumbs up gesture
anymore, and will have to restart the program to start tracking your hand again.
* **Close Fist (Close Gripper)**: This gesture will close the gripper of the robot.
* **Open Palm (Open Gripper)**: This gesture will open the gripper of the robot.
