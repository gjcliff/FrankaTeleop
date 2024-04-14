# Franka Teleoperation

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
```$ pip install mediapipe```
* RealSense Camera
    * follow setup instructions here: https://github.com/IntelRealSense/realsense-ros/tree/ros2-development
* MoveIt  
```$ sudo apt install ros-{ROS_DISTRO}-moveit```

### If you're in the MSR Lab at Northwestern
The Franka robots we have DO NOT have moveit_servo installed on their control
computers. How unfortunate! Because of this, we'll have to copy over and build
moveit_servo and any other dependencies we're missing.
#### Install Necessary Packages  
```$ git clone git@github.com:gjcliff/FrankaTeleop.git```  
```$ git clone git@github.com:ros-planning/moveit2.git```  
```$ git clone git@github.com:ros2/ros_testing.git```  
* install these to some temporary location on your local drive.

#### Copy all packages over to the Franka control computer
* Choose a reasonable and considerate directory on the Franka control computer
to copy the moveit_servo, ros_testing, and Franka-Teleop repositories into.
* **IMPORTANT** only copy the moveit_servo direectory from the moveit2 package.
The computer already has most of the moveit directories you'll need, just not
moveit_servo. You can find moveit_servo in "moveit2/moveit_ros/moveit_servo"
* Copy our packages by running these commands (you must have an active ethernet
connection to the Franka):  
```$ scp FrankaTeleop/* student@station:/home/Documents/your/directory/here```  
```$ scp moveit_servo/* student@station:/home/Documents/your/directory/here```  
```$ scp ros_testing/* student@station:/home/Documents/your/directory/here```  
* build each of these packages and then source their install/setup.bash files
```$ source FrankaTeleop/install/setup.bash```  
```$ source moveit_servo/install/setup.bash```  
```$ source ros_testing/install/setup.bash```  

You might not have to do this, as I installed these files myself on two of the
robots under ~/Documents/graham_winter_project.  

#### Running Commands
**Real Robot**:
Run on robot: ros2 launch cv_franka_bridge integrate_servo.launch.py use_fake_hardware:=false use_rviz:=false robot_ip:=panda0.robot use_realsense:=false run_franka_teleop:=true

Run on your computer: ros2 launch cv_franka_bridge integrate_servo.launch.py use_fake_hardware:=false use_rviz:=true robot_ip:=panda0.robot use_realsense:=true run_franka_teleop:=false  

**Simulated Robot**:
Run on your computer: ros2 launch cv_franka_bridge integrate_servo.launch.py

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
