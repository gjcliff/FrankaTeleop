#!/usr/bin/python

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
import math
import time

# This script makes the end-effector perform pick, pour, and place tasks
# Note that this script may not work for every arm as it was designed for the wx250
# Make sure to adjust commanded joint positions and poses as necessary
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250'
# Then change to this directory and type 'python bartender.py  # python3 bartender.py if using ROS Noetic'

bot = InterbotixManipulatorXS("px100", "arm", "gripper")
ee_origin = [260, 150, -60] # from camera perspective: (x, y, z)

def camera_coor_to_robot_coor(camera_point):
    robot_point = [float(camera_point[0]),float(camera_point[1]),float(camera_point[2])]
    return robot_point
def main():
    f1 = open("/tmp/cv_fifo", "r")
    camera_point = []


    while(True):
        for i in range(3):
            camera_point.append(f1.readline())
            print(camera_point[i])

        # ROUTINE
        robot_point = camera_coor_to_robot_coor(camera_point)
        print(f"origin_point: {ee_origin}")
        print(f"robot_point: {robot_point}")
        print(f"x move: {ee_origin[0] - robot_point[0]}, y move: {ee_origin[1] - robot_point[1]}, z move: {ee_origin[2] - robot_point[2]}")
        # bot.arm.go_to_home_pose()robot_point

        # y = ee_origin[1] - robot_point[1]
        # x = (ee_origin[0] - robot_point[0]) * -1
        # print(f"y: {y}, x: {x}")
        
        # print(f"angle: {angle}")

        # Differences
        xDiff = ee_origin[1] - robot_point[1]
        yDiff = (ee_origin[0] - robot_point[0]) * -1
        zDiff = (ee_origin[2] - robot_point[2]) * -1

        angle = np.arctan2(yDiff,xDiff + 100) # adding 50mm to the xDiff, it's an estimation of how far away the gripper is from the center of the waist joint. This angle is from the waist joint's POV

        print(f"yDiff: {yDiff}, xDiff: {xDiff}, angle: {angle}")

        # # direction factors
        # xDir = xDiff / abs(xDiff)
        # zDir = zDiff / abs(zDiff)

        xD = math.sqrt(xDiff**2 + yDiff**2)/1000 # it doesn't matter if the pen is in the positive or negative x direction, the robot turns and then any point is in the positive x direction

        zD = (zDiff)/1000

        print(f"xD: {xD} zd: {zD}")

        # bot.gripper.grasp(2.0)
        # bot.gripper.set_pressure(1.0)

        try:
            bot.arm.go_to_sleep_pose()
            bot.arm.set_single_joint_position("waist",-angle)
            bot.arm.set_ee_cartesian_trajectory(x=xD,z=zD)
            bot.gripper.grasp(1)
            bot.arm.go_to_sleep_pose()
            bot.gripper.release(1)
        except:
            pass

        time.sleep(2)
        
        with open("fifo_empty.txt", "w") as file:
            file.write("0")

        camera_point.clear()


if __name__=='__main__':
    main()
