#!/usr/bin/env python
#!\file
#
# \author  Wanqing Xia wxia612@aucklanduni.ac.nz
# \date    2022-07-22
#
#
# ---------------------------------------------------------------------

import sys
import time
import rospy
import numpy as np
import signal

from pycontrol.gripper import Robotiq85Gripper

from pycontrol.robot import UR5eRobot

from pycontrol.camera import AzureKinectCamera

def sig_handler(signal, frame):
    print("Existing Program...")
    sys.exit(0)

def open_gripper(gripper):
    success = gripper.open()
    if success:
        rospy.loginfo('Successfully opened')
        time.sleep(2)
    else:
        rospy.loginfo('Open gripper failed')
        raise Exception("Cannot open gripper")
    time.sleep(1)

def close_gripper(gripper):
    success = gripper.close()
    if success:
        rospy.loginfo('Successfully closed')
        time.sleep(1)
    else:
        rospy.loginfo('Close gripper failed')
        raise Exception("Cannot close gripper")
    time.sleep(1)



if __name__ == "__main__":
    rospy.init_node("combined_control")
    robot = UR5eRobot()
    gripper = Robotiq85Gripper()
    camera = AzureKinectCamera()

    signal.signal(signal.SIGINT, sig_handler)

    # send robot to home position
    robot.go_home()

    # opening gripper
    open_gripper(gripper)

    while True:
        # turn over to conveyor side
        pose_list = []
        pose_list.append([0.297, -0.132, 0.272, 2.226, -2.217, 0.0])
        pose_list.append([-0.132, -0.297, 0.272, 0.0, -3.141, 0.0])
        robot.execute_cartesian_trajectory(pose_list)


        over_list =[]
        over_list.append([-0.132, -0.803, 0.478, 0.0, -3.141, 0.0])
        robot.execute_cartesian_trajectory(over_list)

        
        # close gripper
        close_gripper(gripper)

        retract_list = []
        retract_list.append([-0.132, -0.78, 0.35, 0.0, -3.141, 0.0])
        retract_list.append([-0.132, -0.297, 0.272, 0.0, -3.141, 0.0])
        retract_list.append([0.297, -0.132, 0.272, 2.226, -2.217, 0.0])

        robot.execute_cartesian_trajectory(retract_list)


        handover_list = []
        handover_list.append([0.586, -0.132, 0.233, 2.52, -2.51, 1.797])
        robot.execute_cartesian_trajectory(handover_list)

        observe_list = []
        observe_list.append([0.586, -0.132, 0.233, 2.227, -2.217, 0.0])
        observe_list.append([0.586, -0.132, 0.663, 2.227, -2.217, 0.0])
        robot.execute_cartesian_trajectory(observe_list)

        # close gripper
        close_gripper(gripper)

        robot.go_home()

        place_list= []
        place_list.append([0.297, -0.132, 0.272, 2.217, -2.217, 0.0])
        place_list.append([0.175, 0.273, 0.272, 3.14, -0.231, 0.0])
        robot.execute_cartesian_trajectory(place_list)

        # open gripper
        open_gripper(gripper)

        robot.go_home()

