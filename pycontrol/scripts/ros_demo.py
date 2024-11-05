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
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

from pycontrol.gripper import Robotiq85Gripper
# from pycontrol.sensor import FT300Sensor
from pycontrol.robot import UR5eRobot
from pycontrol.conveyor import ConveyorBelt
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
    # sensor = FT300Sensor()
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

        # send converyor to home position
        robot.go_home()

        over_list =[]
        over_list.append([-0.191, -0.668, 0.250, 0.0, -3.141, 0.0])
        robot.execute_cartesian_trajectory(over_list)
        
        # close gripper
        close_gripper(gripper)

        retract_list = []
        retract_list.append([-0.132, -0.78, 0.35, 0.0, -3.141, 0.0])
        retract_list.append([-0.132, -0.297, 0.272, 0.0, -3.141, 0.0])
        retract_list.append([0.297, -0.132, 0.272, 2.226, -2.217, 0.0])

        robot.execute_cartesian_trajectory(retract_list)

        handover_list = []
        handover_list.append([0.349, -0.045, 0.240, 3.141, 0.037, 0.0])
        robot.execute_cartesian_trajectory(handover_list)

        open_gripper(gripper)

        observe_list = []
        observe_list.append([0.586, -0.132, 0.663, 2.227, -2.217, 0.0])
        robot.execute_cartesian_trajectory(observe_list)

        # close gripper
        close_gripper(gripper)

        place_list= []
        place_list.append([0.297, -0.132, 0.348, 2.226, -2.217, 0.0])
        place_list.append([0.077, 0.656, 0.200, 2.176, -2.267, 0.0])
        robot.execute_cartesian_trajectory(place_list)

        # open gripper
        open_gripper(gripper)

        robot.go_home()
        time.sleep(2)
