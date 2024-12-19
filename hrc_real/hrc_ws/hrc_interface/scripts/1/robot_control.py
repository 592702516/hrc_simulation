#!/usr/bin/env python

import numpy as np
import rospy
import time
from libs.robot import UR5eRobot
from libs.gripper import Robotiq85Gripper

class RobotControl:
    def __init__(self):
        self.robot = UR5eRobot()
        self.gripper = Robotiq85Gripper()

    def move_robot(self, position, wait=True, time=0):
        self.robot.execute_cartesian_trajectory(np.array(position))
        print(f"Moving to {position}")

    def open_gripper(self):
        success = self.gripper.open()
        if success:
            rospy.loginfo('Successfully opened')
            time.sleep(2)
        else:
            rospy.loginfo('Open gripper failed')
            raise Exception("Cannot open gripper")
        time.sleep(1)

    def close_gripper(self):
        success = self.gripper.close()
        if success:
            rospy.loginfo('Successfully closed')
            time.sleep(1)
        else:
            rospy.loginfo('Close gripper failed')
            raise Exception("Cannot close gripper")
        time.sleep(1)

    def move_robot_home(self, wait=True, time=0):
        self.move_robot(np.array([0.297, -0.132, 0.250, 2.231, -2.216, 0.0]), wait, time)
        print("Successfully moved robot to home")