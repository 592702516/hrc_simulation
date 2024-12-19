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

from pycontrol.gripper_sim import Robotiq85Gripper
from pycontrol.robot_sim import UR5eRobot
from pycontrol.camera_sim import AzureKinectCamera

def sig_handler(signal, frame):
    print("Existing Program...")
    sys.exit(0)



if __name__ == "__main__":
    rospy.init_node("combined_control")
    robot = UR5eRobot()

    signal.signal(signal.SIGINT, sig_handler)

    # send robot to home position
    robot.go_home()

   
