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
import signal

from pycontrol.gripper_sim import Robotiq85Gripper


def sig_handler(signal, frame):
    print("Exiting Program...")
    sys.exit(0)

def open_gripper(gripper):
    success = gripper.open()
    if success:
        rospy.loginfo('Successfully opened gripper')
        time.sleep(2)
    else:
        rospy.loginfo('Open gripper failed')
        raise Exception("Cannot open gripper")
    time.sleep(1)

def close_gripper(gripper):
    success = gripper.close()
    if success:
        rospy.loginfo('Successfully closed gripper')
        time.sleep(1)
    else:
        rospy.loginfo('Close gripper failed')
        raise Exception("Cannot close gripper")
    time.sleep(1)

if __name__ == "__main__":
    rospy.init_node("gripper_test")
    gripper = Robotiq85Gripper()

    signal.signal(signal.SIGINT, sig_handler)

    # Test opening and closing the gripper
    while True:
        try:
            rospy.loginfo("Opening gripper...")
            open_gripper(gripper)
            time.sleep(2)

            rospy.loginfo("Closing gripper...")
            close_gripper(gripper)
            time.sleep(2)
        except Exception as e:
            rospy.logerr(f"Error: {e}")
            break

