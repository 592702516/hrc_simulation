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
from pycontrol.sensor import FT300Sensor
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

def robot_pick(robot, camera, height, type):
    count = 0
    last_tx, last_ty, last_rot = 0, 0, 0

    
        # only move when count > 6 (object stable for 3 seconds)
    while count < 7:
        detection = camera.get_detect()
        if count == 0:
            if type == "unpacked":
                last_tx = detection.unpacked_tx / 1000
                last_ty = detection.unpacked_ty / 1000
                last_rot = detection.unpacked_rot
            elif type == "packed":
                last_tx = detection.packed_tx / 1000
                last_ty = detection.packed_ty / 1000
                last_rot = detection.packed_rot
            else:
                raise ValueError("The object type is not supported")

        if type == "unpacked":
            dtx = detection.unpacked_tx / 1000
            dty = detection.unpacked_ty / 1000
            drot = detection.unpacked_rot
        elif type == "packed":
            dtx = detection.packed_tx / 1000
            dty = detection.packed_ty / 1000
            drot = detection.packed_rot
        else:
            raise ValueError("The object type is not supported")

        if drot != -100:
            if np.abs(dtx -last_tx) < 0.01 and np.abs(dty-last_ty) < 0.01 and np.abs(drot - last_rot) < 0.05:
                last_tx = dtx
                last_ty = dty
                last_rot = drot
                current_pose = robot.get_actual_pose()
                diag = np.sqrt((current_pose[0] + dtx)**2 + (current_pose[1] + dty)**2)
                # object is too far or may collide with table
                if diag > 0.92:
                    count = 0
                    rospy.loginfo("Object is beyond robot's reach")
                else:
                    # all check pass, plus count
                    count += 1
                    if count == 3:
                        rospy.loginfo("Object stable, confirming pose...")
            else:
                count = 0
                rospy.loginfo("Object is not stable")
        else:
            count = 0
            rospy.loginfo("Object is not stable")

        time.sleep(0.5)

    picking_list = [] # go to picking position
    cp = robot.get_actual_pose()
    cj = robot.get_joint_pose()
    cj_conv = robot.joint_to_cart([cj[0], cj[1], cj[2], cj[3], cj[4], cj[5] + last_rot])
    picking_list.append([cp[0]+ last_tx, cp[1] + last_ty, height, cj_conv[3], cj_conv[4], cj_conv[5]])
    robot.execute_cartesian_trajectory(picking_list)


def wait_movement(robot, conveyor, conveyor_pos):
    while True:
        if np.sum(np.abs(robot.get_pos_error())) < 0.01:
            if np.abs(conveyor.get_coveyor_stat().current_position - conveyor_pos) < 2:
                break
            else:
                time.sleep(0.1)
        else:
            time.sleep(0.1)
    time.sleep(0.5)

if __name__ == "__main__":
    rospy.init_node("combined_control")
    robot = UR5eRobot()
    gripper = Robotiq85Gripper()
    sensor = FT300Sensor()
    conveyor = ConveyorBelt()
    camera = AzureKinectCamera()

    signal.signal(signal.SIGINT, sig_handler)

    # send robot to home position
    robot.go_home()

    # opening gripper
    conveyor.go_home()
    
    conveyor.set_position(270)


