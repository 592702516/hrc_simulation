#!/usr/bin/env python
#
# \author  Rui Zhou rzho774@aucklanduni.ac.nz
# \date    2024-11-01
# \detect the red cube, pick it up and place it.
# \before you run this code, you should add a red cube in this simworld, and check the camera can detect it.

import rospy
import sys
import moveit_commander
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
import numpy as np
from geometry_msgs.msg import Pose
from pycontrol.gripper import Robotiq85Gripper
from pycontrol.robot import UR5eRobot


bridge = CvBridge()
depth_image = None
target_position = None
position_logged = False


def depth_callback(msg):
    global depth_image
    try:
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

def pixel_to_world(x, y):
    if depth_image is None:
        rospy.logwarn("Depth data not available.")
        return None, None, None

    if int(y) >= depth_image.shape[0] or int(x) >= depth_image.shape[1]:
        rospy.logwarn("Coordinates out of range.")
        return None, None, None

    depth_value = depth_image[int(y), int(x)]
    if np.isnan(depth_value) or depth_value <= 0:
        rospy.logwarn("Invalid depth value at given pixel coordinates.")
        return None, None, None

    fx, fy = 525.0, 525.0  # Example focal lengths
    cx, cy = 320.0, 240.0  # Example principal points

    camera_x = (x - cx) * depth_value / fx
    camera_y = (y - cy) * depth_value / fy
    camera_z = depth_value

    world_x = -camera_y + 0.5
    world_y = -camera_x
    world_z = 2.0684 - camera_z

    return world_x, world_y, world_z

def image_callback(msg):
    global target_position, position_logged
    if position_logged:
        return
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return

    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 120, 70])
    upper_red = np.array([10, 255, 255])
    mask = cv2.inRange(hsv_image, lower_red, upper_red)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(c)

        if radius > 10:
            rospy.loginfo(f"Detected red cube at pixel coordinates ({x}, {y}), radius: {radius}")
            real_world_x, real_world_y, real_world_z = pixel_to_world(x, y)
            if real_world_x is not None:
                rospy.loginfo(f"Red cube world coordinates: ({real_world_x}, {real_world_y}, {real_world_z})")
                target_position = [real_world_x, real_world_y, real_world_z + 0.12]
                position_logged = True

def main():
    rospy.init_node('test_ur5e_gripper_controllers')
    robot = UR5eRobot()
    gripper = Robotiq85Gripper()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.Subscriber('/azure_kinect_camera_1/color/image_raw', Image, image_callback)
    rospy.Subscriber('/azure_kinect_camera_1/depth/image_raw', Image, depth_callback)

    try:
        rospy.sleep(0.5)
        while target_position is None or depth_image is None:
            rospy.sleep(0.5)

        # move below the target object 
        target_position[2] += 0.15

        waypoints = [target_position]
        robot.execute_cartesian_trajectory(waypoints)
        rospy.sleep(0.5)

        # down to pick the object up
        target_position[2] -= 0.15
        waypoints = [target_position]

        robot.execute_cartesian_trajectory(waypoints)

        rospy.sleep(0.5)

        # close the gripper
        gripper.close()

        rospy.sleep(2)

        # place to the orginal position
        target_position[2] += 0.3
        waypoints = [target_position]

        robot.execute_cartesian_trajectory(waypoints)

        rospy.sleep(0.5)

        target_position_home = [0.5,0.0,1.2]
        waypoints = [target_position_home]
        robot.execute_cartesian_trajectory(waypoints)

        # open the gripper
        gripper.open()

    except rospy.ROSInterruptException:
        rospy.logerr('stop')

if __name__ == '__main__':
    main()
