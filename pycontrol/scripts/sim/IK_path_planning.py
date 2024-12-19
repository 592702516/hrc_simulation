#!/usr/bin/env python
#
# \author  Rui Zhou rzho774@aucklanduni.ac.nz
# \date    2024-11-01
# \detect red cube and pick it up use IK path planning


import rospy
import sys
import actionlib
import moveit_commander
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import Float64
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import PointStamped
from pycontrol.gripper_sim import Robotiq85Gripper
from pycontrol.robot_sim import UR5eRobot

bridge = CvBridge()
depth_image = None
target_position = None  # Initialize target_position globally
position_logged = False  # Flag to track if the red cube's position has been logged

def compute_inverse_kinematics(target_position):
    group = moveit_commander.MoveGroupCommander("ur5e")
    target_pose = group.get_current_pose().pose
    target_pose.position.x = target_position[0]
    target_pose.position.y = target_position[1]
    target_pose.position.z = target_position[2]

    # Set the pose target with increased tolerance
    group.set_goal_position_tolerance(0.05)
    group.set_goal_orientation_tolerance(0.1)
    group.set_pose_target(target_pose)

    # Plan the trajectory to the target pose
    rospy.loginfo("Planning to target pose...")
    success, plan, planning_time, error_code = group.plan()
    rospy.loginfo(f"Planning result: success={success}, planning_time={planning_time}, error_code={error_code}")

    if success and plan.joint_trajectory.points:
        joint_positions = plan.joint_trajectory.points[-1].positions
        rospy.loginfo(f"Computed joint positions: {joint_positions}")
        print(f"Computed joint positions: {joint_positions}")
        return joint_positions
    else:
        rospy.logwarn("No valid joint positions found for the given target position.")
        return None

def depth_callback(msg):
    global depth_image
    try:
        # Convert ROS Image message to OpenCV format for depth data
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    except CvBridgeError as e:
        rospy.logerr("CvBridge error: {0}".format(e))

def pixel_to_world(x, y):
    if depth_image is None:
        rospy.logwarn("Depth data not available.")
        return None, None, None

    # Ensure coordinates are within bounds
    if int(y) >= depth_image.shape[0] or int(x) >= depth_image.shape[1]:
        rospy.logwarn("Out of range")
        return None, None, None

    # Get depth value at (x, y)
    depth_value = depth_image[int(y), int(x)]
    if np.isnan(depth_value) or depth_value <= 0:
        rospy.logwarn("Depth data not available.")
        return None, None, None

    # Example conversion, assuming simple pinhole camera model
    fx, fy = 525.0, 525.0  
    cx, cy = 320.0, 240.0 

    camera_x = (x - cx) * depth_value / fx
    camera_y = (y - cy) * depth_value / fy
    camera_z = depth_value

    # Transform the camera coordinates to world coordinates based on the fixed camera position
    world_x = - camera_y + 0.5  # Camera position in the world frame (x = 0.5 m)
    world_y = - camera_x
    world_z = 2.0684 - camera_z   # Camera height in the world frame (z = 2.0 m)

    return world_x, world_y, world_z

def image_callback(msg):
    global target_position, position_logged  # Make target_position global to use in main(), and track if position is logged
    if position_logged:
        return  # Exit the callback if the position has already been logged

    try:
        # Convert ROS Image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except CvBridgeError as e:
        rospy.logerr("CvBridge error: {0}".format(e))
        return

    # Convert the image to HSV color space for red detection
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 120, 70])
    upper_red = np.array([10, 255, 255])
    mask = cv2.inRange(hsv_image, lower_red, upper_red)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        # Get the largest contour to detect the red cube
        c = max(contours, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(c)

        if radius > 10: 
            rospy.loginfo(f"detec the red cube at ({x}, {y})，The radius is: {radius}")

            real_world_x, real_world_y, real_world_z = pixel_to_world(x, y)
            if real_world_x is not None:
                rospy.loginfo(f"The world coordinates of red cube: ({real_world_x}, {real_world_y}, {real_world_z})")
                target_position = [real_world_x, real_world_y, real_world_z + 0.1]  # Adjust to 10cm above the detected position
                position_logged = True  # Set the flag to True after logging the position

def main():
    rospy.init_node('test_ur5e_gripper_controllers')
    robot = UR5eRobot()
    gripper = Robotiq85Gripper()

    rospy.Subscriber('/azure_kinect_camera_1/color/image_raw', Image, image_callback)
    rospy.Subscriber('/azure_kinect_camera_1/depth/image_raw', Image, depth_callback)

    try:
        # Wait for the red cube position to be detected
        rospy.sleep(1)
        while target_position is None:
            rospy.sleep(0.5)

        # Compute the joint positions for the given target world position
        joint_positions = [compute_inverse_kinematics(target_position)]
        rospy.loginfo(joint_positions)

        if joint_positions:
            robot.execute_joint_trajectory(joint_positions)

            time.sleep(2)

            gripper.close()

            rospy.sleep(2)

            target_position[2] += 0.3

            joint_positions_up = [compute_inverse_kinematics(target_position)]

            if joint_positions_up:
                robot.execute_joint_trajectory(joint_positions_up)
            
            time.sleep(2)
            gripper.open()

    except rospy.ROSInterruptException:
        rospy.logerr('Interrupted during execution')

if __name__ == '__main__':
    main()
