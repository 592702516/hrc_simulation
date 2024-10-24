import rospy
import sys
import actionlib
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import Float64
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

bridge = CvBridge()
depth_image = None
red_cube_detected = False
cube_position = None

def depth_callback(msg):
    global depth_image
    try:
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

# Function to convert pixel coordinates to real-world coordinates (using depth)
def pixel_to_world(x, y):
    global depth_image
    if depth_image is None:
        rospy.logwarn("Depth data is not yet available.")
        return None, None, None

    # Ensure coordinates are within bounds
    if int(y) >= depth_image.shape[0] or int(x) >= depth_image.shape[1]:
        rospy.logwarn("Coordinates are out of bounds.")
        return None, None, None

    # Get depth value at (x, y)
    depth_value = depth_image[int(y), int(x)]
    if np.isnan(depth_value) or depth_value <= 0:
        rospy.logwarn("Invalid depth value for the given pixel coordinates.")
        return None, None, None

    # Example conversion, assuming simple pinhole camera model
    fx, fy = 525.0, 525.0  # Focal length in pixels (example values)
    cx, cy = 320.0, 240.0  # Principal point (example values)

    # Converting from camera coordinates to world coordinates
    real_world_x = (x - cx) * depth_value / fx
    real_world_y = (y - cy) * depth_value / fy
    real_world_z = depth_value

    return real_world_x, real_world_y, real_world_z

def image_callback(msg):
    global red_cube_detected, cube_position, depth_image
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return

    if depth_image is None:
        rospy.logwarn("Depth data is not yet available.")
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
            rospy.loginfo(f"Detected red cube at pixel coordinates: ({x}, {y}) with radius: {radius}")
            real_world_x, real_world_y, real_world_z = pixel_to_world(x, y)
            if real_world_x is not None:
                # Convert to world coordinates
                cube_position = (real_world_x, real_world_y, real_world_z)
                
                red_cube_detected = True

def move_to_position(target_pose):
    group.set_pose_target(target_pose)
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

def main():
    global red_cube_detected, cube_position
    rospy.init_node('red_cube_pick_node')

    # Initialize MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    global group
    group = moveit_commander.MoveGroupCommander("ur5e")  # "manipulator" is the default name for UR5e arm

    rospy.Subscriber('/azure_kinect_camera_1/color/image_raw', Image, image_callback)
    rospy.Subscriber('/azure_kinect_camera_1/depth/image_raw', Image, depth_callback)

    rate = rospy.Rate(2)  # 2 Hz
    while not rospy.is_shutdown():
        if red_cube_detected and cube_position is not None:
            rospy.loginfo(f"Moving UR5e to position above the red cube at: {cube_position}")

            # Create a target_pose for the UR5e to reach 10 cm above the detected red cube
            target_pose = geometry_msgs.msg.Pose()
            target_pose.position.x = cube_position[0]
            target_pose.position.y = cube_position[1]
            target_pose.position.z = cube_position[2] + 0.1  # Move 10 cm above the cube
            target_pose.orientation.w = 1.0  # Set orientation as needed

            # Move UR5e to the target position
            move_to_position(target_pose)

            # Close the gripper
            rospy.sleep(2)  # Add delay for stable positioning before closing gripper

            # After picking, move up by 30 cm
            target_pose.position.z += 0.3
            move_to_position(target_pose)

            red_cube_detected = False
            cube_position = None

        rate.sleep()

if __name__ == '__main__':
    main()
