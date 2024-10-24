import rospy
import actionlib
import time
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import Float64

bridge = CvBridge()

def send_gripper_command(controller_name, joint_names, positions, duration=2.0):
    client = actionlib.SimpleActionClient(f'/{controller_name}/follow_joint_trajectory', FollowJointTrajectoryAction)
    rospy.loginfo(f'Waiting for {controller_name} action server...')
    client.wait_for_server()
    rospy.loginfo(f'{controller_name} action server is up. Sending trajectory...')

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = joint_names

    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = rospy.Duration(duration)

    goal.trajectory.points.append(point)
    goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)

    client.send_goal(goal)
    client.wait_for_result()

    if client.get_result():
        rospy.loginfo(f'{controller_name} executed successfully.')
    else:
        rospy.logwarn(f'{controller_name} did not execute successfully.')

def image_callback(msg):
    try:
        # Convert ROS Image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
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

        if radius > 10:  # Threshold to ensure the detected object is significant
            rospy.loginfo(f"Detected red cube at coordinates: ({x}, {y}) with radius: {radius}")
            
            # Close the gripper to pick the cube
            send_gripper_command(
                controller_name='gripper_controller',
                joint_names=['gripper_finger1_joint'],
                positions=[0.5],  # Close gripper
                duration=3.0
            )

            rospy.sleep(2)

            # Open the gripper to release the cube
            send_gripper_command(
                controller_name='gripper_controller',
                joint_names=['gripper_finger1_joint'],
                positions=[0.0],  # Open gripper
                duration=3.0
            )

def main():
    rospy.init_node('camera_robot_integration_node')

    # Subscribe to the correct camera topic to detect the red cube
    rospy.Subscriber('/azure_kinect_camera_1/color/image_raw', Image, image_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
