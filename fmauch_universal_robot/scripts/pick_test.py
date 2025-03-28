# pick test
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
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

bridge = CvBridge()
depth_image = None
target_position = None  # Initialize target_position globally

def send_trajectory(controller_name, joint_names, positions, duration=5.0):
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
        rospy.logerr("CvBridge 错误: {0}".format(e))

def pixel_to_world(x, y):
    global depth_image
    if depth_image is None:
        rospy.logwarn("深度数据尚未可用。")
        return None, None, None

    # Ensure coordinates are within bounds
    if int(y) >= depth_image.shape[0] or int(x) >= depth_image.shape[1]:
        rospy.logwarn("坐标超出范围。")
        return None, None, None

    # Get depth value at (x, y)
    depth_value = depth_image[int(y), int(x)]
    if np.isnan(depth_value) or depth_value <= 0:
        rospy.logwarn("给定像素坐标的深度值无效。")
        return None, None, None

    # Example conversion, assuming simple pinhole camera model
    fx, fy = 525.0, 525.0  # Focal length in pixels (example values)
    cx, cy = 320.0, 240.0  # Principal point (example values)

    camera_x = (x - cx) * depth_value / fx
    camera_y = (y - cy) * depth_value / fy
    camera_z = depth_value

    # Transform the camera coordinates to world coordinates based on the fixed camera position
    world_x = camera_x + 0.5  # Camera position in the world frame (x = 0.5 m)
    world_y = camera_y
    world_z = camera_z   # Camera height in the world frame (z = 2.0 m)

    return world_x, world_y, world_z

def image_callback(msg):
    global target_position  # Make target_position global to use in main()
    try:
        # Convert ROS Image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except CvBridgeError as e:
        rospy.logerr("CvBridge 错误: {0}".format(e))
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
            rospy.loginfo(f"在像素坐标 ({x}, {y}) 处检测到红色立方体，半径为: {radius}")

            # Convert pixel coordinates to world coordinates
            real_world_x, real_world_y, real_world_z = pixel_to_world(x, y)
            if real_world_x is not None:
                rospy.loginfo(f"红色立方体的实际世界坐标: ({real_world_x}, {real_world_y}, {real_world_z})")
                target_position = [real_world_x, real_world_y, real_world_z + 0.1]  # Adjust to 10cm above the detected position

def main():
    rospy.init_node('test_ur5e_gripper_controllers')

    # Initialize MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("ur5e")

    rospy.Subscriber('/azure_kinect_camera_1/color/image_raw', Image, image_callback)
    rospy.Subscriber('/azure_kinect_camera_1/depth/image_raw', Image, depth_callback)

    try:
        # Wait for the red cube position to be detected
        rospy.sleep(2)
        while target_position is None:
            rospy.sleep(1)

        # Compute the joint positions for the given target world position
        joint_positions = compute_inverse_kinematics(target_position)

        if joint_positions:
            # Move the robot to the computed joint positions
            send_trajectory(
                controller_name='ur5e_controller',
                joint_names=[
                    'workbench_joint',
                    'shoulder_pan_joint',
                    'shoulder_lift_joint',
                    'elbow_joint',
                    'wrist_1_joint',
                    'wrist_2_joint',
                    'wrist_3_joint'
                ],
                positions=joint_positions,
                duration=5.0
            )

        # Adding a delay before testing the gripper
        time.sleep(2)

        # Testing Gripper controller using action client
        send_gripper_command(
            controller_name='gripper_controller',
            joint_names=['gripper_finger1_joint'],
            positions=[0.5],  # Assuming the gripper is closing
            duration=3.0
        )
        time.sleep(2)
        # Testing Gripper controller using action client
        send_gripper_command(
            controller_name='gripper_controller',
            joint_names=['gripper_finger1_joint'],
            positions=[0],  # Assuming the gripper is opening
            duration=3.0
        )

    except rospy.ROSInterruptException:
        rospy.logerr('Interrupted during execution')

if __name__ == '__main__':
    main()
