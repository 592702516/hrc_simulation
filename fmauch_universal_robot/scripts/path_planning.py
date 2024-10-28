# detect red cube and pick it up
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
from moveit_msgs.msg import OrientationConstraint, Constraints

bridge = CvBridge()
depth_image = None
target_position = None  # Initialize target_position globally
position_logged = False  # Flag to track if the red cube's position has been logged

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
    
    # Set the target pose with the desired position
    target_pose = group.get_current_pose().pose
    target_pose.position.x = target_position[0]
    target_pose.position.y = target_position[1]
    target_pose.position.z = target_position[2]

    # Define orientation to keep the wrist_2_link pointing down
    target_pose.orientation.x = 0.0
    target_pose.orientation.y = 1.0  # This depends on the desired downward pointing direction
    target_pose.orientation.z = 0.0
    target_pose.orientation.w = 0.0

    # Set pose target with orientation
    group.set_start_state_to_current_state()
    group.set_start_state_to_current_state()
    group.set_pose_target(target_pose)

    # Create an orientation constraint for the wrist to point down
    ocm = OrientationConstraint()
    ocm.link_name = "wrist_2_link"
    ocm.header.frame_id = "base_link"
    ocm.orientation.y = 1.0  # Keep the orientation to point down
    ocm.absolute_x_axis_tolerance = 0.3
    ocm.absolute_y_axis_tolerance = 0.3
    ocm.absolute_z_axis_tolerance = 0.3
    ocm.weight = 1.0

    # Set the constraint in MoveIt
    constraints = Constraints()
    constraints.orientation_constraints.append(ocm)
    group.set_path_constraints(constraints)

    # Plan the trajectory
    rospy.loginfo("Planning to target pose with orientation constraint...")
    success, plan, planning_time, error_code = group.plan()
    rospy.loginfo(f"Planning result: success={success}, planning_time={planning_time}, error_code={error_code}")

    # Clear the path constraints after planning
    group.clear_path_constraints()

    if success and plan.joint_trajectory.points:
        joint_positions = plan.joint_trajectory.points[-1].positions
        rospy.loginfo(f"Computed joint positions: {joint_positions}")
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
                target_position = [real_world_x, real_world_y, real_world_z + 0.14]  # Adjust to 10cm above the detected position
                position_logged = True  # Set the flag to True after logging the position

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
        rospy.sleep(1)
        while target_position is None:
            rospy.sleep(0.5)

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
                duration=2.0
            )

            # Adding a delay before testing the gripper
            time.sleep(2)
        

            # Close the gripper to pick up the cube
            send_gripper_command(
                controller_name='gripper_controller',
                joint_names=['gripper_finger1_joint'],
                positions=[0.4],  # Assuming the gripper is closing
                duration=1.0
            )

            # Adding a delay before lifting the arm
            time.sleep(1)
                       
            # Update target position to move up by 0.3m
            target_position[2] += 0.3

            # Compute joint positions for the new target position (lift up)
            joint_positions_up = compute_inverse_kinematics(target_position)

            if joint_positions_up:
                # Move the robot arm up by 0.3m
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
                    positions=joint_positions_up,
                    duration=2.0
                )
            # Testing Gripper controller using action client
            time.sleep(1)
            send_gripper_command(
                controller_name='gripper_controller',
                joint_names=['gripper_finger1_joint'],
                positions=[0],  # Assuming the gripper is opening
                duration=1.0
            )

    except rospy.ROSInterruptException:
        rospy.logerr('Interrupted during execution')

if __name__ == '__main__':
    main()
