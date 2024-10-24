import rospy
import sys
import actionlib
import moveit_commander
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import Float64
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

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

def get_end_effector_position():
    group = moveit_commander.MoveGroupCommander("ur5e")
    current_pose = group.get_current_pose().pose
    rospy.loginfo(f"End-Effector Position: x={current_pose.position.x}, y={current_pose.position.y}, z={current_pose.position.z}")
    rospy.loginfo(f"End-Effector Orientation: x={current_pose.orientation.x}, y={current_pose.orientation.y}, z={current_pose.orientation.z}, w={current_pose.orientation.w}")
    print(f"End-Effector Position: x={current_pose.position.x}, y={current_pose.position.y}, z={current_pose.position.z}")
    print(f"End-Effector Orientation: x={current_pose.orientation.x}, y={current_pose.orientation.y}, z={current_pose.orientation.z}, w={current_pose.orientation.w}")

def main():
    rospy.init_node('test_ur5e_gripper_controllers')

    # Initialize MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("ur5e")

    try:
        # Set the joint positions directly
        group.set_joint_value_target([0.4, 0.0, -1.6, -1.5, 4.7, 1.5708, 0.0])
        plan = group.plan()
        group.go(wait=True)

        # Print the end-effector position after moving to the target joint positions
        get_end_effector_position()

        # Adding a delay before testing the gripper
        time.sleep(2)

        # Testing Gripper controller using action client
        send_gripper_command(
            controller_name='gripper_controller',
            joint_names=['gripper_finger1_joint'],
            positions=[0.5],  # Assuming the gripper is closing to 3.1415(pi) position
            duration=3.0
        )
        time.sleep(2)
        # Testing Gripper controller using action client
        send_gripper_command(
            controller_name='gripper_controller',
            joint_names=['gripper_finger1_joint'],
            positions=[0],  # Assuming the gripper is closing to 3.1415(pi) position
            duration=3.0
        )

    except rospy.ROSInterruptException:
        rospy.logerr('Interrupted during execution')

if __name__ == '__main__':
    main()
