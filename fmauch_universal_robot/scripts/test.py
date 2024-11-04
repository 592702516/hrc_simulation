# send the trajectory to the joints
import rospy
import actionlib
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


def main():
    rospy.init_node('test_ur5e_gripper_controllers')

    try:
        # Testing UR5e arm controller
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
            positions=[0.4, 0.0, -1.6, -1.5, 4.67, 1.5708, 0.0],
            duration=5.0
        )

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
