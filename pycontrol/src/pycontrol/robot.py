import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTrajectoryControllerState
import time
import numpy as np

class UR5eRobot:
    def __init__(self):
        rospy.Subscriber("/ur5e_controller/state", JointTrajectoryControllerState, self._update_robot_stat, queue_size=10)
        self._robot_pub = rospy.Publisher('/ur5e_controller/command', JointTrajectory, queue_size=10)

        self._robot_stat = JointTrajectoryControllerState()
        self._r = rospy.Rate(1)

    def _update_robot_stat(self, stat):
        self._robot_stat = stat

    def execute_joint_trajectory(self, positions):
        joint_names=[
                'workbench_joint',
                'shoulder_pan_joint',
                'shoulder_lift_joint',
                'elbow_joint',
                'wrist_1_joint',
                'wrist_2_joint',
                'wrist_3_joint'
            ]
        duration=5.0
        client = actionlib.SimpleActionClient(f'/ur5e_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for ur5e_controller action server...')
        client.wait_for_server()
        rospy.loginfo('ur5e_controller action server is up. Sending trajectory...')

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(duration)

        goal.trajectory.points.append(point)
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)

        client.send_goal(goal)
        client.wait_for_result()


