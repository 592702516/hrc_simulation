#!/usr/bin/env python
# license removed for brevity
from turtle import position
import rospy
from std_msgs.msg import Header
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from math import *

def get_goal():
    goal = JointTrajectory()
    goal.header = Header()
    goal.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'workbench_joint']
    goal.header.stamp = rospy.Time.now()
    point = JointTrajectoryPoint()
    point.positions = [0.50, -0.55, -2.6, -1.57, 1.58, 0.0, 0.4]
    point.velocities = [0.1, 0, 0, 0, 0, 0, 0]
    point.time_from_start = rospy.Duration(2.0)
    goal.points.append(point)
    return goal


def talker():
    pub = rospy.Publisher('ur5e_controller/command', JointTrajectory, queue_size=10)
    rospy.init_node('send_joints', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        goal = get_goal()
        rospy.loginfo(goal)
        pub.publish(goal)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
