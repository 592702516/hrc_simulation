#!/usr/bin/env python
import rospy

from control_msgs.msg import FollowJointTrajectoryActionFeedback


def callback(data):
    rospy.loginfo(data.feedback.actual.pose)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/ur5e_controller/follow_joint_trajectory/feedback", FollowJointTrajectoryActionFeedback
, callback)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()