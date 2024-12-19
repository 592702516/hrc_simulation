import sys
import time
import rospy
import numpy as np
import signal

from geometry_msgs.msg import WrenchStamped

from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose

from libs.robot import UR5eRobot
from libs.gripper import Robotiq85Gripper



# def callback(data):
#     if intialised  == True:
#         print(f"Forces are: {data.wrench.force.x}, {data.wrench.force.y}, {data.wrench.force.z}")
#         print(f"Moments are: {data.wrench.torque.x}, {data.wrench.torque.y}, {data.wrench.torque.z}")
#         while not (data.wrench.force_y < -110 or data.wrench.torque_x > -0.5):

#             continue
#         open_gripper(gripper)


def open_gripper(gripper):
    success = gripper.open()
    if success:
        rospy.loginfo('Successfully opened')
        time.sleep(2)
    else:
        rospy.loginfo('Open gripper failed')
        raise Exception("Cannot open gripper")
    time.sleep(1)

def close_gripper(gripper):
    success = gripper.close()
    if success:
        rospy.loginfo('Successfully closed')
        time.sleep(1)
    else:
        rospy.loginfo('Close gripper failed')
        raise Exception("Cannot close gripper")
    time.sleep(1)

if __name__ == "__main__":

    intialised = False
    rospy.init_node("combined_control")
    robot = UR5eRobot()
    gripper = Robotiq85Gripper()

    # ft_sub = rospy.Subscriber('/robotiq_ft_wrench', WrenchStamped, callback)

    rospy.sleep(1)
    open_gripper(gripper)

    rospy.sleep(1)
    close_gripper(gripper)

    intialised = True


    # rospy.spin()

    

    # open_gripper(gripper)
    # rospy.sleep(0.5)

    # rospy.sleep(1)
    # close_gripper(gripper)
    # rospy.sleep(0.5)