import sys
from turtle import pos
import rospy
import time
import numpy as np

import sys
import time
import rospy
import actionlib
import math as m
import csv
import os
import time



from sensor_msgs.msg import JointState

from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
import geometry_msgs.msg as geometry_msgs
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryFeedback, JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from scipy.spatial.transform import Rotation
from ur_ikfast import ur_kinematics
from std_msgs.msg import Float64MultiArray, Float32MultiArray, Float32, Bool, Int8MultiArray
from pathlib import Path

import rosbag
from visualization_msgs.msg import Marker, MarkerArray

import subprocess

def play_bag(bag_file_path, loop=False, rate=1.0, start_time=0, duration=None):
    # Build the rosbag play command
    command = ['rosbag', 'play', bag_file_path, '--rate', str(rate), '--start', str(start_time)]
    
    # Add loop option if specified
    if loop:
        command.append('--loop')
    
    # Add duration option if specified
    if duration:
        command.extend(['--duration', str(duration)])
    
    # Start the rosbag play process
    process = subprocess.Popen(command)
    
    # Wait for the process to complete
    process.wait()



goal_active = False

def callback_ag(data):
    global goal_active
    goal_active = bool(data.data[0])
    print(goal_active)

    

rospy.init_node("avoidance_server_test", anonymous=True)

pub_rp = rospy.Publisher('required_pos', Float32MultiArray, queue_size=1)
sub_ag = rospy.Subscriber("active_avoidance_goal", Int8MultiArray, callback_ag)
pub_bt = rospy.Publisher('/body_tracking_data', MarkerArray, queue_size=10)


real_start_time = time.time()
# Give some time for the publisher to connect   
rospy.sleep(1)


pos = Float32MultiArray()

goal_not_active_count = 0
while goal_not_active_count<10:
    if(not goal_active):
        goal_not_active_count = goal_not_active_count + 1
    else:
        goal_not_active_count = 0
    print("waiting")
print(goal_active)

pos.data = [0.297, -0.132, 0.250, 2.231, -2.216, 0.0]

pub_rp.publish(pos)

rospy.sleep(0.8)
goal_not_active_count = 0
while goal_not_active_count<10:
    if(not goal_active):
        goal_not_active_count = goal_not_active_count + 1
    else:
        goal_not_active_count = 0
    print("waiting")
print(goal_active)


pos.data = [0.43,-0.48,0.26,0,0,0]


pub_rp.publish(pos)




rospy.sleep(0.8)
goal_not_active_count = 0
while goal_not_active_count<10:
    if(not goal_active):
        goal_not_active_count = goal_not_active_count + 1
    else:
        goal_not_active_count = 0
    print("waiting")
print(goal_active)

 


pos.data = [0.68,0.42,0.22,0,0,0]


pub_rp.publish(pos)



rospy.sleep(0.8)
goal_not_active_count = 0
while goal_not_active_count<10:
    if(not goal_active):
        goal_not_active_count = goal_not_active_count + 1
    else:
        goal_not_active_count = 0
    print("waiting")
print(goal_active)



