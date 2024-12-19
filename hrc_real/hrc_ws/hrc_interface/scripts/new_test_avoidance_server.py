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

import rospy
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import time
import numpy as np

# Initialize global variables
previous_velocities = []
previous_time = None
acceleration_data = []  # Store acceleration data for plotting
jerk_data = []          # Store jerk data for plotting
time_data = []          # Store time data for plotting

x_positions = []
y_positions = []
z_positions = []

start_record =False

accel_cost = 0


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

ur_kin = ur_kinematics.URKinematics('ur5e')

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

def callback(msg):
    global previous_velocities, previous_time, acceleration_data, jerk_data, time_data, start_record, accel_cost, end_eff
    if start_record:
        current_time = time.time()
        if previous_time is not None:
            # Calculate acceleration for each joint
            accelerations = []
            for i in range(len(msg.velocity)):
                # Calculate acceleration
                acceleration = (msg.velocity[i] - previous_velocities[i]) / ((1/500))
                accel_cost = accel_cost + (msg.velocity[i] *msg.velocity[i] )
                accelerations.append(acceleration)

            # Store the current time and accelerations
            time_data.append(current_time)
            acceleration_data.append(accelerations)

            # Calculate jerk for each joint
            if len(acceleration_data) > 1:  # Ensure we have at least two data points
                jerks = []
                for i in range(len(accelerations)):
                    # Calculate jerk
                    # jerk = (accelerations[i] - acceleration_data[-2][i]) / (current_time - previous_time)
                    jerk = msg.velocity[i]
                    jerks.append(jerk)
                jerk_data.append(jerks)

            joint_positions = [msg.position[2], msg.position[1], msg.position[0], msg.position[3], msg.position[4], msg.position[5]]

            # Ensure you have 6 joint positions for the UR5e
            if len(joint_positions) == 6:
                # Compute forward kinematics to get the end-effector position
                T = ur_kin.forward(joint_positions)

                # Check if T is a 1D array (e.g., if it contains just the position)
                if T.ndim == 1 and len(T) >= 3:
                    position = T[:3]  # First 3 elements as position (x, y, z)
                    position[0] = position[0] * -1
                    position[1] = position[1] * -1
                    x_positions.append(position[0])
                    y_positions.append(position[1])
                    z_positions.append(position[2])

                    print(f"End-effector position: {position}")
                else:
                    print(f"Unexpected result shape: {T.shape}")



        # Update previous velocities and time
        previous_velocities = msg.velocity
        previous_time = current_time


def plot_accelerations():
    # Convert acceleration data to a numpy array for easier handling
    acceleration_array = np.array(acceleration_data)
    jerk_array = np.array(jerk_data)

    # Check if acceleration data is available
    if acceleration_array.size == 0:
        print("No acceleration data to plot. Waiting for data...")
    else:
        print("Shape of acceleration_array:", acceleration_array.shape)
        print("Length of time_data:", len(time_data))

        # If acceleration_array is 1D, reshape it to 2D (with one joint)
        if acceleration_array.ndim == 1:
            acceleration_array = acceleration_array.reshape(-1, 1)

        print("Length of acceleration_array:", acceleration_array.shape[0])

        # Plot accelerations
        plt.figure()
        for i in range(acceleration_array.shape[1]):
            if len(time_data) == acceleration_array.shape[0]:
                plt.plot(time_data, acceleration_array[:, i], label=f'Joint {i}')
            else:
                print(f"Skipping Joint {i} due to mismatched lengths.")

        plt.title('Joint Accelerations Over Time With a_sat')
        plt.xlabel('Time (s)')
        plt.ylabel('Acceleration (rad/s²)')
        plt.legend()
        plt.grid()
        plt.show()

    # Check if jerk data is available
    if jerk_array.size == 0:
        print("No jerk data to plot. Waiting for data...")
    else:
        # Plot jerks
        plt.figure()
        for i in range(jerk_array.shape[1]):
            if len(time_data) > 1 and jerk_array.shape[0] == len(time_data) - 1:
                plt.plot(time_data[1:], jerk_array[:, i], label=f'Joint {i} Jerk')  # Time shifted by 1 for jerk
            else:
                print(f"Skipping Jerk plot for Joint {i} due to mismatched lengths.")

        plt.title('Joint Velocities Over Time With a_sat')
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity (rad/s)')
        plt.legend()
        plt.grid()
        plt.show()


        # Plot end-effector 3d space
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plot the X, Y, Z positions
        ax.plot(x_positions, y_positions, z_positions, label='End-Effector Path')

        # Highlight start and end points
        ax.scatter(x_positions[0], y_positions[0], z_positions[0], color='green', s=100, label='Start Position')  # Start point
        ax.scatter(x_positions[-1], y_positions[-1], z_positions[-1], color='red', s=100, label='End Position')  # End point

        # Set axis limits
        ax.set_xlim(0.2, 0.8)     # X-axis range
        ax.set_ylim(-0.61, 0.43)  # Y-axis range
        ax.set_zlim(0.132, 0.6)   # Z-axis range

        # Set plot labels and title
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_zlabel('Z Position (m)')
        ax.set_title('End-Effector Path in 3D Space')

        # Show the legend and plot
        ax.legend()
        plt.show()

    

rospy.init_node("avoidance_server_test", anonymous=True)

pub_rp = rospy.Publisher('required_pos', Float32MultiArray, queue_size=1)
sub_ag = rospy.Subscriber("active_avoidance_goal", Int8MultiArray, callback_ag)
pub_bt = rospy.Publisher('/body_tracking_data', MarkerArray, queue_size=10)
rospy.Subscriber('/joint_states', JointState, callback)


try:
   
    real_start_time = time.time()
    # Give some time for the publisher to connect   
    rospy.sleep(1)


    pos = Float32MultiArray()


    pos.data = [0.297, -0.132, 0.250, 2.231, -2.216, 0.0]

    pub_rp.publish(pos)


    pos.data = [0.43,-0.48,0.26,0,0,0]


    pub_rp.publish(pos)

    rospy.sleep(0.8)
    goal_not_active_count = 0
    while goal_not_active_count<10:
        if(not goal_active):
            goal_not_active_count = goal_not_active_count + 1
        else:
            goal_not_active_count = 0
        # print("waiting")
    print(goal_active)

    # play_bag(bag_file, loop=False, rate=1.0, start_time=8)

    start_record = True
    rospy.sleep(1)


    pos.data = [0.68,0.42,0.22,0,0,0]


    pub_rp.publish(pos)


    rospy.sleep(0.8)
    goal_not_active_count = 0
    while goal_not_active_count<10:
        if(not goal_active):
            goal_not_active_count = goal_not_active_count + 1
        else:
            goal_not_active_count = 0
        # print("waiting")
    print(goal_active)

    rospy.spin()
    plot_accelerations()
    # print(accel_cost)
except:
    plot_accelerations()
    print(accel_cost)
