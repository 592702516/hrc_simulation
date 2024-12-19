#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import numpy as np
import time

# Initialize global variables
previous_velocities = []
previous_time = None
acceleration_data = []  # Store acceleration data for plotting
time_data = []          # Store time data for plotting


def callback(msg):
    global previous_velocities, previous_time, acceleration_data, time_data

    current_time = time.time()
    # print(current_time)
    if previous_time is not None:
        # Calculate acceleration for each joint
        accelerations = []
        for i in range(len(msg.position)):
            # Calculate acceleration
            # acceleration = (msg.position[i] - previous_velocities[i]) / (current_time - previous_time)
            acceleration = msg.velocity[i]
            accelerations.append(acceleration)

        # Store the current time and accelerations
        time_data.append(current_time)
        acceleration_data.append(accelerations)

    # Update previous velocities and time
    previous_velocities = msg.position
    previous_time = current_time

def plot_accelerations():
    # Convert acceleration data to a numpy array for easier handling
    import numpy as np
    acceleration_array = np.array(acceleration_data)

    plt.figure()
    for i in range(acceleration_array.shape[1]):
        plt.plot(time_data, acceleration_array[:, i], label=f'Joint {i}')

    plt.title('Joint Accelerations Over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (rad/s² or N/s²)')
    plt.legend()
    plt.grid()
    plt.show()

def main():
    global previous_time

    rospy.init_node('joint_acceleration_calculator', anonymous=True)
    rospy.Subscriber('/joint_states', JointState, callback)

    # Keep the node alive
    rospy.spin()

    # Call plot function on exit
    plot_accelerations()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        # Call plot function on keyboard interrupt
        plot_accelerations()
