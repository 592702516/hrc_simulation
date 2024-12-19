import rosbag
import rospy
from pathlib import Path
import csv
import numpy as np
# Initialize the ROS node (optional, depending on your processing)
rospy.init_node('rosbag_processor', anonymous=True)

# Defines the path of the YOLOv8 model
bag_file = Path(__file__).parent / "displacement_data" / "test_dp7_ws.bag"


first = True

old_x= 0
old_y= 0
old_z= 0

displacement_sum = 0

start_time = 0


with open('assembly_displacement_dp_ws_7.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    
    writer.writerow(['T', 'X', 'Y', 'Z', 'absolute_displacement', 'Current_Sum'])
    with rosbag.Bag(bag_file, 'r') as bag:
        # print(bag.read_messages())
        # Iterate through messages in the bag
        for topic, msg, t in bag.read_messages():
            # Process the message based on the topic
            # print(topic)
            if topic == '/body_tracking_data':
                
                # Example: Print the message
                for marker in msg.markers:
                    if(marker.id%100 == 15):


                        if (not first):
                            absolute_displacement = np.sqrt((marker.pose.position.x - old_x)**2 + (marker.pose.position.y - old_y)**2 + (marker.pose.position.z - old_z)**2)
                            writer.writerow([(t.to_sec() - start_time), marker.pose.position.x, marker.pose.position.y, marker.pose.position.z, absolute_displacement, displacement_sum])
                            displacement_sum  += absolute_displacement

                            print(f"x = {marker.pose.position.x}, y = {marker.pose.position.y}, z = {marker.pose.position.z}")
                            old_x = marker.pose.position.x
                            old_y = marker.pose.position.y
                            old_z = marker.pose.position.z

                        else:
                            old_x = marker.pose.position.x
                            old_y = marker.pose.position.y
                            old_z = marker.pose.position.z
                            start_time = t.to_sec()
                            first = False



    print(displacement_sum)
    writer.writerow(['NULL', 'NULL', 'NULL','NULL', 'NULL', displacement_sum])
        # Add more conditions for other topics as needed
