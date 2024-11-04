#!/usr/bin/env python
# auto photo when detect the red
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os
import time

# Set up a folder for saving pictures
save_folder = "/home/lisms/picture"
if not os.path.exists(save_folder):
    os.makedirs(save_folder)

bridge = CvBridge()

# Function to detect red color and save the image
def detect_red_and_save_image(image_msg):
    global last_saved_time
    # Convert the ROS Image message to OpenCV format
    cv_image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    
    # Convert the image to HSV color space for better color detection
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    
    # Define the red color range in HSV
    lower_red = (0, 120, 70)
    upper_red = (10, 255, 255)
    
    # Create a mask for red color
    mask1 = cv2.inRange(hsv_image, lower_red, upper_red)
    
    # Second range for detecting red
    lower_red2 = (170, 120, 70)
    upper_red2 = (180, 255, 255)
    mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
    
    # Combine both masks
    red_mask = mask1 + mask2
    
    # Check if red is detected
    red_detected = cv2.countNonZero(red_mask) > 0
    
    # Save image every 1 second if red is detected
    current_time = time.time()
    if red_detected and current_time - last_saved_time >= 1:
        image_filename = os.path.join(save_folder, f"red_detected_{int(current_time)}.jpg")
        cv2.imwrite(image_filename, cv_image)
        rospy.loginfo(f"Red detected. Image saved as {image_filename}")
        last_saved_time = current_time

# Initialize the ROS node
if __name__ == '__main__':
    rospy.init_node('red_color_detector', anonymous=True)
    
    last_saved_time = 0
    
    # Subscribe to the image topic
    image_topic = "/azure_kinect_camera_1/color/image_raw"
    rospy.Subscriber(image_topic, Image, detect_red_and_save_image)
    
    # Spin to keep the node running
    rospy.spin()

