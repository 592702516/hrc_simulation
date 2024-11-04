# test the depth data 
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

bridge = CvBridge()

# Callback function to receive depth image data
def depth_callback(msg):
    try:
        # Convert ROS Image message to OpenCV format
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
        # Check if depth data is available
        if depth_image is not None:
            rospy.loginfo("Depth data received successfully.")
            # Optionally, print some depth values to verify
            depth_center_value = depth_image[depth_image.shape[0] // 2, depth_image.shape[1] // 2]
            rospy.loginfo(f"Depth value at the center: {depth_center_value} meters")
        else:
            rospy.logwarn("Depth image is empty.")
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error: {e}")

# Main function to initialize the ROS node and subscribe to the depth topic
def main():
    rospy.init_node('depth_camera_test_node', anonymous=True)

    # Subscribe to the depth image topic
    rospy.Subscriber('/azure_kinect_camera_1/depth/image_raw', Image, depth_callback)

    rospy.loginfo("Subscribed to depth image topic: /azure_kinect_camera_1/depth/image_raw")

    # Spin to keep the script running and receiving messages
    rospy.spin()

if __name__ == '__main__':
    main()