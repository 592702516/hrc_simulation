import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2

bridge = CvBridge()
depth_data = None

# Callback function to get depth data
def depth_callback(msg):
    global depth_data
    depth_data = msg

# Function to convert pixel coordinates to real-world coordinates
def pixel_to_world(x, y):
    global depth_data
    if depth_data is None:
        rospy.logwarn("Depth data is not available yet.")
        return None, None, None

    # Extract depth value from the PointCloud2 data
    for point in pc2.read_points(depth_data, field_names=("x", "y", "z"), skip_nans=True):
        if int(point[0]) == int(x) and int(point[1]) == int(y):
            real_world_x, real_world_y, real_world_z = point
            return real_world_x, real_world_y, real_world_z

    rospy.logwarn("Depth value for the given pixel coordinates not found.")
    return None, None, None

def image_callback(msg):
    try:
        # Convert ROS Image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return

    # Convert the image to HSV color space for red detection
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 120, 70])
    upper_red = np.array([10, 255, 255])
    mask = cv2.inRange(hsv_image, lower_red, upper_red)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        # Get the largest contour to detect the red cube
        c = max(contours, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(c)

        if radius > 10:  # Threshold to ensure the detected object is significant
            rospy.loginfo(f"Detected red cube at pixel coordinates: ({x}, {y}) with radius: {radius}")
            
            # Convert pixel coordinates to real-world coordinates
            real_world_x, real_world_y, real_world_z = pixel_to_world(x, y)
            if real_world_x is not None:
                rospy.loginfo(f"Red cube real-world coordinates: ({real_world_x}, {real_world_y}, {real_world_z})")

def main():
    rospy.init_node('red_cube_position_node')

    # Subscribe to the correct camera topic to detect the red cube
    rospy.Subscriber('/azure_kinect_camera_1/color/image_raw', Image, image_callback)
    rospy.Subscriber('/azure_kinect_camera_1/depth/points', PointCloud2, depth_callback)

    rospy.spin()

if __name__ == '__main__':
    main()

