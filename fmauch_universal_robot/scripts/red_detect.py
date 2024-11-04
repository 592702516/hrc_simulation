# detect the red cube
import rospy
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
depth_image = None

def depth_callback(msg):
    global depth_image
    try:
        # Convert ROS Image message to OpenCV format for depth data
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    except CvBridgeError as e:
        rospy.logerr("CvBridge 错误: {0}".format(e))

def pixel_to_world(x, y):
    global depth_image
    if depth_image is None:
        rospy.logwarn("深度数据尚未可用。")
        return None, None, None

    # Ensure coordinates are within bounds
    if int(y) >= depth_image.shape[0] or int(x) >= depth_image.shape[1]:
        rospy.logwarn("坐标超出范围。")
        return None, None, None

    # Get depth value at (x, y)
    depth_value = depth_image[int(y), int(x)]
    if np.isnan(depth_value) or depth_value <= 0:
        rospy.logwarn("给定像素坐标的深度值无效。")
        return None, None, None

    # Example conversion, assuming simple pinhole camera model
    fx, fy = 525.0, 525.0  # Focal length in pixels (example values)
    cx, cy = 320.0, 240.0  # Principal point (example values)

    camera_x = (x - cx) * depth_value / fx
    camera_y = (y - cy) * depth_value / fy
    camera_z = depth_value

    # Transform the camera coordinates to world coordinates based on the fixed camera position
    world_x = - camera_y + 0.5  # Camera position in the world frame (x = 0.5 m)
    world_y = - camera_x  
    world_z = 2.0684 - camera_z   # Camera height in the world frame (z = 2.0 m)

    return world_x, world_y, world_z

    # Ensure coordinates are within bounds
    if int(y) >= depth_image.shape[0] or int(x) >= depth_image.shape[1]:
        rospy.logwarn("坐标超出范围。")
        return None, None, None

    # Get depth value at (x, y)
    depth_value = depth_image[int(y), int(x)]
    if np.isnan(depth_value) or depth_value <= 0:
        rospy.logwarn("给定像素坐标的深度值无效。")
        return None, None, None

    # Example conversion, assuming simple pinhole camera model
    fx, fy = 525.0, 525.0  # Focal length in pixels (example values)
    cx, cy = 320.0, 240.0  # Principal point (example values)

    camera_x = (x - cx) * depth_value / fx
    camera_y = (y - cy) * depth_value / fy
    camera_z = depth_value

    # Create a PointStamped message for the camera frame
    camera_point = PointStamped()
    camera_point.header.frame_id = "world_camera"
    camera_point.point.x = camera_x
    camera_point.point.y = camera_y
    camera_point.point.z = camera_z

    # Transform the point to the world frame
    world_point = transform_camera_to_world(camera_point)
    if world_point is not None:
        return world_point.point.x, world_point.point.y, world_point.point.z
    else:
        return None, None, None

def transform_camera_to_world(camera_point):
    try:
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)
        transform = tf_buffer.lookup_transform('world', 'world_camera', rospy.Time(0), rospy.Duration(1.0))
        world_point = tf2_geometry_msgs.do_transform_point(camera_point, transform)
        return world_point
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr(f"Transform error: {e}")
        return None

def image_callback(msg):
    try:
        # Convert ROS Image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except CvBridgeError as e:
        rospy.logerr("CvBridge 错误: {0}".format(e))
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
            rospy.loginfo(f"在像素坐标 ({x}, {y}) 处检测到红色立方体，半径为: {radius}")

            # Convert pixel coordinates to world coordinates
            real_world_x, real_world_y, real_world_z = pixel_to_world(x, y)
            if real_world_x is not None:
                rospy.loginfo(f"红色立方体的实际世界坐标: ({real_world_x}, {real_world_y}, {real_world_z})")

def main():
    rospy.init_node('red_cube_position_node')

    # Subscribe to the RGB and depth image topics
    rospy.Subscriber('/azure_kinect_camera_1/color/image_raw', Image, image_callback)
    rospy.Subscriber('/azure_kinect_camera_1/depth/image_raw', Image, depth_callback)   

    rospy.spin()

if __name__ == '__main__':
    main()
