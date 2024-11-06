import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# import sys
# import time
from yolo_msgs.msg import yolo

class AzureKinectCamera:
    def __init__(self):
        self.bridge = CvBridge()
        self._latest_image = None
        self._latest_depth = None
        rospy.Subscriber("/azure_kinect_camera_1/color/image_raw", Image, self._update_image, queue_size=10)
        rospy.Subscriber("/azure_kinect_camera_1/depth/image_raw", Image, self._update_depth, queue_size=10)
    #     self.listener = rospy.Subscriber("/yolov5", yolo, self._update_object_stat, queue_size=10)
    #     self._detect = yolo()

    # def _update_object_stat(self, stat):
    #     self._detect = stat
    
    # def get_detect(self):
    #     return self._detect
    

    def _update_image(self, data):
        try:
            # Convert the ROS Image message to an OpenCV image
            self._latest_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
    def _update_depth(self, data):
        try:
            self._latest_depth = self.bridge.imgmsg_to_cv2(data, "16UC1")  # Depth images often use 16-bit format
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
    
    def get_image(self):
        return self._latest_image
    def get_depth(self):
        return self._latest_depth








