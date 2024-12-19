import rospy
import numpy as np
from pathlib import Path
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
import time

class YOLOv8ROS:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('yolov8_ros_node', anonymous=True)

        # Create a CvBridge object
        self.bridge = CvBridge()

        # Defines the path of the YOLOv8 model
        model_path = Path(__file__).parent / "libs" / "models" / "best_final2.pt"
        print(model_path)

        # Load your trained YOLOv8 model
        self.model = YOLO(model_path)

        # Subscribe to the RGB image topic
        self.image_sub = rospy.Subscriber('/rgb/image_raw', Image, self.image_callback)

        # self.mask_sub = rospy.Subscriber('/body_index_map/image_raw', Image, self.mask_callback)

        self.mask = None

    def image_callback(self, data):
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(f'CvBridge Error: {e}')
            return
        
        # Define the target resolution (1920x1080 for 1080p)
        width = 1920
        height = 1080
        dim = (width, height)

        # Resize the image to 1080p
        resized_image = cv2.resize(cv_image, dim, interpolation=cv2.INTER_AREA)
        

        # Step 2: Adjust the contrast
        # alpha > 1.0 increases contrast, beta can adjust brightness (keep it 0 to only adjust contrast)
        alpha = 1.5  # Contrast control (1.0-3.0)
        beta = 0     # Brightness control (0-100)

        # Apply the contrast and brightness adjustment
        adjusted_image = cv2.convertScaleAbs(resized_image, alpha=alpha, beta=beta)


        # Run YOLOv8 inference
        results = self.model(resized_image)

        print(results)

        # Visualize the results
        annotated_image = results[0].plot()

        # Display the image with cv2.imshow
        cv2.imshow('YOLOv8 Detections', annotated_image)
        cv2.waitKey(1)  # Display the image for 1 ms (adjust as needed)


if __name__ == '__main__':

    try:
        yolov8_ros = YOLOv8ROS()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
