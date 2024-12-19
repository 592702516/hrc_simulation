#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from ultralytics import YOLO
from pathlib import Path
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class VisionProcessing:
    def __init__(self):
        model_path = Path(__file__).parent / "libs" / "models" / "good.pt"
        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        self.latest_rgb = None

    def rgb_callback(self, msg):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error processing RGB image: {e}")

    def detect_tool_position(self, tool_class_id):
        if self.latest_rgb is None:
            return None, None
        results = self.model(self.latest_rgb)
        for box in results[0].boxes:
            if int(box.cls[0]) == tool_class_id:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                X, Y = self.pixel_to_3d(center_x, center_y)
                tf_x, tf_y = self.transform_coords(X, Y)
                return tf_x, tf_y
        return None, None

    def detect_process(self):
        if self.latest_rgb is None:
            return None
        results = self.model(self.latest_rgb)
        for box in results[0].boxes:
            cls = int(box.cls[0])
            if cls in [3, 4, 5, 6]:  # Process A, B, C, D
                return cls
        return None

    def detect_hand_gesture(self):
        if self.latest_rgb is None:
            return None

        results = self.model(self.latest_rgb)
        for box in results[0].boxes:
            class_id = int(box.cls[0])
            if class_id == 0:  # Fist
                return "fist"
            elif class_id == 2:  # Palm
                return "palm"
        return None

    # Transformation methods if needed for vision processing:
    def pixel_to_3d(self, u, v):
        depth = 1.223  
        pixels = np.array([[u, v]], dtype=np.float32)
        K = [913.650390625, 0.0, 955.0496215820312, 0.0, 913.8749389648438, 550.6069946289062, 0.0, 0.0, 1.0]
        D = [0.18453791737556458, -2.423478603363037, 0.00043306572479195893, -0.0002455342037137598, 1.5398979187011719, 0.0690656453371048, -2.238345146179199, 1.4565629959106445]
        camera_matrix = np.array(K).reshape((3, 3))
        dist_coeffs = np.array(D)
        undistorted_points = cv2.undistortPoints(pixels, camera_matrix, dist_coeffs)
        X = depth * undistorted_points[0][0][0] - 0.02
        Y = depth * undistorted_points[0][0][1]
        return X, Y

    def transform_coords(self, x, y):
        tf_x = -y + (0.766 - 0.09) 
        tf_y = -x + (1.036 - 0.812)    
        sf = 0.07 if tf_y < 0 else 0
        tf_y -= (sf * tf_y)
        return tf_x, tf_y