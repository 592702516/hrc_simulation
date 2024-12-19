import rospy
import numpy as np
from pathlib import Path
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
from std_msgs.msg import Bool, Int8MultiArray

def transform_coords(x, y, z):
    tf_x = -y + (0.566 - 0.09) 
    tf_y = -x + (1.036 - 0.812)
    tf_z = -z + 1.223 
    return tf_x, tf_y, tf_z

class HandTracking:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('hand_tracking', anonymous=True)

        # Subscribe to body-tracking topic
        self.joints = rospy.Subscriber('/body_tracking_data', MarkerArray, self.joint_callback)

        # Subscribe to the camera image for YOLO detection
        self.image_sub = rospy.Subscriber('/rgb/image_raw', Image, self.image_callback)

        # YOLO model for palm detection
        model_path = Path(__file__).parent / "libs" / "models" / "best_final2.pt"
        self.model = YOLO(model_path)

        # Right hand position (transformed)
        self.tf_rhx, self.tf_rhy, self.tf_rhz = None, None, None

        # Flag for detecting hand in the robot's workspace
        self.hand_in_boundary = False

        # Latest image for YOLO detection
        self.latest_image = None
        self.bridge = CvBridge()

    def joint_callback(self, data):
        """
        Callback for body tracking data to extract and transform right-hand position.
        """
        try:
            if data.markers:
                for marker in data.markers:
                    joint_id = marker.id % 100

                    # Store right-hand joint coordinates (ID 15 for left hand)
                    if joint_id == 15:  
                        raw_x, raw_y, raw_z = (
                            marker.pose.position.x,
                            marker.pose.position.y,
                            marker.pose.position.z,
                        )
                        # Transform the coordinates
                        self.tf_rhx, self.tf_rhy, self.tf_rhz = transform_coords(raw_x, raw_y, raw_z)

                if self.tf_rhx is not None and self.tf_rhy is not None and self.tf_rhz is not None:
                    # Print transformed right-hand position
                    print(f"Transformed right hand position: X: {self.tf_rhx:.2f}, Y: {self.tf_rhy:.2f}, Z: {self.tf_rhz:.2f}")

                    # Check if right hand is within robot workspace boundaries
                    self.hand_in_boundary = (
                        0.24 < self.tf_rhx < 0.88 and
                        -0.48 < self.tf_rhy < 0.6 and
                        0.15 < self.tf_rhz < 0.6
                    )
        except Exception as e:
            rospy.logerr(f"Error in joint_callback: {e}")

    def image_callback(self, msg):
        """
        Callback for the image topic to process the frame using YOLO.
        """
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.detect_palm()
        except Exception as e:
            rospy.logerr(f"Error in image_callback: {e}")

    def detect_palm(self):
        """
        Use YOLO to detect the palm and extract bounding box information.
        """
        if self.latest_image is None:
            return

        results = self.model(self.latest_image)

        # Extract predictions
        predictions = results[0].boxes
        class_ids = predictions.cls.int().tolist()
        confidences = predictions.conf.tolist()
        bound_boxes = predictions.xywh.tolist()

        # Filter detections by confidence threshold
        detections = [
            (class_id, confidence, bbox)
            for class_id, confidence, bbox in zip(class_ids, confidences, bound_boxes)
            if confidence >= 0.83
        ]

        if detections:
            for class_id, confidence, bbox in detections:
                x, y, w, h = bbox
                print(f"Detected palm: Class ID: {class_id}, Confidence: {confidence:.2f}, Bounding Box: {bbox}")
                cv2.rectangle(self.latest_image, (int(x - w / 2), int(y - h / 2)), (int(x + w / 2), int(y + h / 2)), (0, 255, 0), 2)
                cv2.putText(self.latest_image, f"Palm {confidence:.2f}", (int(x), int(y - h / 2 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Show the annotated image
        cv2.imshow("YOLO Palm Detection", self.latest_image)
        cv2.waitKey(1)

    def run(self):
        """
        Main loop for monitoring right-hand position and palm detection.
        """
        rospy.loginfo("Hand tracking node running...")
        rospy.spin()


if __name__ == "__main__":
    try:
        tracker = HandTracking()
        tracker.run()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
