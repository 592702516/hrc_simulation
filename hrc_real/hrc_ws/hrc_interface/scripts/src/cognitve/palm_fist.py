import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray
from cv_bridge import CvBridge
from ultralytics import YOLO
from pathlib import Path


class YOLOv8DepthNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('yolov8_ros_node', anonymous=True)

        # Create a CvBridge object
        self.bridge = CvBridge()

        # Define topics for RGB and depth images
        self.rgb_topic = "/rgb/image_raw"
        self.joint_topic = "/body_tracking_data"

        # Subscribe to image and joint topics
        self.rgb_sub = rospy.Subscriber(self.rgb_topic, Image, self.rgb_callback)
        self.joint_sub = rospy.Subscriber(self.joint_topic, MarkerArray, self.joint_callback)

        # Define path to YOLOv8 model
        model_path = Path(__file__).parent / "libs" / "models" / "best_final2.pt"
        self.model = YOLO(model_path)

        # Initialize latest images and joint data
        self.latest_rgb = None
        self.tf_rhx = self.tf_rhy = self.tf_rhz = None
        self.hand_in_boundary = False

    def rgb_callback(self, msg):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error processing RGB image: {e}")

    def joint_callback(self, data):
        """
        Callback for body tracking data to extract and transform right-hand position.
        """
        try:
            if data.markers:
                for marker in data.markers:
                    joint_id = marker.id % 100

                    # Store right-hand joint coordinates (ID 15 for right hand)
                    if joint_id == 15:  
                        raw_x, raw_y, raw_z = (
                            marker.pose.position.x,
                            marker.pose.position.y,
                            marker.pose.position.z,
                        )
                        # Transform the coordinates
                        self.tf_rhx, self.tf_rhy, self.tf_rhz = self.transform_palm_coords(raw_x, raw_y, raw_z)

                if self.tf_rhx is not None and self.tf_rhy is not None and self.tf_rhz is not None:
                    # Print transformed right-hand position
                    rospy.loginfo(
                        f"Transformed right hand position: X: {self.tf_rhx:.2f}, Y: {self.tf_rhy:.2f}, Z: {self.tf_rhz:.2f}"
                    )

                    # Check if right hand is within robot workspace boundaries
                    self.hand_in_boundary = (
                        0.24 < self.tf_rhx < 0.88 and
                        -0.48 < self.tf_rhy < 0.6 and
                        0.15 < self.tf_rhz < 1.0
                    )
        except Exception as e:
            rospy.logerr(f"Error in joint_callback: {e}")

    # Converts pixel coordinates to 3D camera coordinates
    def pixel_to_3d(self, u, v):
        depth = 1.223  

        pixels = np.array([[u, v]], dtype=np.float32)

        K = [
            913.650390625, 0.0, 955.0496215820312,
            0.0, 913.8749389648438, 550.6069946289062,
            0.0, 0.0, 1.0
        ]
        D = [
            0.18453791737556458, -2.423478603363037,
            0.00043306572479195893, -0.0002455342037137598,
            1.5398979187011719, 0.0690656453371048,
            -2.238345146179199, 1.4565629959106445
        ]

        camera_matrix = np.array(K).reshape((3, 3))
        dist_coeffs = np.array(D)

        undistorted_points = cv2.undistortPoints(pixels, camera_matrix, dist_coeffs)

        X = depth * undistorted_points[0][0][0] - 0.02
        Y = depth * undistorted_points[0][0][1]

        return X, Y

    # Transforms camera coordinates to the robot base reference frame
    def transform_coords(self, x, y):
        # For tool detection
        tf_x = -y + (0.766 - 0.07) 
        tf_y = -x + (1.036 - 0.812)    

        sf = 0.07 if tf_y < 0 else 0
        tf_y = tf_y - (sf * tf_y)

        return tf_x, tf_y

    def transform_palm_coords(self, x, y, z):
        # For palm position
        tf_x = -y + (0.616 - 0.09) 
        tf_y = -x + (1.036 - 0.712)
        tf_z = -z + 1.223 
        return tf_x, tf_y, tf_z

    def process_images(self):
        if self.latest_rgb is None:
            return

        display_image = self.latest_rgb.copy()
        results = self.model(self.latest_rgb)

        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            class_id = int(box.cls[0])

            # Handle detected classes
            if class_id == 0:
                rospy.loginfo("Detected: Fist")
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                X, Y = self.pixel_to_3d(center_x, center_y)
                tf_x, tf_y = self.transform_coords(X, Y)

                rospy.loginfo(f"Tool coordinates in robot frame: ({tf_x:.2f}, {tf_y:.2f})")

                label = f"Robot Coords: ({tf_x:.2f}, {tf_y:.2f})"
                cv2.rectangle(display_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(display_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            elif class_id == 1:
                rospy.loginfo("Detected: Palm")
                if self.hand_in_boundary:
                    rospy.loginfo(
                        f"Right hand position in boundary: X: {self.tf_rhx:.2f}, Y: {self.tf_rhy:.2f}, Z: {self.tf_rhz:.2f}"
                    )
                else:
                    rospy.loginfo("Right hand is outside of boundary")

                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                X, Y = self.pixel_to_3d(center_x, center_y)
                tf_x, tf_y = self.transform_coords(X, Y)
                rospy.loginfo(f"{tf_x:.2f}, {tf_y:.2f}")

                cv2.rectangle(display_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(display_image, "Palm", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        cv2.imshow("YOLOv8 Tool Detection", display_image)
        cv2.waitKey(1)

    def run(self):
        rospy.loginfo("YOLOv8DepthNode is running")
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.process_images()
            rate.sleep()


if __name__ == "__main__":
    try:
        node = YOLOv8DepthNode()
        node.run()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
