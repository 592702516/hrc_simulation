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

from libs.robot import UR5eRobot
# from gripper import Robotiq85Gripper

class YOLOv8ROS:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('yolov8_ros_node', anonymous=True)

        # Create a CvBridge object
        self.bridge = CvBridge()

        # Defines the path of the YOLOv8 model
        model_path = Path(__file__).parent / "libs" / "models" / "best_final2.pt"

        model_path = Path(__file__).parent/ "src" / "libs" / "models" / "best_final2.pt"

        # Load your trained YOLOv8 model
        self.model = YOLO(model_path)

        # Subscribe to the RGB image topic
        self.image_sub = rospy.Subscriber('/rgb/image_raw', Image, self.image_callback)
        
        self.prev_pos = None


    def image_callback(self, data):
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(f'CvBridge Error: {e}')
            return

        # Perform prediction
        results = self.model(cv_image)

        # Extract class IDs and confidence scores
        predictions = results[0].boxes
        class_ids = predictions.cls.int().tolist()  # Convert to list of class IDs
        confidences = predictions.conf.tolist()     # Convert to list of confidences
        bound_boxes = predictions.xywh.tolist()

        # # Prepare the Float64MultiArray message
        # gesture_msg = Float64MultiArray()
        # tool_lib = Float64MultiArray()

        detections = [(class_id, confidence, bbox) for class_id, confidence, bbox in zip(class_ids, confidences, bound_boxes)]
        detections = [(class_id, confidence, bbox) for class_id, confidence, bbox in detections if confidence >= 0.8]

        # Flatten the class IDs, confidences, bounding boxes into a single list
        gesture_data = [(class_id, confidence, bbox) for class_id, confidence, bbox in detections if ((class_id == 0) or (class_id == 1) or (class_id == 2))]
        tool_data = [(class_id, confidence, bbox) for class_id, confidence, bbox in detections if ((class_id == 3) or (class_id == 4) or (class_id == 5))]

        # checks if there is a tool being detected
        if tool_data:

            print(f"x = {tool_data[0][2][0]}, y = {tool_data[0][2][1]}")

            [x, y, z] = pixel_to_3d(u=tool_data[0][2][0], v=tool_data[0][2][1])

            tf_x, tf_y = transform_coords(x, y)
            tf_z = -0.12    # robot base is 12cm higher than table, DO NOT USE THIS TO MOVE THE ROBOT - WILL CRASH INTO THE TABLE!!!

            print(f"Desired tool is at: {tf_x}, {tf_y}, {tf_z}")

            tf_x = tf_x + 0.1    # add the offset now before checking the boundaries

            # checks if the tool is within the safe boundaries of the workspace
            if ((tf_x > 0.24) and (tf_x < 0.88) and (tf_y > -0.48) and (tf_y < 0.6)):
                print('Safe to move')

                robot_coords = np.array([tf_x, tf_y, 0.25, 2.231, -2.216, 0.0])

                # If there is no previous position, set the previous robot position to home
                if self.prev_pos is None:
                    self.prev_pos =  np.array([0.4, -0.132, 0.250, 2.231, -2.216, 0.0])

                # If left hand has moved more than 5cm from its previous position, execute robot movement
                if((abs(self.prev_pos[0] - robot_coords[0]) > 0.05) or (abs(self.prev_pos[1] - robot_coords[1]) > 0.05)):
                    robot.execute_cartesian_trajectory_preempt(robot_coords)
                    self.prev_pos = robot_coords

                else:
                    print("Tool has not moved far from previous position")

            else:
                print('Tool outside of workspace boundaries')

        else:
            print('No tools detected')

        # Visualize the results
        annotated_image = results[0].plot()

        # Display the image with cv2.imshow
        cv2.imshow('YOLOv8 Detections', annotated_image)
        cv2.waitKey(1)  # Display the image for 1 ms (adjust as needed)


# Converts the tool pixel location to distance with respect to the camera reference frame
def pixel_to_3d(u, v):
    depth = 1.223  

    pixels = np.array([[u, v]], dtype=np.float32)

    K = [913.650390625, 0.0, 955.0496215820312, 
         0.0, 913.8749389648438, 550.6069946289062, 
         0.0, 0.0, 1.0]
    
    D = [0.18453791737556458, -2.423478603363037, 0.00043306572479195893, -0.0002455342037137598, 1.5398979187011719, 0.0690656453371048, -2.238345146179199, 1.4565629959106445]
    
    camera_matrix = np.array(K).reshape((3, 3))

    dist_coeffs = np.array(D)
    
    undistorted_points = cv2.undistortPoints(pixels, camera_matrix, dist_coeffs)
    
    # Compute 3D coordinates
    X = depth * undistorted_points[0][0][0] 
    Y = depth * undistorted_points[0][0][1] -0.13

    return X, Y, depth


# Transforms the tool coordinates from the camera refernce frame to the robot base reference frame
def transform_coords(x, y):

    tf_x = -y + (0.566 - 0.09) #
    tf_y = -x + (1.022 - 0.812)+0.04#   

    sf = 0.07 if tf_y < 0 else 0

    tf_y = tf_y - (sf * tf_y)

    return tf_x, tf_y


if __name__ == '__main__':

    robot = UR5eRobot()
    robot.connect_to_server()
    print("Initiated robot in main")

    try:
        yolov8_ros = YOLOv8ROS()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
