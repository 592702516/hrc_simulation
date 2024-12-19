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
from geometry_msgs.msg import WrenchStamped

from libs.robot import UR5eRobot
from libs.gripper import Robotiq85Gripper

import logging

# Set YOLOv8's logging to only show warnings and errors
logging.getLogger('ultralytics').setLevel(logging.WARNING)

class YOLOv8ROS:
    def __init__(self):

        # Create a CvBridge object
        self.bridge = CvBridge()

        # Defines the path of the YOLOv8 model
        model_path = Path(__file__).parent / "libs" / "models" / "best_v3.pt"

        # Load your trained YOLOv8 model
        self.model = YOLO(model_path)

        # Subscribe to the RGB image topic
        self.image_sub = rospy.Subscriber('/rgb/image_raw', Image, self.image_callback)

        # Subscribe to the ft_sensor topic
        self.ft_sub = rospy.Subscriber('/robotiq_ft_wrench', WrenchStamped, self.ft_callback)

        # Variables for tool hand over
        self.prev_pos = None
        self.input_flag = True
        self.desired_tool = None
        self.tool_class = None
        self.tool_data = None

        # Variables for gesture recognition
        self.gesture_data = None

        # Variables for ft sensor
        self.force_y = None
        self.torque_x = None

        self.tool_dictionary = {
            'wrench': 3,
            'pliers': 4,
            'screwdriver': 5
        }

    def ft_callback(self, data):
        self.force_y = data.wrench.force.y
        self.torque_x = data.wrench.torque.x


    def image_callback(self, data):
        
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(f'CvBridge Error: {e}')
            return

        results = self.model(cv_image)
        
        # Perform prediction once an input has been given
        if not self.input_flag:
            # Extract class IDs and confidence scores
            predictions = results[0].boxes
            class_ids = predictions.cls.int().tolist()  # Convert to list of class IDs
            confidences = predictions.conf.tolist()     # Convert to list of confidences
            bound_boxes = predictions.xywh.tolist()

            # Flatten the class IDs, confidences, bounding boxes into a single list
            detections = [(class_id, confidence, bbox) for class_id, confidence, bbox in zip(class_ids, confidences, bound_boxes)]

            # Filters out any detections with a confidence interval less than 0.8
            detections = [(class_id, confidence, bbox) for class_id, confidence, bbox in detections if confidence >= 0.83]

            # Sorts the detections into two separate lists of gestures and tools
            self.gesture_data = [(class_id, confidence, bbox) for class_id, confidence, bbox in detections if ((class_id == 0) or (class_id == 1) or (class_id == 2))]
            self.tool_data = [(class_id, confidence, bbox) for class_id, confidence, bbox in detections if ((class_id == 3) or (class_id == 4) or (class_id == 5))]

        # Visualize the results
        annotated_image = results[0].plot()

        # Display the image with cv2.imshow
        cv2.imshow('YOLOv8 Detections', annotated_image)
        cv2.waitKey(1)  # Display the image for 1 ms (adjust as needed)


# Converts the tool pixel location to distance with respect to the camera reference frame
def pixel_to_3d(u, v):
    depth = 1.275  

    pixels = np.array([[u, v]], dtype=np.float32)

    K = [913.650390625, 0.0, 955.0496215820312, 
         0.0, 913.8749389648438, 550.6069946289062, 
         0.0, 0.0, 1.0]
    
    D = [0.18453791737556458, -2.423478603363037, 0.00043306572479195893, -0.0002455342037137598, 1.5398979187011719, 0.0690656453371048, -2.238345146179199, 1.4565629959106445]
    
    camera_matrix = np.array(K).reshape((3, 3))

    dist_coeffs = np.array(D)
    
    undistorted_points = cv2.undistortPoints(pixels, camera_matrix, dist_coeffs)
    
    # Compute 3D coordinates
    X = depth * undistorted_points[0][0][0] - 0.02
    Y = depth * undistorted_points[0][0][1]

    return X, Y

# Transforms the tool coordinates from the camera refernce frame to the robot base reference frame
def transform_coords(x, y):

    tf_x = -y + (0.5 + 0.09)
    tf_y = -x + (1.064 - 0.84)

    sf = 0.07 if tf_y < 0 else 0

    tf_y = tf_y - (sf * tf_y)

    return tf_x, tf_y


# if raising flags in this function should it be moved into the yolov8 class???
def pick_up(current_pos):

    # Lower the gripper to be able to grap tool
    new_pos = np.array([current_pos[0], current_pos[1], 0.135, 2.231, -2.216, 0.0])
    robot.execute_cartesian_trajectory(new_pos)

    # Grab the tool
    gripper.close()
    rospy.sleep(3)

    # Hand tool over to worker
    new_pos = np.array([current_pos[0], current_pos[1], 0.40, 2.231, -2.216, 0.0])
    robot.execute_cartesian_trajectory(new_pos)
    new_pos = np.array([0.6, -0.2, 0.40, 2.3, -2.3, 0.2])
    robot.execute_cartesian_trajectory(new_pos)

    # Release tool
    while not (yolov8_ros.force_y < -110 or yolov8_ros.torque_x > -0.5):
        continue
    gripper.open()
    rospy.sleep(1)

    # Back up
    new_pos = np.array([0.4, -0.2, 0.60, 2.231, -2.216, 0.0])
    robot.execute_cartesian_trajectory(new_pos)

    # Go home
    robot.go_home()


if __name__ == '__main__':

    # Initialize the ROS node
    rospy.init_node('yolov8_ros_node', anonymous=True)

    robot = UR5eRobot()
    robot.connect_to_server()
    gripper = Robotiq85Gripper()

    robot.go_home()

    try:
        yolov8_ros = YOLOv8ROS()
    except rospy.ROSInterruptException:
        pass


    print("Initiated robot in main")
    while not rospy.is_shutdown():

        rospy.loginfo("check")
        # Code blocking input statement that only runs on startup
        if yolov8_ros.input_flag:
            rospy.loginfo("check2")
            # Keeps asking for a tool until a valid input is given
            while not yolov8_ros.tool_class:
                # should i make tool_name and tool_class class attributes??
                tool_name = input("Type the desired tool name and press enter: ")
                tool_name = tool_name.lower()
                yolov8_ros.tool_class = yolov8_ros.tool_dictionary.get(tool_name)
                if not yolov8_ros.tool_class:
                    print('Invalid tool')
            yolov8_ros.input_flag = False
            print(f"Desired tool is: {tool_name}, tool class is: {yolov8_ros.tool_class}")

        
         # Checks if there is a tool being detected
        if yolov8_ros.tool_data:

            # Loops through all detected tools and stores the data of the desired tool
            for tool in yolov8_ros.tool_data:
                if tool[0] == yolov8_ros.tool_class:
                    yolov8_ros.desired_tool = tool
                    break
            
            # Checks that the desired tool is in the frame
            if yolov8_ros.desired_tool:

                # calculates the location of the tool with the highest confidence interval
                [x, y] = pixel_to_3d(u=yolov8_ros.desired_tool[2][0], v=yolov8_ros.desired_tool[2][1])

                tf_x, tf_y = transform_coords(x, y)
                tf_z = -0.12    # robot base is 12cm higher than table, DO NOT USE THIS TO MOVE THE ROBOT - WILL CRASH INTO THE TABLE!!!

                print(f"Desired tool is at: {tf_x}, {tf_y}, {tf_z}")

                tf_x = tf_x + 0.10    # add the offset now before checking the boundaries

                # checks if the tool is within the safe boundaries of the workspace
                if ((tf_x > 0.24) and (tf_x < 0.88) and (tf_y > -0.48) and (tf_y < 0.6)):
                    print('Safe to move')

                    robot_coords = np.array([tf_x, tf_y, 0.25, 2.231, -2.216, 0.0])

                    # If there is no previous position, set the previous robot position to home
                    if yolov8_ros.prev_pos is None:
                        yolov8_ros.prev_pos =  np.array([0.4, -0.132, 0.250, 2.231, -2.216, 0.0])

                    # If left hand has moved more than 5cm from its previous position, execute robot movement
                    if((abs(yolov8_ros.prev_pos[0] - robot_coords[0]) > 0.05) or (abs(yolov8_ros.prev_pos[1] - robot_coords[1]) > 0.05)):
                        robot.execute_cartesian_trajectory(robot_coords)    # note: needs to use the preempt as this is non-blocking and the camera will keep updating
                        yolov8_ros.prev_pos = robot_coords

                        # Pick up and hand-over tool
                        pick_up(robot_coords)

                        # Reset input flag to ask for another tool
                        yolov8_ros.input_flag = True
                        yolov8_ros.tool_class = None
                        yolov8_ros.desired_tool = None

                    else:
                        print('Tool has not moved far from previous position')

                else:
                    print('Tool outside of workspace boundaries')
            
            else:
                print('Desired tool cannot be found')

        else:
            print('No tools detected')

    rospy.spin()


