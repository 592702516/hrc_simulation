import rospy
import numpy as np
import time
import threading
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

        # Subscribe to the Azure Kinect body tracking topic
        self.joints = rospy.Subscriber('/body_tracking_data', MarkerArray, self.joint_callback)

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
        self.current_gesture = 2    # change to whatever the ignore state is
        self.prev_gesture = 2       # might not need
        self.gesture_timer = time.time()  # Start a timer

        # Variables for ft sensor
        self.force_y = None
        self.torque_x = None

        self.has_tool = False
        self.need_tool = False
        self.tool_counter = 0

        # Initialising variables for FSM
        self.current_state = 1
        self.current_distance = 0

        # Initialize variables to store joint positions
        self.rhx, self.rhy, self.rhz = None, None, None  # Left hand position
        self.pelvx, self.pelvy, self.pelvz = None, None, None  # Pelvis position        

        self.tool_dictionary = {
            'wrench': 3,
            'pliers': 4,
            'screwdriver': 5
        }

        self.reverse_tool_dictionary = {
            3: 'wrench',
            4: 'pliers',
            5: 'screwdriver'
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
        
        # Define the target resolution (1920x1080 for 1080p)
        width = 1920
        height = 1080
        dim = (width, height)

        # Resize the image to 1080p
        # resized_image = cv2.resize(cv_image, dim, interpolation=cv2.INTER_AREA)

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


    def joint_callback(self, data):
        # print('rospy.spin() is working')
        try:
            if data.markers:
                # Loop through all markers in the MarkerArray
                for marker in data.markers:
                    
                    # Ignoring the first two digits of marker id
                    joint_id = marker.id % 100

                    # Storing pelvis joint coordinates
                    if joint_id == 0:  # ID for pelvis
                        self.pelvx = marker.pose.position.x
                        self.pelvy = marker.pose.position.y
                        self.pelvz = marker.pose.position.z
                        
                    # Storing right-hand joint coordinates
                    elif joint_id == 15:  # ID for left hand
                        self.rhx = marker.pose.position.x
                        self.rhy = marker.pose.position.y
                        self.rhz = marker.pose.position.z

                # Calculating distance between right hand and pelvis
                self.current_distance = np.sqrt((self.pelvx - self.rhx)**2 + (self.pelvy - self.rhy)**2 + (self.pelvz - self.rhz)**2)

            else:
                self.current_distance = 0
        except:
            self.current_distance = 0


# Converts the tool pixel location to distance with respect to the camera reference frame
def pixel_to_3d(u, v):

    depth = 1.275

    # Camera intrinsics
    fx = 913.6504
    fy = 913.8749
    cx = 955.0496
    cy = 550.6070
    
    # Normalize pixel coordinates
    x_norm = (u - cx) / fx
    y_norm = (v - cy) / fy
    
    # Compute 3D coordinates
    X = depth * x_norm
    Y = depth * y_norm
    
    return X, Y

# Transforms the tool coordinates from the camera refernce frame to the robot base reference frame
def transform_coords(x, y, z):

    tf_x = -y + (0.5 + 0.09)
    tf_y = -x + (1.064 - 0.84)
    tf_z = -z + (1.23 - 0.12)

    return tf_x, tf_y, tf_z

# Should use these functions to also raise and lower global flags, indicating whether the gripper has a tool or not.
def open_gripper(gripper):
    success = gripper.open()
    if success:
        rospy.loginfo('Successfully opened')
        yolov8_ros.has_tool = False
        time.sleep(2)
    else:
        rospy.loginfo('Open gripper failed')
        yolov8_ros.has_tool = True
        raise Exception("Cannot open gripper")
    time.sleep(1)

def close_gripper(gripper):
    success = gripper.close()
    if success:
        rospy.loginfo('Successfully closed')
        yolov8_ros.has_tool = True
        time.sleep(1)
    else:
        rospy.loginfo('Close gripper failed')
        yolov8_ros.has_tool = False
        raise Exception("Cannot close gripper")
    time.sleep(1)


# if raising flags in this function should it be moved into the yolov8 class???
def pick_up(current_pos):

    # Lower the gripper to be able to grap tool
    new_pos = np.array([current_pos[0], current_pos[1], 0.135, 2.231, -2.216, 0.0])
    robot.execute_cartesian_trajectory(new_pos)

    # Grab the tool
    close_gripper(gripper)
    rospy.sleep(1)

    # Raise the gripper
    new_pos = np.array([current_pos[0], current_pos[1], 0.40, 2.231, -2.216, 0.0])
    robot.execute_cartesian_trajectory(new_pos)

    # Go home
    robot.go_home()


def hand_over():

    if yolov8_ros.rhx:
        x, y, z = transform_coords(yolov8_ros.rhx, yolov8_ros.rhy, yolov8_ros.rhz)
        print(f'Right hand is at: {x}, {y}, {z}')
        x = x - 0.15
        z = z + 0.30

        if ((x > 0.24) and (x < 0.88) and (y > -0.48) and (y < 0.6) and (z > 0.25) and (z < 0.6)):   # z > 0.25 with gripper, 0.15 without
            print("Safe to move")

            right_hand = np.array([x, y, z, 2.231, -2.216, 0.0])
            robot.execute_cartesian_trajectory_preempt(right_hand)

    # Need to make it go back if the gesture changes!
    while(yolov8_ros.current_gesture == 1):
        if (yolov8_ros.force_y < -110 or yolov8_ros.torque_x > -0.5):
            # Release tool
            open_gripper(gripper)
            rospy.sleep(1)
            yolov8_ros.has_tool = False
            break
        # Should update the gesture within the while loop
        update_gesture()

    robot.go_home()


def update_gesture():
    # If no valid gestures detected, set the current gesture to ignore
    if not yolov8_ros.gesture_data:
        yolov8_ros.current_gesture = 2  # Change to ignore state

    # Else if the detected gesture is different to the current gesture, update the gesture and reset timer
    elif (yolov8_ros.gesture_data[0][0] != yolov8_ros.current_gesture) and (time.time() - yolov8_ros.gesture_timer > 1):
        yolov8_ros.current_gesture = yolov8_ros.gesture_data[0][0]
        yolov8_ros.gesture_timer = time.time()  # Reset timer when gesture changes
        rospy.loginfo("check")
    
    print(yolov8_ros.current_gesture)
    print(yolov8_ros.current_distance)
    print(time.time() - yolov8_ros.gesture_timer)

# Need to add right hand mapping to this function to prevent accidental tool calling
def wait_for_gesture():
    
    update_gesture()

    if ((yolov8_ros.current_gesture == 1) and (time.time() - yolov8_ros.gesture_timer > 1) and (yolov8_ros.current_distance > 0.5) and (yolov8_ros.has_tool)):  
        yolov8_ros.current_state = 1
        yolov8_ros.gesture_timer = time.time()  # Reset timer when state changes
        yolov8_ros.need_tool = True
        rospy.loginfo('Performing tool hand-over')
        
        # Should actually enter a hand tracking state that can be exited if the gesture changes or we leave the frame (similar to the old hand tracking script)
        hand_over()

        # If we exit hand_over without actually giving the tool (if we leave the frame or change gesture), then this logic needs to be changed. Can't reset the flags twice.
        # Reset input flag to ask for another tool
        yolov8_ros.input_flag = True
        yolov8_ros.tool_class = None
        yolov8_ros.desired_tool = None
        yolov8_ros.need_tool = False
        yolov8_ros.tool_counter = yolov8_ros.tool_counter + 1

    elif ((yolov8_ros.current_gesture == 1) and (time.time() - yolov8_ros.gesture_timer > 1) and (yolov8_ros.current_distance > 0.5) and (yolov8_ros.has_tool)):  
        yolov8_ros.current_state = 2
        yolov8_ros.gesture_timer = time.time()  # Reset timer when state changes

        # Reset input flag to ask for another tool
        yolov8_ros.input_flag = True
        yolov8_ros.tool_class = None
        yolov8_ros.desired_tool = None
        yolov8_ros.need_tool = False
        yolov8_ros.tool_counter = yolov8_ros.tool_counter + 1
        rospy.loginfo('Correcting current tool in hand')


def get_tool():
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
            # print(f"{x}, {y}, {z}")
            # print(f"tool width is: {tool_data[0][2][3]}")

            tf_x, tf_y, _ = transform_coords(x, y, z=0)
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
                    # Replace with 'wait_for_gesture()'???
                    pick_up(robot_coords)

                else:
                    print('Tool has not moved far from previous position')

            else:
                print('Tool outside of workspace boundaries')
        
        else:
            print('Desired tool cannot be found')

    else:
        print('No tools detected')

        
# def listener():
#     rospy.spin()


if __name__ == '__main__':

    # Initialize the ROS node
    rospy.init_node('yolov8_ros_node', anonymous=True)

    robot = UR5eRobot()
    robot.connect_to_server()
    gripper = Robotiq85Gripper()

    robot.go_home()

    rate = rospy.Rate(5)  # Set the loop rate to 10 Hz

    try:
        yolov8_ros = YOLOv8ROS()
    except rospy.ROSInterruptException:
        pass

    # spin_thread = threading.Thread(target=listener)
    # spin_thread.start()

    # Generate a random array of tools to pick up
    tool_order = np.array([3, 4, 5, 6])
    print(f"Original array is: {tool_order}")
    np.random.shuffle(tool_order)
    print(f"Shuffled array is: {tool_order}")


    print("Initiated robot in main")
    while not rospy.is_shutdown():

                if yolov8_ros.tool_counter < (np.size(tool_order)):
                    # Condition to update the current desired tool upon start up and when a hand over has been completed
                    if yolov8_ros.input_flag:
                        rospy.loginfo("check")
                        yolov8_ros.tool_class = tool_order[yolov8_ros.tool_counter]
                        yolov8_ros.input_flag = False
                        print(f"Desired tool is: {yolov8_ros.reverse_tool_dictionary.get(tool_order[yolov8_ros.tool_counter])}, tool class is: {yolov8_ros.tool_class}")
                        rospy.sleep(2)

                    # Maybe change to if condition?
                    while not yolov8_ros.has_tool:
                        get_tool()

                    wait_for_gesture()

                else:
                    print('Finished handing over all tools')
                    break



            # rate.sleep()

            rospy.spin()
