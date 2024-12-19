import rospy
import numpy as np
import time
from pathlib import Path
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
from geometry_msgs.msg import WrenchStamped
from robotiq_85_msgs.msg import GripperStat
import HRC_FSM
from std_msgs.msg import Bool, Int8MultiArray
import logging

# Set YOLOv8's logging to only show warnings and errors
logging.getLogger('ultralytics').setLevel(logging.WARNING)

class YOLOv8ROS:
    def __init__(self):
        self.fsm = HRC_FSM.HRC_FSM("Avoidance", "ROS")

        # Create a CvBridge object
        self.bridge = CvBridge()

        # Defines the path of the YOLOv8 model
        model_path = Path(__file__).parent / "libs" / "models" / "best_final2.pt"

        # Load your trained YOLOv8 model
        self.model = YOLO(model_path)

        # Subscribe to the RGB image topic
        self.image_sub = rospy.Subscriber('/rgb/image_raw', Image, self.image_callback, queue_size=1)

        # Variable to store image from callback
        self.image = None

        # Subscribe to the Azure Kinect body tracking topics
        self.joints = rospy.Subscriber('/body_tracking_data', MarkerArray, self.joint_callback)

        # Subscribe to the ft_sensor topic
        self.ft_sub = rospy.Subscriber('/robotiq_ft_wrench', WrenchStamped, self.ft_callback)  

        # Subscribe to the gripper stat topic
        self.gripstat_sub = rospy.Subscriber('/gripper/stat', GripperStat, self.gs_callback)  

        self.sub_ag = rospy.Subscriber("active_avoidance_goal", Int8MultiArray, self.callback_ag)

        self.hand_in_boundary = False

        self.tool_dictionary = {
            'wrench': 3,
            'screwdriver': 5,
            'T_hex_key': 6
        }

        self.reverse_tool_dictionary = {
            3: 'wrench',
            5: 'screwdriver',
            6: 'T_hex_key'
        }


    def callback_ag(self, data):
        # global goal_active
        self.fsm.goal_active = bool(data.data[0])


    def gs_callback(self, data):
        self.fsm.is_moving = data.is_moving
        self.fsm.obj_detect = data.obj_detected


    def ft_callback(self, data):
        self.fsm.force_y = data.wrench.force.y
        self.fsm.torque_x = data.wrench.torque.x


    def image_callback(self, data):
        
        try:
            # Convert the ROS Image message to an OpenCV image
            self.image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(f'CvBridge Error: {e}')
            return


    def joint_callback(self, data):

        try:
            if data.markers:
                # Loop through all markers in the MarkerArray
                for marker in data.markers:
                    
                    # Ignoring the first two digits of marker id
                    joint_id = marker.id % 100

                    # Storing pelvis joint coordinates
                    if joint_id == 0:  # ID for pelvis
                        self.fsm.pelvx = marker.pose.position.x
                        self.fsm.pelvy = marker.pose.position.y
                        self.fsm.pelvz = marker.pose.position.z
                        
                    # Storing right-hand joint coordinates
                    elif joint_id == 15:  # ID for left hand
                        self.fsm.rhx = marker.pose.position.x
                        self.fsm.rhy = marker.pose.position.y
                        self.fsm.rhz = marker.pose.position.z

                # Calculating distance between right hand and pelvis
                self.fsm.current_distance = np.sqrt((self.fsm.pelvx - self.fsm.rhx)**2 + (self.fsm.pelvy - self.fsm.rhy)**2 + (self.fsm.pelvz - self.fsm.rhz)**2)

                self.fsm.rhx, self.fsm.rhy, self.fsm.rhz = transform_coords(self.fsm.rhx, self.fsm.rhy, self.fsm.rhz)

                
                # print(f'Right hand is at: {self.fsm.rhx}, {self.fsm.rhy}, {self.fsm.rhz}')
                self.fsm.rhx = self.fsm.rhx - 0.02
                self.fsm.rhz = self.fsm.rhz + 0.32

                # checks if right hand is within close proximity of the robot arm to pass object to 
                if ((self.fsm.rhx > 0.24) and (self.fsm.rhx < 0.88) and (self.fsm.rhy > -0.48) and (self.fsm.rhy < 0.6) and (self.fsm.rhz > 0.15) and (self.fsm.rhz < 0.6)):   # z > 0.25 with gripper, 0.15 without
                    self.hand_in_boundary = True
                else:
                    self.hand_in_boundary = False

            else:
                self.fsm.current_distance = 0
        except:
            self.fsm.current_distance = 0


    def update_gesture(self):
        if self.fsm.state == "handover":
            timer = 3
        else:
            timer = 1
        # If no valid gestures detected, set the current gesture to ignore
        if not self.fsm.gesture_data:
            self.fsm.current_gesture = 2  # Change to ignore state

        # Else if the detected gesture is different to the current gesture, update the gesture and reset timer
        elif (self.fsm.gesture_data[0][0] != self.fsm.current_gesture) and (time.time() - self.fsm.gesture_timer > timer):
            self.fsm.current_gesture = self.fsm.gesture_data[0][0]
            self.fsm.gesture_timer = time.time()  # Reset timer when gesture changes
        

def transform_coords(x, y, z):

    tf_x = -y + (0.566 - 0.09) 
    tf_y = -x + (1.036 - 0.812)
    tf_z = -z + (1.223 - 0.12)

    return tf_x, tf_y, tf_z


if __name__ == '__main__':

    # Initialize the ROS node
    rospy.init_node('yolov8_ros_node', anonymous=True)


    rate = rospy.Rate(30)  # Set the loop rate to 10 Hz

    yolov8_ros = YOLOv8ROS()

    while yolov8_ros is None:
        rospy.sleep(0.1)

    while not rospy.is_shutdown():
        results = yolov8_ros.model.predict(yolov8_ros.image)

        # Extract class IDs and confidence scores
        predictions = results[0].boxes
        class_ids = predictions.cls.int().tolist()  # Convert to list of class IDs
        confidences = predictions.conf.tolist()     # Convert to list of confidences
        bound_boxes = predictions.xywh.tolist()

        # Flatten the clas+-    
        # s IDs, confidences, bounding boxes into a single list
        detections = [(class_id, confidence, bbox) for class_id, confidence, bbox in zip(class_ids, confidences, bound_boxes)]

        # Filters out any detections with a confidence interval less than 0.8
        detections = [(class_id, confidence, bbox) for class_id, confidence, bbox in detections if confidence >= 0.83]

        # Sorts the detections into two separate lists of gestures and tools
        yolov8_ros.fsm.gesture_data = [(class_id, confidence, bbox) for class_id, confidence, bbox in detections if ((class_id == 0) or (class_id == 1) or (class_id == 2))]
        yolov8_ros.fsm.tool_data = [(class_id, confidence, bbox) for class_id, confidence, bbox in detections if ((class_id == 3) or (class_id == 4) or (class_id == 5))]

        # Visualize the results
        annotated_image = results[0].plot()

        yolov8_ros.update_gesture()
        if yolov8_ros.hand_in_boundary:
            # print("Hand within valid region")
            if yolov8_ros.fsm.current_gesture == 1:
                # print("open palm")
                yolov8_ros.fsm.handle_event("openPalm")
            elif yolov8_ros.fsm.current_gesture == 0:
                # print("closed palm")
                yolov8_ros.fsm.handle_event("closedFist")
        else: 
            yolov8_ros.fsm.handle_event("removedHand")
        if yolov8_ros.fsm.worker_inventory.off_hand:
            if yolov8_ros.fsm.tool_data:
                for tool in yolov8_ros.fsm.worker_inventory.off_hand.values():
                    desired_tool_data = [(class_id, confidence, bbox) for class_id, confidence, bbox in detections if (class_id == tool.tool_id)]
                    # print(f"Looking for {yolov8_ros.reverse_tool_dictionary.get(desired_tool_data[0][0])}")    
                    for detected_tool in desired_tool_data:
                        if not (detected_tool[2][0] > 1440 and detected_tool[2][1] > 650):
                            print(tool)
                            print(detected_tool)
                            yolov8_ros.fsm.tool_position = [detected_tool[2][0], detected_tool[2][1]]
                            break
                        else:
                            print("The desired tool to return is in home region")
            else:
                print("Tool cannot be found")    

        if (yolov8_ros.fsm.force_y < -110 or yolov8_ros.fsm.torque_x > -0.5):
            yolov8_ros.fsm.handle_event("tug")

        print(f"Current stage is: {yolov8_ros.fsm.stage_counter}")   
        print(f"Current state is: {yolov8_ros.fsm.state}")

        # Display the image with cv2.imshow
        cv2.imshow('YOLOv8 Detections', annotated_image)
        cv2.waitKey(1)  # Display the image for 1 ms (adjust as needed)

        # rospy.loginfo("check")
        yolov8_ros.fsm.handle_event("tick")

        rate.sleep()


    rospy.spin()
