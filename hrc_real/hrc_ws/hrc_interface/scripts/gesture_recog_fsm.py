import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from std_msgs.msg import Int32
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

        # Load your trained YOLOv8 model
        self.model = YOLO('best.pt_v3')

        # Subscribe to the RGB image topic
        self.image_sub = rospy.Subscriber('/rgb/image_raw', Image, self.image_callback)
        
        self.joints = rospy.Subscriber('/body_tracking_data', MarkerArray, self.joint_callback)

        self.state = rospy.Publisher('/gesture_state', Int32, queue_size=10)

        # # Create ROS publisher for the detected classes and confidence intervals (should also publish position for picking up tools)
        # self.gesture_pub = rospy.Publisher('/gesture_pub', Float64MultiArray, queue_size=10)

        # # Create ROS publisher for the detected classes and confidence intervals (should also publish position for picking up tools)
        # self.tool_pub = rospy.Publisher('/tool_location', Float64MultiArray, queue_size=10)

        # Initialising variables for FSM
        self.current_state = 1
        self.current_distance = 0
        self.current_gesture = 2    # change to whatever the ignore state is
        self.prev_gesture = 2       # might not need
        self.gesture_timer = time.time()  # Start a timer

        # Initialize variables to store joint positions
        self.lhx, self.lhy, self.lhz = None, None, None  # Left hand position
        self.pelvx, self.pelvy, self.pelvz = None, None, None  # Pelvis position

    def image_callback(self, data):

        print('hi')

        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(f'CvBridge Error: {e}')
            return

        # Run YOLOv8 inference
        results = self.model.predict(cv_image)

        # Extract class IDs and confidence scores
        predictions = results[0].boxes
        class_ids = predictions.cls.int().tolist()  # Convert to list of class IDs
        confidences = predictions.conf.tolist()     # Convert to list of confidences

        # Prepare the Float64MultiArray message
        gesture_msg = Float64MultiArray()

        # Flatten the class IDs and confidences into a single list
        gesture_data = [(class_id, confidence) for class_id, confidence in zip(class_ids, confidences)]

        # Filter out gestures with a confidence interval less than 0.8
        gesture_data = [(class_id, confidence) for class_id, confidence in gesture_data if confidence >= 0.8]

        # Sort by class in the order: ignore (2), closed fist (0), open palm (1)
        gesture_data.sort(key=lambda x: (x[0] != 2, x[0] != 0, x[0] != 1))

        # Formats the array of tuples into a single array for publishing
        gesture_msg.data = [item for sublist in gesture_data for item in sublist]

        # If no valid gestures detected, set the current gesture to ignore
        if not gesture_data:
            self.current_gesture = 2  # Change to ignore state
        elif gesture_data[0][0] != self.current_gesture:
            self.current_gesture = gesture_data[0][0]
            self.gesture_timer = time.time()  # Reset timer when gesture changes

        # Use the gesture, distance, and time to change states. (CIs have already been filtered so they will all be above 0.8)

        print(self.current_gesture)
        print(self.current_distance)
        print(time.time() - self.gesture_timer)

        # State 0:
        if self.current_state == 0:
            print('State 0')
            # If open palm & 2 seconds have passed & left hand is 50cm from, then go to state 1
            if ((self.current_gesture == 1) and (time.time() - self.gesture_timer > 2) and (self.current_distance > 0.5)):  
                self.current_state = 1
                self.gesture_timer = time.time()  # Reset timer when state changes
                print('Transition to state 1')

            # do we need an else statement here?

        # State 1:
        elif self.current_state == 1:
            print('State 1')
            if ((self.current_gesture == 0) and (time.time() - self.gesture_timer > 2) and (self.current_distance > 0.5)):  
                self.current_state = 2
                self.gesture_timer = time.time()  # Reset timer when state changes
                print('Transition to state 2')
            # elif ((self.current_gesture != 1) and (time.time() - self.gesture_timer > 5)):  # Replace with subscriber to the gripper status node to track tool hand over
            #     self.current_state = 0
            #     print('Transition to state 0')

        # State 2:
        elif self.current_state == 2:
            print('State 2')
            if (((self.current_gesture != 1) and (time.time() - self.gesture_timer > 5)) or (self.current_distance < 0.6)):  # Replace with appropriate condition
                self.current_state = 0
                self.gesture_timer = time.time()  # Reset timer when state changes
                print('Transition to state 0')
            elif ((self.current_gesture == 1) and (time.time() - self.gesture_timer > 2) and (self.current_distance > 0.5)):    # Replace with appropriate condition
                self.current_state = 1
                self.gesture_timer = time.time()  # Reset timer when state changes
                print('Transition to state 1')

        # Publish the updated state to a ROS node
        self.state.publish(self.current_state)

        # Visualize the results
        annotated_image = results[0].plot()

        # Display the image with cv2.imshow
        cv2.imshow('YOLOv8 Detections', annotated_image)
        cv2.waitKey(1)  # Display the image for 1 ms (adjust as needed)

    def joint_callback(self, data):
        try:
            if data.markers:
                # Loop through all markers in the MarkerArray
                for marker in data.markers:
                    
                    # ignoring the first two digits of marker id
                    joint_id = marker.id % 100

                    # print(joint_id)

                    # rospy.loginfo(f"Marker ID: {marker.id}, Position: {marker.pose.position}")

                    if joint_id == 0:  # ID for pelvis
                        self.pelvx = marker.pose.position.x
                        self.pelvy = marker.pose.position.y
                        self.pelvz = marker.pose.position.z
                        
                    elif joint_id == 8:  # ID for left hand
                        self.lhx = marker.pose.position.x
                        self.lhy = marker.pose.position.y
                        self.lhz = marker.pose.position.z
                    

                # # print(self.lhx)
                # rospy.loginfo(f"Pelvis: ({self.pelvx}, {self.pelvy}, {self.pelvz})")
                # rospy.loginfo(f"Left Hand: ({self.lhx}, {self.lhy}, {self.lhz})")

                self.current_distance = np.sqrt((self.pelvx - self.lhx)**2 + (self.pelvy - self.lhy)**2 + (self.pelvz - self.lhz)**2)
                # print(self.current_distance)
                # Current distance threshold should be around 0.5m
                # rospy.sleep(1)
            else:
                self.current_distance = 0
        except:
            self.current_distance = 0
        #     print("nothing")
        # print(self.current_distance)

def transform_coords(x,y,z):

    # Transforms coordinates according to camera calibration
    tx = x 
    ty = y 
    tz = z
    return [tx, ty, tz]

if __name__ == '__main__':

    try:
        yolov8_ros = YOLOv8ROS()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
