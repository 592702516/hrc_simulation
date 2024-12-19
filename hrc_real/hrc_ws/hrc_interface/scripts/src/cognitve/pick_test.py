import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from pathlib import Path
from std_msgs.msg import Float32MultiArray
import random
import time
from libs.robot import UR5eRobot
from libs.gripper import Robotiq85Gripper

class YOLOv8DepthNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('yolov8_ros_node', anonymous=True)

        # Create a CvBridge object
        self.bridge = CvBridge()

        # Define topics for RGB and depth images
        self.rgb_topic = "/rgb/image_raw"
        self.pub_rp = rospy.Publisher('/robot_position', Float32MultiArray, queue_size=10)

        # Subscribe to image topics
        self.rgb_sub = rospy.Subscriber(self.rgb_topic, Image, self.rgb_callback)

        # Define path to YOLOv8 model
        model_path = Path(__file__).parent / "libs" / "models" / "best_final2.pt"
        self.model = YOLO(model_path)

        # Initialize latest images
        self.latest_rgb = None
        self.robot = UR5eRobot()
        self.gripper = Robotiq85Gripper()
        self.goal_active = True

        # Robot state
        self.robot_controller = "Avoidance"  # or "ROS" or "Fake Controller"
        self.gripper_controller = "ROS"  # or whatever controller you have for the gripper

    def rgb_callback(self, msg):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error processing RGB image: {e}")
    
    def pixel_to_3d(self, u, v):
        depth = 1.223  

        pixels = np.array([[u, v]], dtype=np.float32)

        K = [913.650390625, 0.0, 955.0496215820312, 
             0.0, 913.8749389648438, 550.6069946289062, 
             0.0, 0.0, 1.0]
        
        D = [0.18453791737556458, -2.423478603363037, 0.00043306572479195893, -0.0002455342037137598, 1.5398979187011719, 0.0690656453371048, -2.238345146179199, 1.4565629959106445]

        camera_matrix = np.array(K).reshape((3, 3))
        dist_coeffs = np.array(D)
        
        undistorted_points = cv2.undistortPoints(pixels, camera_matrix, dist_coeffs)
        
        X = depth * undistorted_points[0][0][0] - 0.02
        Y = depth * undistorted_points[0][0][1]

        return X, Y

    def transform_coords(self, x, y):
        tf_x = -y + (0.766 - 0.07) #
        tf_y = -x + (1.036 - 0.812)#   

        sf = 0.07 if tf_y < 0 else 0
        tf_y = tf_y - (sf * tf_y)

        return tf_x, tf_y
    
    def move_robot(self, position, wait=True, time=0):
        self.robot.execute_cartesian_trajectory(np.array(position))
        print(f"Moving to {position}")


    def open_gripper(self):
        success = self.gripper.open()
        if success:
            rospy.loginfo('Successfully opened')
            time.sleep(2)
        else:
            rospy.loginfo('Open gripper failed')
            raise Exception("Cannot open gripper")
        time.sleep(1)

    def close_gripper(self):
        success = self.gripper.close()
        if success:
            rospy.loginfo('Successfully closed')
            time.sleep(1)
        else:
            rospy.loginfo('Close gripper failed')
            raise Exception("Cannot close gripper")
        time.sleep(1)

    def move_robot_home(self, wait=True, time=0):
        self.move_robot(np.array([0.297, -0.132, 0.250, 2.231, -2.216, 0.0]), wait, time)
        print(f"Successfully moved fake robot to home")

    def get_random_class_id(self, input_number):
        # Define possible class_ids based on input_number
        class_map = {
            1: [3, 5, 6],  # If input is 1, the possible class_ids are 3, 5, 6
            2: [3, 5, 6],  # If input is 2, the possible class_ids are 3, 5, 6
            3: [3, 5, 6]   # If input is 3, the possible class_ids are 3, 5, 6
        }

        # Check if the input number is valid
        if input_number not in class_map:
            print("Invalid input. Please enter a number between 1 and 3.")
            return None
        
        # Get the list of possible class_ids for the given input number
        possible_class_ids = class_map[input_number]
        
        # Randomly select one class_id from the list
        selected_class_id = random.choice(possible_class_ids)
        
        return selected_class_id

    def detect_tool_position(self, selected_class_id):
        if self.latest_rgb is None:
            print("No image received.")
            return

        # Run YOLOv8 inference
        results = self.model(self.latest_rgb)

        # Process the results
        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            class_id = int(box.cls[0])

            # If the class_id matches the selected class_id, get the position
            if class_id == selected_class_id:
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                X, Y = self.pixel_to_3d(center_x, center_y)
                tf_x, tf_y = self.transform_coords(X, Y)

                print(f"Detected tool: Class ID {class_id}, Position: ({tf_x}, {tf_y})")
                return tf_x, tf_y

        print(f"No tool with class ID {selected_class_id} detected.")
        return None, None

    def pick_tool(self, input_number):
        selected_class_id = self.get_random_class_id(input_number)
        
        if selected_class_id is None:
            return
        
        # Detect the tool position using YOLO
        x, y = self.detect_tool_position(selected_class_id)
        
        if x is not None and y is not None:
            # Move to the tool position with predefined Z height and orientation
            self.move_robot([x, y, 0.1335, 2.231, -2.216, 0.0], False, 0.5)  # Move to position above the tool
            # Close the gripper to pick up the tool
            self.close_gripper()
            # Move back to home or perform other actions as needed
            self.move_robot_home()

    def run(self):
        rospy.loginfo("YOLOv8DepthNode is running")
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            try:
                input_number = int(input("Enter a number (1, 2, or 3): "))
                self.pick_tool(input_number)
                rate.sleep()
            except ValueError:
                print("Invalid input. Please enter a valid number (1, 2, or 3).")

if __name__ == "__main__":
    try:
        node = YOLOv8DepthNode()
        node.run()
    except rospy.ROSInterruptException:
        pass


