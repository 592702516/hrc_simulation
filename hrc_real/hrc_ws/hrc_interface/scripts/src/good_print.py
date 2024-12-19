import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray
from cv_bridge import CvBridge
from ultralytics import YOLO
from pathlib import Path
from std_msgs.msg import Float32MultiArray
import random
import time
from libs.robot import UR5eRobot
from libs.gripper import Robotiq85Gripper

class IntegratedToolManipulationNode:
    def __init__(self):
        rospy.init_node('integrated_tool_manipulation_node', anonymous=True)
        self.bridge = CvBridge()
        
        # Subscribers
        self.rgb_sub = rospy.Subscriber("/rgb/image_raw", Image, self.rgb_callback)
        self.joint_sub = rospy.Subscriber("/body_tracking_data", MarkerArray, self.joint_callback)

        # Publishers
        self.pub_rp = rospy.Publisher('/robot_position', Float32MultiArray, queue_size=10)

        # Model setup
        model_path = Path(__file__).parent / "libs" / "models" / "best_final2.pt"
        self.model = YOLO(model_path)

        # Robot and Gripper setup
        self.robot = UR5eRobot()
        self.gripper = Robotiq85Gripper()

        # State variables
        self.latest_rgb = None
        self.rhx, self.rhy, self.rhz = None, None, None
        self.pelvx, self.pelvy, self.pelvz = None, None, None
        self.hand_in_boundary = False
        self.current_distance = None
        self.tool_positions = {}  # Dictionary to store initial tool positions

    def rgb_callback(self, msg):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error processing RGB image: {e}")

    def joint_callback(self, data):
        try:
            if data.markers:
                for marker in data.markers:
                    joint_id = marker.id % 100
                    if joint_id == 0:  # Pelvis
                        self.pelvx, self.pelvy, self.pelvz = marker.pose.position.x, marker.pose.position.y, marker.pose.position.z
                    elif joint_id == 15:  # Right hand
                        self.rhx, self.rhy, self.rhz = self.transform_palm_coords(
                            marker.pose.position.x, 
                            marker.pose.position.y, 
                            marker.pose.position.z
                        )
                        self.rhx -= 0.02
                        self.rhz += 0.32
                        self.check_hand_boundary()
                
                self.current_distance = np.sqrt((self.pelvx - self.rhx)**2 + (self.pelvy - self.rhy)**2 + (self.pelvz - self.rhz)**2)
            else:
                self.current_distance = 0
        except:
            self.current_distance = 0

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

    def transform_palm_coords(self, x, y, z):
        tf_x = -y + (0.566 - 0.09) 
        tf_y = -x + (1.036 - 0.812)
        tf_z = -z + (1.223 - 0.12)
        return tf_x, tf_y, tf_z

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
        print("Successfully moved robot to home")

    def check_hand_boundary(self):
        if ((self.rhx > 0.24) and (self.rhx < 0.88) and (self.rhy > -0.48) and (self.rhy < 0.6) and (self.rhz > 0.05) and (self.rhz < 0.6)):
            self.hand_in_boundary = True
        else:
            self.hand_in_boundary = False

    def get_random_class_id(self, input_number):
        class_map = {
            1: [3, 5, 6],
            2: [3, 5, 6],
            3: [3, 5, 6]
        }
        if input_number not in class_map:
            return None
        return random.choice(class_map[input_number])

    def detect_tool_position(self, selected_class_id):
        if self.latest_rgb is None:
            return None, None
        results = self.model(self.latest_rgb)
        for box in results[0].boxes:
            if int(box.cls[0]) == selected_class_id:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                X, Y = self.pixel_to_3d(center_x, center_y)
                tf_x, tf_y = self.transform_coords(X, Y)
                return tf_x, tf_y
        return None, None

    def pick_tool(self, input_number):
            max_attempts = 999  # Maximum attempts to find a tool
            attempts = 0
            detected_tool = False

            while attempts < max_attempts:
                selected_class_id = self.get_random_class_id(input_number)
                
                if selected_class_id is None:
                    rospy.logwarn("Invalid input number; no tools associated with it.")
                    return

                rospy.loginfo(f"Attempting to detect tool with class_id: {selected_class_id}")
                x, y = self.detect_tool_position(selected_class_id)
                
                if x is not None and y is not None:
                    # Tool detected, proceed with picking it up
                    rospy.loginfo(f"Tool detected at ({x}, {y}). Moving to pick it up.")
                    self.tool_positions[input_number] = (x, y)  # Store the tool position
                    
                    rospy.loginfo(f"Tool detected at ({x}, {y}). Moving to pick it up.")
                    
                    self.move_robot([x, y, 0.1335, 2.231, -2.216, 0.0], False, 0.5)
                    self.close_gripper()
                    self.move_robot_home()
                    detected_tool = True
                    return
                else:
                    rospy.logwarn(f"Tool with class_id {selected_class_id} not detected. Retrying...")

                attempts += 1

            if not detected_tool:
                rospy.loginfo("No tools detected. Need additional tools.")

    def detect_hand_gesture(self):
        if self.latest_rgb is None:
            return None

        results = self.model(self.latest_rgb)
        for box in results[0].boxes:
            class_id = int(box.cls[0])
            if class_id == 0:  # Fist
                return "fist"
            elif class_id == 1:  # Palm
                return "palm"
        return None

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            try:
                input_number = int(input("Enter a number (1, 2, or 3): "))
                self.pick_tool(input_number)
                
                while True:
                    gesture = self.detect_hand_gesture()
                    if gesture == "palm" and self.hand_in_boundary:
                        self.move_robot([self.rhx, self.rhy, self.rhz, 2.231, -2.216, 0.0], False, 0.5)
                        self.open_gripper()
                        self.move_robot_home()
                        break
                    elif gesture == "fist":
                        if input_number in self.tool_positions:
                            x, y = self.tool_positions[input_number]
                            # Move to the tool's initial position to return it
                            self.move_robot([x, y, 0.1335, 2.231, -2.216, 0.0], False, 0.5)
                            self.open_gripper()  # Drop the tool
                            self.move_robot_home()  # Return home
                        break
                    rate.sleep()
            except ValueError:
                print("Invalid input. Please enter a valid number (1, 2, or 3).")

if __name__ == "__main__":
    try:
        node = IntegratedToolManipulationNode()
        node.run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass



