#!/usr/bin/env python
# Author: Rui Zhou 
# email: rzho774@aucklanduni.ac.nz

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
import json
import os
import signal
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
        model_path = Path(__file__).parent / "libs" / "models" / "good.pt"
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
        self.tool_positions = {}
        self.rl = ReinforcementLearning([3, 4, 5, 6], [7, 8, 9, 10], epsilon=0.1, learning_rate=0.2)  # Updated for new classes

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

    def get_random_tool(self, process_id):
        return self.rl.choose_action(process_id)

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

    def pick_tool(self, process_id):
        global selected_tool
        selected_tool = self.get_random_tool(process_id)
        rospy.loginfo(f"Attempting to detect tool with class_id: {selected_tool}")
        x, y = self.detect_tool_position(selected_tool)
        
        if x is not None and y is not None:
            self.tool_positions[process_id] = (x, y, 0.1335)  # Store the tool position
            rospy.loginfo(f"Tool detected at ({x}, {y}). Moving to pick it up.")
            self.move_robot([x, y, 0.1335, 2.231, -2.216, 0.0], False, 0.5)
            self.close_gripper()
            self.move_robot_home()
            return True
        return False

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

    def handle_gesture(self, process_id):
        gesture = self.detect_hand_gesture()
        if gesture == "palm" and self.hand_in_boundary:
            x, y, z = self.rhx, self.rhy, self.rhz
            self.move_robot([x, y, z, 2.231, -2.216, 0.0], False, 0.5)
            self.open_gripper()
            self.move_robot_home()
            self.rl.update_probabilities(process_id, selected_tool, 1)  # Positive feedback for ε-greedy update
            return True
        elif gesture == "fist":
            if process_id in self.tool_positions:
                x, y, z = self.tool_positions[process_id]
                self.move_robot([x, y, z, 2.231, -2.216, 0.0], False, 0.5)
                self.open_gripper()  # Drop the tool
                self.move_robot_home()  # Return home
                self.rl.update_probabilities(process_id, selected_tool, 0)  # Negative feedback for ε-greedy update
                return True
        return False

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            try:
                current_process = self.detect_process()
                if current_process is None:
                    rospy.loginfo("No process detected. Waiting...")
                    continue

                if not self.pick_tool(current_process):
                    rospy.loginfo("Failed to pick up tool. Retrying...")
                    continue

                while True:
                    if self.handle_gesture(current_process):
                        break
                    rate.sleep()

            except Exception as e:
                rospy.logerr(f"An error occurred: {e}")

class ReinforcementLearning:
    def __init__(self, tasks, actions, epsilon=0.1, learning_rate=0.2, save_file="try.json"):
        self.tasks = tasks  # Tasks: [3, 4, 5, 6]
        self.actions = actions  # Actions: [7, 8, 9, 10]
        self.epsilon = epsilon  # Exploration probability
        self.learning_rate = learning_rate  # Learning rate
        self.save_file = save_file
        if os.path.exists(self.save_file):
            self.task_action_probabilities = self.load_probabilities()
        else:
            self.task_action_probabilities = {task: np.ones(len(actions)) / len(actions) for task in tasks}

    def choose_action(self, task):
        if random.random() < self.epsilon:
            return random.choice(self.actions)
        else:
            probs = self.task_action_probabilities[task]
            return np.random.choice(self.actions, p=probs)

    def update_probabilities(self, task, action, reward):
        current_probs = self.task_action_probabilities[task]
        action_index = self.actions.index(action)
        if reward == 1:
            increase = self.learning_rate * (1 - current_probs[action_index])
            current_probs[action_index] += increase
        else:
            decrease = self.learning_rate * current_probs[action_index]
            current_probs[action_index] -= decrease

        total_adjustment = decrease if reward == 0 else -increase
        redistribute = total_adjustment / (len(self.actions) - 1)
        for i in range(len(current_probs)):
            if i != action_index:
                current_probs[i] += redistribute
        current_probs = np.clip(current_probs, 0, 1)
        current_probs /= np.sum(current_probs)
        self.task_action_probabilities[task] = current_probs
        self.save_probabilities()

    def save_probabilities(self):
        data = {str(task): probs.tolist() for task, probs in self.task_action_probabilities.items()}
        with open(self.save_file, "w") as f:
            json.dump(data, f)

    def load_probabilities(self):
        try:
            with open(self.save_file, "r") as f:
                data = json.load(f)
            return {int(task): np.array(probs) for task, probs in data.items()}
        except:
            return {task: np.ones(len(self.actions)) / len(self.actions) for task in self.tasks}

def exit_gracefully(signum, frame):
    print("\nProgram interrupted by user. Saving state and exiting...")
    node.rl.save_probabilities()
    rospy.signal_shutdown("User interrupt")

if __name__ == "__main__":
    node = IntegratedToolManipulationNode()
    signal.signal(signal.SIGINT, exit_gracefully)
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass