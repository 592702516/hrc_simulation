#!/usr/bin/env python
import rospy
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Float32MultiArray
from robot_control import RobotControl
from vision_processing import VisionProcessing
from sensor_msgs.msg import Image
from e_greedy import EGreedy
import signal
import numpy as np
import time

class IntegratedToolManipulationNode:
    def __init__(self):
        rospy.init_node('integrated_tool_manipulation_node', anonymous=True)
        
        self.vision = VisionProcessing()
        self.robot = RobotControl()
        self.e_greedy = EGreedy([3, 4, 5, 6], [7, 8, 9, 10], epsilon=0.1, learning_rate=0.2)

        # Subscribers
        self.rgb_sub = rospy.Subscriber("/rgb/image_raw", Image, self.vision.rgb_callback)
        self.joint_sub = rospy.Subscriber("/body_tracking_data", MarkerArray, self.joint_callback)

        # Publishers
        self.pub_rp = rospy.Publisher('/robot_position', Float32MultiArray, queue_size=10)

        # State variables
        self.rhx, self.rhy, self.rhz = None, None, None
        self.pelvx, self.pelvy, self.pelvz = None, None, None
        self.hand_in_boundary = False
        self.current_distance = None
        self.tool_positions = {}
        self.selected_tool = None

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

    def transform_palm_coords(self, x, y, z):
        tf_x = -y + (0.566 - 0.09) 
        tf_y = -x + (1.036 - 0.812)
        tf_z = -z + (1.223 - 0.12)
        return tf_x, tf_y, tf_z

    def check_hand_boundary(self):
        if ((self.rhx > 0.24) and (self.rhx < 0.88) and (self.rhy > -0.48) and (self.rhy < 0.6) and (self.rhz > 0.05) and (self.rhz < 0.6)):
            self.hand_in_boundary = True
        else:
            self.hand_in_boundary = False

    def get_random_tool(self, process_id):
        return self.e_greedy.choose_action(process_id)

    def pick_tool(self, process_id):
        self.selected_tool = self.get_random_tool(process_id)
        rospy.loginfo(f"Attempting to detect tool with class_id: {self.selected_tool}")
        x, y = self.vision.detect_tool_position(self.selected_tool)
        
        if x is not None and y is not None:
            self.tool_positions[process_id] = (x, y, 0.1335)  # Store the tool position
            rospy.loginfo(f"Tool detected at ({x}, {y}). Moving to pick it up.")
            self.robot.move_robot([x, y, 0.1335, 2.231, -2.216, 0.0], False, 0.5)
            self.robot.close_gripper()
            self.robot.move_robot_home()
            return True
        return False

    def handle_gesture(self, process_id):
        gesture = self.vision.detect_hand_gesture()
        if gesture == "palm" and self.hand_in_boundary:
            x, y, z = self.rhx, self.rhy, self.rhz
            self.robot.move_robot([x, y, z, 2.231, -2.216, 0.0], False, 0.5)
            self.robot.open_gripper()
            self.robot.move_robot_home()
            self.e_greedy.update_probabilities(process_id, self.selected_tool, 1)  # Positive feedback for ε-greedy update
            return True
        elif gesture == "fist":
            if process_id in self.tool_positions:
                x, y, z = self.tool_positions[process_id]
                self.robot.move_robot([x, y, z, 2.231, -2.216, 0.0], False, 0.5)
                self.robot.open_gripper()  # Drop the tool
                self.robot.move_robot_home()  # Return home
                self.e_greedy.update_probabilities(process_id, self.selected_tool, 0)  # Negative feedback for ε-greedy update
                return True
        return False

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            try:
                current_process = self.vision.detect_process()
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

if __name__ == "__main__":
    node = IntegratedToolManipulationNode()
    signal.signal(signal.SIGINT, lambda signum, frame: (rospy.loginfo("Exiting..."), node.e_greedy.save_probabilities(), rospy.signal_shutdown("User interrupt")))
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass