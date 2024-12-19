import rospy
import numpy as np
import time
from Inventory import Inventory
from Tool import Tool
from libs.robot import UR5eRobot
from libs.gripper import Robotiq85Gripper
import cv2
from std_msgs.msg import Float32MultiArray, Bool


class HRC_FSM:
    def __init__(self,  robot_controller = "ROS", gripper_controller = "ROS"):
        self.robot_controller = robot_controller
        self.gripper_controller = gripper_controller

        self.pub_rp = rospy.Publisher('/required_pos', Float32MultiArray, queue_size=10)
        self.pub_ih = rospy.Publisher('/ignore_hand', Bool, queue_size=10)
        self.goal_active = True

        # # Starting State
        self.state = "Initialised"
        if(robot_controller == "ROS"):
            self.robot = UR5eRobot()
            self.robot.connect_to_server()
            self.robot.go_home()

        if(gripper_controller == "ROS"):
            self.gripper = Robotiq85Gripper()


        # Create assembly tool list for demonstration
        self.assembly_stages = ['screwdriver_phillips', 'T_hex_key', 'wrench_19', 'wrench_17']
        self.stage_counter = 0

        # Variables for gesture recognition 
        self.gesture_data = None
        self.current_gesture = 2    # change to whatever the ignore state is
        self.prev_gesture = 2       # might not need
        self.gesture_timer = time.time()  # Start a timer

        # Variables for ft sensor
        self.force_y = None
        self.torque_x = None

        # Variables for gripper stats
        self.is_moving = None
        self.obj_detect = None
    
        # Initialize variables to store joint positions
        self.rhx, self.rhy, self.rhz = None, None, None  # Left hand position
        self.pelvx, self.pelvy, self.pelvz = None, None, None  # Pelvis position     

        self.current_distance = None  
        self.tool_position = None

        # Create all tool objects
        screwdriver_phillips = Tool(5, 'screwdriver_phillips', (0.25, -0.3))
        wrench_17 = Tool(3, 'wrench_17', (0.25, -0.4))
        wrench_19 = Tool(3, 'wrench_19', (0.25, -0.5))
        T_hex = Tool(6,  'T_hex_key', (0.25, -0.6))

        # Create robot and worker inventories
        self.robot_inventory = Inventory()
        self.worker_inventory = Inventory()
        
        # Add all initial tools to robot inventory
        self.robot_inventory.add_tool(screwdriver_phillips)
        self.robot_inventory.add_tool(wrench_17)
        self.robot_inventory.add_tool(wrench_19)
        self.robot_inventory.add_tool(T_hex)

        print(self.robot_inventory)
        print(self.worker_inventory)


        # FSM Table
        self.transitions = {
            "Initialised": {"tick": ("Loaded", self.initialise)},
            "Loaded": {"closedFist": ("Corrected", self.correct_tool),       #Might also include load Tool as it might have to put the tool back
                     "openPalm": ("Handover", self.handover_tool),       
                       },
            "Handover": {"tug": ("Exchange", self.exchange_tool),
                         "closedFist": ("Corrected", self.correct_tool),
                         "removedHand": ("Loaded", self.return_home),
                         },
            "Exchange": {"tick": ("Return", self.return_tool),
                         },
            "Return": {"tick": ("Loaded", self.load_tool)},
            "Corrected": {"tick": ("Loaded", self.return_home)},
            "Error": {"processError": ("Initialised",self.reset_fsm)}
        }

    
    def move_robot(self, position, wait=True, time = 0):
        if(self.robot_controller == "ROS"):
            self.robot.execute_cartesian_trajectory(np.array(position))
        elif(self.robot_controller == "Avoidance"):
            pos = Float32MultiArray()
            pos.data = position
            self.pub_rp.publish(pos)
            if wait:
                rospy.sleep(0.8)
                goal_not_active_count = 0
                while goal_not_active_count<10:
                    if(not self.goal_active):
                        goal_not_active_count = goal_not_active_count + 1
                    else:
                        goal_not_active_count = 0
                    # print("waiting")
            print(self.goal_active)
            rospy.sleep(time)

        elif(self.robot_controller == "Fake Controller"):
            print(f"Moving to {position}")
            rospy.sleep(5)
            print(f"Successfully moved fake robot to {position}")


    def move_gripper(self, cmd):

        if self.gripper_controller == "ROS":
            if cmd == "close":
                 self.gripper.close()
                 rospy.sleep(0.1)
                 while self.is_moving:
                     rospy.sleep(0.1)
                     continue
                 rospy.sleep(0.5)
            else:
                self.gripper.open()
                rospy.sleep(0.1)
                while self.is_moving:
                     continue
                rospy.sleep(0.5)
        else:
            if cmd == "close":
                print("Fake gripper is now closed")
            else:
                print("Fake gripper is now open")


    def move_robot_home(self, wait=True, time = 0):
        if(self.robot_controller == "ROS"):
            self.robot.go_home()
        elif(self.robot_controller == "Avoidance"):
            self.move_robot(np.array([0.297, -0.132, 0.250, 2.231, -2.216, 0.0]), wait, time)
        elif(self.robot_controller == "Fake Controller"):
            rospy.sleep(5)
            print(f"Successfully moved fake robot to home")



    def handle_event(self, event):
        # Get the current state transitions
        state_transitions = self.transitions.get(self.state, {})

        # Get the transition for the event
        next_state, action = state_transitions.get(event, (None, None))
        
        if next_state:
            # Perform the action associated with the transition
            executed = action()
            # Update the current state
            if executed is True:
                self.state = next_state
                rospy.sleep(0.25)
                print(f"Current state is now '{self.state}'")
                print(f" Robot inventory is: {self.robot_inventory}")
                print(f" Worker inventory is: {self.worker_inventory}")

            else:
                print("Failed to execute action , remaining in intial state")
        # else:

            # print(f"No transition available for event '{event}' in state '{self.state}'")

        rospy.on_shutdown(self.shutdown_node)

    def shutdown_node(self):
        print("Assembly complete, shutting down ROS node")

    def return_home(self):
        self.move_robot_home()
        # Move robot to home coordinates]
        print("Successfully returned home")
        return True

    def initialise(self):
        # Intialise inventory, tool objects, and there permanent positions

        # Hovers over each tool to set-up the workspace - uncomment this when needed
        # self.move_robot((0.25, -0.3, 0.25, 2.231, -2.216, 0.0))
        # rospy.sleep(1)
        # self.move_robot((0.25, -0.4, 0.25, 2.231, -2.216, 0.0))
        # rospy.sleep(1)
        # self.move_robot((0.25, -0.5, 0.25, 2.231, -2.216, 0.0))
        # rospy.sleep(1)
        # self.move_robot((0.25, -0.6, 0.25, 2.231, -2.216, 0.0))

        print('FSM is now initialised')
        self.move_robot_home()
        self.load_tool()
        return True

    def load_tool(self):
        # Keep loading tools until the assembly stages are complete
        if self.stage_counter < len(self.assembly_stages):
            
            tool_required = self.assembly_stages[self.stage_counter]

            print(f"Now loading {tool_required}")

            tool = self.robot_inventory.get_tool(tool_required)

            print(tool.tool_name)

            position = tool.get_position()
            x = position[0]
            y = position[1]

            # goes to above tool location
            self.move_robot([x, y, 0.25, 2.231, -2.216, 0.0], False, 0.5)

            # goes to tool location
            self.move_robot([x, y, 0.1335, 2.231, -2.216, 0.0]) 

            self.move_gripper("close")

            # Checks if the gripper has a tool in-hand
            while not self.obj_detect:
                self.move_robot([x, y, 0.25, 2.231, -2.216, 0.0], False, 0.5)
                self.move_gripper("open")
                x = x + 0.01
                self.move_robot([x, y, 0.1335, 2.231, -2.216, 0.0]) 
                self.move_gripper("close")


            # goes to above tool location with gripped tool
            self.move_robot([x, y, 0.25, 2.231, -2.216, 0.0], False, 0.5)

            # sends robot home
            self.move_robot_home(False, 1)

            if not self.obj_detect:
                self.move_gripper("open")
                self.load_tool()
            else:
                # # updates inventory
                self.robot_inventory.move_tool(tool.tool_name)
                print("Successfully loaded tool")
            
        elif (self.stage_counter == len(self.assembly_stages)):
            # sends robot home
            tool_name = next(iter(self.worker_inventory.on_hand))
            self.worker_inventory.move_tool(tool_name)
            self.state = "Exchange"
            return False

        elif (self.stage_counter > len(self.assembly_stages)):
            self.move_robot_home()
            rospy.signal_shutdown("Assembly complete")
        return True
    

    def handover_tool(self):

        data = Bool()
        data.data = True

        self.pub_ih.publish(data)

        print("Safe to move")

        right_hand = np.array([self.rhx, self.rhy, self.rhz, 2.231, -2.216, 0.0])
        self.move_robot(right_hand, wait=False)

        print("Successfully moved to worker's positions")
        return True


    def exchange_tool(self):
        # Release tool
        self.move_gripper("open")

        # Moves any tool in the worker on hand into their off hand before exchanging if it exists
        if self.worker_inventory.on_hand:
            tool_name = next(iter(self.worker_inventory.on_hand))
            self.worker_inventory.move_tool(tool_name, "off_hand")

        # moves robot's on hand tool to worker's on hand
        tool_name = next(iter(self.robot_inventory.on_hand))
        self.robot_inventory.move_tool(tool_name, "on_hand", self.worker_inventory)
        
        print("Successfully exhanged tool tool")

        data = Bool()
        data.data = False

        for i in range(0,10):
            self.pub_ih.publish(data)
        return True


    def process_error(self):
        # error processing
        print("An error has occured with such and such (display error information)")
        return True

    def reset_fsm(self):
        print("Resetted FSM")
        input("Are all tools in home position?")
        self.stage_counter = 0

        return True

    def correct_tool(self):
        # asks user to confirm through gestures if they want to manually correct the tool
        print("New tool location has been update for xxxxx, loading this new tool")

        return True
    
    def return_tool(self):
        if self.worker_inventory.off_hand:
            # go to tool position
            if self.tool_position:

                x, y = (pixel_to_3d(self.tool_position[0], self.tool_position[1]))

                nx,ny = transform_coords(x, y)

                print(nx, ny)

                # Offset to pick up tool from centre
                nx = nx + 0.2
                
                self.move_robot([nx,ny, 0.25, 2.231, -2.216, 0.0], False, 1)
                self.move_robot([nx,ny, 0.1335, 2.231, -2.216, 0.0])

                # pick up
                self.move_gripper("close")
                # rospy.sleep(1.5)
                self.move_robot([nx,ny, 0.25, 2.231, -2.216, 0.0], False, 1)
                
                # Returns False if no object is in the gripper, keeping the robot in the "Exchange" state
                if not self.obj_detect:
                    self.move_gripper("open")
                    self.tool_position = None
                    # self.return_tool()
                    return False

                # Moves tool from worker off-hand to robot on-hand
                tool_name = next(iter(self.worker_inventory.off_hand))
                self.worker_inventory.move_tool(tool_name, "on_hand", self.robot_inventory)

                # Returns tool to it's home position
                tool = next(iter(self.robot_inventory.on_hand.values()))
                print(tool.tool_position)
                # go tool home posiotion
                self.move_robot([tool.tool_position[0], tool.tool_position[1], 0.25, 2.231, -2.216, 0.0], False, 1)
                self.move_robot([tool.tool_position[0], tool.tool_position[1], 0.140, 2.231, -2.216, 0.0])
                self.move_gripper("open")
                # rospy.sleep(1.5)
                self.move_robot([tool.tool_position[0], tool.tool_position[1], 0.25, 2.231, -2.216, 0.0], False, 1)


                tool_name = next(iter(self.robot_inventory.on_hand))
                self.robot_inventory.move_tool(tool_name, "off_hand")

                self.tool_position = None

            # go home
                print("Successfully returned old tool")
                self.stage_counter = self.stage_counter + 1
                return True
            else: 
                print("Tool not found")
                self.move_robot_home(False)
                return False
        else:
            print("No tool to return")
        
        self.stage_counter = self.stage_counter + 1
        return True
    

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
    X = depth * undistorted_points[0][0][0] -0.02
    Y = depth * undistorted_points[0][0][1]

    return X, Y


# Transforms the tool coordinates from the camera refernce frame to the robot base reference frame
def transform_coords(x, y):

    tf_x = -y + (0.566 - 0.09) #
    tf_y = -x + (1.036 - 0.812)#   

    sf = 0.07 if tf_y < 0 else 0

    tf_y = tf_y - (sf * tf_y)

    return tf_x, tf_y
