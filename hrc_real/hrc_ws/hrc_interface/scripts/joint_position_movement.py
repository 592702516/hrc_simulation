
import sys
from turtle import pos
import rospy
import time
import numpy as np

import sys
import time
import rospy
import actionlib
import math as m
import csv
import os

from sensor_msgs.msg import JointState

from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
import geometry_msgs.msg as geometry_msgs
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryFeedback, JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from scipy.spatial.transform import Rotation
from ur_ikfast import ur_kinematics
from std_msgs.msg import Float64MultiArray, Float32MultiArray, Bool, Int8MultiArray
from tf.transformations import quaternion_matrix, rotation_matrix, euler_matrix, euler_from_quaternion,  quaternion_from_matrix

script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.append(script_dir)

from UR5EJacobian import UR5EJacobian as urj
PI = 3.14159


# Open CSV files to write end effector and obstacle positions
ee_csv_filename = "end_effector_positions.csv"
obstacle_csv_filename = "obstacle_positions.csv"
ee_csv_file = open(ee_csv_filename, mode='w', newline='')
obstacle_csv_file = open(obstacle_csv_filename, mode='w', newline='')
ee_csv_writer = csv.writer(ee_csv_file)
obstacle_csv_writer = csv.writer(obstacle_csv_file)
ee_csv_writer.writerow(['X', 'Y', 'Z'])  # Write header for end effector
obstacle_csv_writer.writerow(['X', 'Y', 'Z'])  # Write header for obstacle


Boundaries = [
    [0.24, 0.88],   #max x
    [-0.48, 0.43],   #max y
    [0.25, 0.6],    #max z
]

min_x = 0.19
max_x = 0.8
min_y = -0.61
max_y = 0.43       #0.6
min_z = 0.132
max_z = 0.6



JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# All of those controllers can be used to execute joint-based trajectories.
# The scaled versions should be preferred over the non-scaled versions.
JOINT_TRAJECTORY_CONTROLLERS = [
    "scaled_pos_joint_traj_controller",
    "scaled_vel_joint_traj_controller",
    "pos_joint_traj_controller",
    "vel_joint_traj_controller",
    "forward_joint_traj_controller",
]

# All of those controllers can be used to execute Cartesian trajectories.
# The scaled versions should be preferred over the non-scaled versions.
CARTESIAN_TRAJECTORY_CONTROLLERS = [
    "pose_based_cartesian_traj_controller",
    "joint_based_cartesian_traj_controller",
    "forward_cartesian_traj_controller",
]

# We'll have to make sure that none of these controllers are running, as they will
# be conflicting with the joint trajectory controllers
CONFLICTING_CONTROLLERS = ["joint_group_vel_controller", "twist_controller"]

def create_transform_matrix(xyz, rpy):
    """
    Create a 4x4 homogeneous transformation matrix from XYZ translation and RPY rotation.
    
    :param xyz: A tuple or list of translation (x, y, z)
    :param rpy: A tuple or list of rotation (roll, pitch, yaw)
    :return: A 4x4 homogeneous transformation matrix
    """
    tx, ty, tz = xyz

    if(len(rpy) == 4):
        rpy = euler_from_quaternion(rpy)

    roll, pitch, yaw = rpy
    
    # Create rotation matrix from RPY
    R = euler_matrix(roll, pitch, yaw)
    
    # Initialize a 4x4 identity matrix
    T = np.eye(4)
    
    # Set the top-left 3x3 submatrix to the rotation matrix
    T[0:3, 0:3] = R[0:3, 0:3]
    
    # Set the top-right 3x1 submatrix to the translation vector
    T[0:3, 3] = [tx, ty, tz]
    
    return T


class Avoidance:
     def __init__(self):
        timeout = rospy.Duration(5)
        # self.joint_trajectory_controller =  "joint_group_vel_controller"
        self._actual_pose = geometry_msgs.Pose()
        self._desired_pose = geometry_msgs.Pose()
        self._position_error = [0, 0, 0]
        self._joint_pose = JointTrajectoryPoint().positions

        self.ur5e_arm = ur_kinematics.URKinematics('ur5e')
        self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[0]

        self.current_pose2 = np.zeros(6)
        self.current_pose = np.zeros(6)
        self.current_cPose = self.joint_to_cart(self.current_pose)
         
        # self.pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=10)
        timeout = rospy.Duration(5)
        # if not self.trajectory_client.wait_for_server(timeout):
        #     # rospy.logerr("Could not reach controller action server.")
        #     sys.exit(-1)  
        # 
        # 
        # self.trajectory_client = actionlib.SimpleActionClient(
        #     "{}/follow_joint_trajectory".format(self.joint_trajectory_controller),
        #     FollowJointTrajectoryAction,
        # )
        # # Wait for action server to be ready
        # timeout = rospy.Duration(1)
        # if not self.trajectory_client.wait_for_server(timeout):
        #     rospy.logerr("Could not reach controller action server.")
        #     sys.exit(-1)     
        
        self.sub = rospy.Subscriber("joint_states", JointState, self.callback)  # Define subscriber
        # self.body_sub =  rospy.Subscriber('/joint_coordinates', Float64MultiArray, self.joint_coordinates_callback)
        self.sub_obs = rospy.Subscriber("/obs_dist", Float32MultiArray, self.callback_obs)
        self.sub_rp = rospy.Subscriber("/required_pos", Float32MultiArray, self.callback_rp)
        self.home_pose = np.array([0.297, -0.132, 0.250, 2.231, -2.216, 0.0])
        self.requested_cPose = self.home_pose
        self.max_speed = 0.1
        self.time_slice = 1
        self.left_hand_pose = np.zeros(3)
        self.jac = urj()
        self.obs_nearest_joint = None
        self.obs_abs_dist = None
        self.obs_direction = None
        self.obs_boundary = False
        self.updated_joints_at_startup = False
        self.obstacle_list = []
        self.old_W = [0, 0, 0, 0, 0, 0]

        self.pub_ag = rospy.Publisher('active_avoidance_goal', Int8MultiArray, queue_size=5)

        self.pub_v = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=10)
 
        print("Initialised Avoidance Object")
        #cartesian interpolation settings

     def callback_rp(self, data):
        if ((data.data[0] > min_x) and (data.data[0] < max_x) and (data.data[1] > min_y) and (data.data[1] < max_y) and (data.data[2] > min_z) and (data.data[2] < max_z)):
            self.requested_cPose = data.data
            print(data.data)
        else:
            print(f"invalid position passed into avoidance server: {data.data}") 
            print(f"invalid position passed into avoidance server: {data.data}") 

     def callback_obs(self, data):
        #  print("Obstacle recieved")
        #  print(data)

        if data.data:
            self.obstacle_list = []
            for i in range(0,int(len(data.data)/6)):
                self.obstacle_list.append([data.data[(i*6) -6], data.data[(i*6) -5], data.data[(i*6) -4:(i*6) -1], data.data[(i*6) -1]])
                self.obs_nearest_joint = data.data[0]
                self.obs_abs_dist = data.data[1]
                self.obs_direction = data.data[2:4]
                self.obs_boundary = bool(data.data[-1])
            # print(self.obstacle_list)
        else:
            self.obstacle_list = None
        # print("check")
     
     def joint_coordinates_callback(self, data):
        try:

            # Extract left hand coordinates from the received message
            lhx = data.data[24]
            lhy = data.data[25]
            lhz = data.data[26]
        
            # Transformed left hand coordinates w.r.t robot base
            base_link_x = -lhy + (0.5 + 0.09)   #0.42
            base_link_y = -lhx + (1.064 - 0.812)  #1.084
            base_link_z = -lhz + (1.23 - 0.12)      # was 1 before camera was adjusted up
            
            
            self.left_hand_pose = [base_link_x, base_link_y, base_link_z]
            # print("left hand is at")
            # print(base_link_x, base_link_y, base_link_z)

              
        except IndexError:
            print("Received data does not contain expected number of coordinates.")
         


     def execute_cartesian_velocity(self, speed, joint = None, link_dist = None):
            # jac = jacobian(dh_params)
            if((joint == None) or (link_dist is None)):
                original_jac = self.jac.jacobian(self.current_pose2)
                joint = 5
                link_dist = 0.0966
            else:
                original_jac = self.jac.jacobian(self.current_pose2, joint, abs(link_dist))
            # inv_jac = np.linalg.inv(jac)

            # joint_vel = np.dot(inv_jac, np.transpose(speed))
            # print(f"Jacobian is {original_jac[:,0:joint+1]}")
            jac = original_jac[:,0:joint+1]
            c = np.sqrt(np.linalg.det(np.dot(np.transpose(jac), jac)))
            # print(f"c is {c}")
            zeta = 0.01 / (np.tan(3.14+c) + 0.01)
            # print(f"zeta is {zeta}")
            new_J = np.dot(np.linalg.inv((np.dot(np.transpose(jac), jac) + (zeta*zeta*np.eye(joint+1)))), np.transpose(jac))

            joint_vel = np.dot(new_J, np.transpose(speed))
            if(joint<5):
                joint_vel = np.append(joint_vel, np.zeros(5-joint))
            return joint_vel
            # self.execute_joint_position(joint_vel)

     def correct_orientation(self):

        # convert current joint coordinates to cartesian coordinates
        old_pose = self.joint_to_cart(self.current_pose2)

        print("Current Cartesian p[ose is]")
        print(old_pose)

        # print(old_pose)
        # quaterion for end effector facing down
        quat = [0.7071068, 0.7071068, 0.01, 0.01]

        # applies joint quarterion and gets new pose
        new_pose = self.cart_to_joint(old_pose[:3]+(quat))


        # Checks IK solution incase you get an extreme one
        diff = np.sum(np.abs(new_pose - old_pose))

        if(diff<6):
            # Create and fill trajectory goal
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = JOINT_NAMES

            point = JointTrajectoryPoint()
            point.positions = new_pose
            point.time_from_start = rospy.Duration(2)
            goal.trajectory.points.append(point)

            self.trajectory_client.send_goal(goal)
            print("Corrected end effector solution")
        else:
            print("BAD IK solution")
            print(diff)
            print(old_pose)
            print(new_pose)
         
         
    # redefined to use a position controller rather than using a velocity controller
     def execute_joint_position(self, joint_vel, time = 0.1):
        # scale all the joint speeds relatively if one or more joints exceed the max speed threshold set
        # print(f"recieve {joint_vel}")
        # Keep it at 0.1 for testi\ng obstacle avoidance else 0.4 for actual
        threshold_speed = 0.4
        max_joint_speed = 0

        # acceleration checks and scaling

        # finds the max requsted accel from the new joint velocity
        requested_accels = (joint_vel - self.old_W)/time
        max_requested_accel = max(abs(requested_accels))
        
        # if the acceleration exeeds the current limit, scales the acceerations and alterantively, changes the velocity
        if max_requested_accel> 0.6:     #0.5
            accel_scaling_factor  = 0.6/max_requested_accel
            new_accels = requested_accels * accel_scaling_factor
            joint_vel = (new_accels*time) + self.old_W


        for jv, val in enumerate(joint_vel):
            if(abs(val)) > max_joint_speed:
                max_joint_speed = abs(val)
            
        # print(f"sent speed is {joint_vel}")
        if (max_joint_speed  > threshold_speed):
            # print("joint speed at limit")
            scaling_factor = threshold_speed / max_joint_speed
            joint_vel = joint_vel * scaling_factor
            # print(f"new speed 
            # 
            # 
            # 
            # is {joint_vel}")

        self.old_W = joint_vel


        # print(f" send {joint_vel}")
        traj = Float64MultiArray()
        traj.data = joint_vel
        # print(traj)
        self.pub_v.publish(traj)


     def _switch_controller(self, target_controller):
        """Activates the desired controller and stops all others from the predefined list above"""
        other_controllers = (
            JOINT_TRAJECTORY_CONTROLLERS
            + CARTESIAN_TRAJECTORY_CONTROLLERS
            + CONFLICTING_CONTROLLERS
        )

        other_controllers.remove(target_controller)

        srv = LoadControllerRequest()
        srv.name = target_controller
        self.load_srv(srv)

        srv = SwitchControllerRequest()
        srv.stop_controllers = other_controllers
        srv.start_controllers = [target_controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_srv(srv)
        self._current_controller = target_controller
    
     def go_home(self):
         self.potential_field(self.home_pose)

     
     def callback(self, data):
        if(self.updated_joints_at_startup is False):
            self.updated_joints_at_startup = True
        self.current_pose2 = np.array([data.position[2],data.position[1], data.position[0], data.position[3],data.position[4],data.position[5]])

        # for i in range(len(self.current_pose2)):
        #     dh_params[i][3] = self.current_pose2[i]


     def _vec2quat(self, x, y, z):
         # Create a rotation object from rotation vector in radians
         rot = Rotation.from_rotvec([x, y, z])
          # Convert to quaternions and print
         rq = rot.as_quat()
         return [rq[0], rq[1], rq[2], rq[3]]
     
     def _quat2vec(self, x, y, z, w):
        # Create a rotation object from rotation vector in radians
        rot = Rotation.from_quat([x, y, z, w])
        # Convert to quaternions and print

        
        rr = rot.as_euler('xyz', degrees=False)
        # rr = rot.as_mrp()
        return rr[0], rr[1], rr[2]
     
     def joint_to_cart(self, pose):
        # convert joint pose(in radians) to cartisian point
        pose_quat = self.ur5e_arm.forward(pose)
        # return pose_quat
        vec = self._quat2vec(pose_quat[3],pose_quat[4], pose_quat[5], pose_quat[6])
        # return vec
        return [pose_quat[0], pose_quat[1], pose_quat[2], vec[0], vec[1], vec[2]]

     
     def potential_field(self, position):
        threshold_distance = 0.25
        threshold_boundary_distance = 0.1
        k_attr=1.5
        k_rep = 0.001
        k_rep_b = 0.002
        self.current_cPose=self.joint_to_cart(self.current_pose2)
        abs_dist = np.linalg.norm(np.array(position[:3]) - np.array(self.current_cPose[:3]))
        rate = rospy.Rate(17)    #20
        local_minimum_flag = False
        local_minimum_counter = 0

        # If the target position is within boundaries
        if ((position[0] > min_x) and (position[0] < max_x) and (position[1] > min_y) and (position[1] < max_y) and (position[2] > min_z) and (position[2] < max_z)): 
        #  find current pose
            while abs_dist > 0.002:
                self.current_cPose=self.joint_to_cart(self.current_pose2)
                self.current_cPose[0] = self.current_cPose[0] * -1
                self.current_cPose[1] = self.current_cPose[1] * -1

                
                # print("current pose is {}", self.current_cPose)
                abs_dist = np.linalg.norm(np.array(position[:3]) - np.array(self.current_cPose[:3]))

                v = np.array(position[:3]) - np.array(self.current_cPose[:3])
                d_hat = v / np.linalg.norm(v)
                d_hat = d_hat / np.linalg.norm(d_hat)

                v_attr = k_attr * abs_dist

                if(v_attr > 0.25):
                    v_attr = 0.25
                
                v_repel = 0
                Vx = (v_attr * d_hat[0])
                Vy = (v_attr * d_hat[1])
                Vz = (v_attr * d_hat[2])

                w_attr = self.execute_cartesian_velocity([Vx, Vy, Vz, 0, 0, 0], 5, 0.0966)

                w_body = np.zeros(6)
                w_bound = np.zeros(6)
                w_bound_end = np.zeros(6)
                w_total_repel = np.zeros(6)
                if(self.obstacle_list is not None):
                    for (joint, link_dist, r, ws_bool) in self.obstacle_list:
                        mag_r = np.linalg.norm(r)
                            # will stop the script if the obstacle is touching the robot arm as this is
                            # dangerous
                        if(mag_r != 0):
                                
                            r_hat = r/mag_r

                            if ws_bool:  
                                print("Near Boundary")
                                if(mag_r<=threshold_boundary_distance):  #add if no obstacles are nearby 
                                    v_repel = k_rep_b* (((1/mag_r) - (1/threshold_boundary_distance)) * (1/(mag_r * mag_r)))
                                else:
                                    v_repel = 0
                            else:
                                if(mag_r<=threshold_distance):
                                    v_repel = k_rep * (((1/mag_r) - (1/threshold_distance)) * (1/(mag_r * mag_r)))
                                else:
                                    v_repel = 0


                                # psuedo code if no person obstacles and near boundary where goal is within, cause boundary to decay
                            # if()
                            if(v_repel > 0.3):
                                v_repel = 0.3
                            print(f"v_repel is {v_repel}")


                            Vx = -v_repel * r_hat[0]
                            Vy = -v_repel * r_hat[1]
                            Vz = v_repel * r_hat[2]

                                                # will stop the script if the obstacle is touching the robot arm as this is
                        # dangerous
                        if(mag_r != 0):
                            w_repel = self.execute_cartesian_velocity([Vx, Vy, Vz, 0, 0, 0], int(joint), link_dist)
                        else:
                            print("Obstacle too close to arm, ignoring obstacle")

                        
                        if ws_bool:
                            if(joint == 5):
                                w_bound_end = w_bound_end + w_repel
                            else:
                                w_bound = w_bound + w_repel
                            
                        else:
                            w_body = w_body + w_repel


                    if(np.all(w_bound == 0) and np.all(w_body==0)):
                        print("No obstacles nearby, ignore workspace obstacles near end effector")
                        w_total_repel = w_bound   #
                    else:
                        print("Person or other obstacles are within proximity of cobot")
                        w_total_repel = w_bound+ w_bound_end + w_repel

                w_total = w_total_repel + w_attr
                

                self.execute_joint_position(w_total, 0.12)


                rate.sleep()
        else:
            print("Position is out of bounds")

        # self.execute_joint_velocity([0,0,0,0,0,0])
     
     def avoidance_server(self):
        threshold_distance = 0.2
        threshold_boundary_distance = 0.1
        k_attr=3
        k_rep = 0.001
        k_rep_b = 0.0012
        self.current_cPose=self.joint_to_cart(self.current_pose2)
        abs_dist = np.linalg.norm(np.array(self.requested_cPose[:3]) - np.array(self.current_cPose[:3]))
        rate = rospy.Rate(75)
        local_min_counter = 0
        corrected = False
        #  find current pose
        while not rospy.is_shutdown():
            self.current_cPose=self.joint_to_cart(self.current_pose2)
            self.current_cPose[0] = self.current_cPose[0] * -1
            self.current_cPose[1] = self.current_cPose[1] * -1

            abs_dist = np.linalg.norm(np.array(self.requested_cPose[:3]) - np.array(self.current_cPose[:3]))

            v = np.array(self.requested_cPose[:3]) - np.array(self.current_cPose[:3])
            d_hat = v / np.linalg.norm(v)
            d_hat = d_hat / np.linalg.norm(d_hat)

            v_attr = k_attr * abs_dist

            if(v_attr > 0.25):
                v_attr = 0.25
            # print("v_atrr is ")
            # print(v_attr)

            
            v_repel = 0
            Vx = (v_attr * d_hat[0])
            Vy = (v_attr * d_hat[1])
            Vz = (v_attr * d_hat[2])

            w_attr = self.execute_cartesian_velocity([Vx, Vy, Vz, 0, 0, 0], 5, 0.0966)

            w_body = np.zeros(6)
            w_bound = np.zeros(6)
            w_bound_end = np.zeros(6)
            w_total_repel = np.zeros(6)
            if(self.obstacle_list is not None):
                for (joint, link_dist, r, ws_bool) in self.obstacle_list:
                    mag_r = np.linalg.norm(r)
                    if(mag_r != 0):
                            
                        r_hat = r/mag_r

                        if ws_bool:  
                            # print("Near Boundary")
                            if(mag_r<=threshold_boundary_distance):  #add if no obstacles are nearby 
                                v_repel = k_rep_b* (((1/mag_r) - (1/threshold_boundary_distance)) * (1/(mag_r * mag_r)))
                            else:
                                v_repel = 0
                        else:
                            if(mag_r<=threshold_distance):
                                v_repel = k_rep * (((1/mag_r) - (1/threshold_distance)) * (1/(mag_r * mag_r)))
                            else:
                                v_repel = 0

                        if(v_repel > 0.3):
                            v_repel = 0.3
                        # print(f"v_repel is {v_repel}")


                        Vx = -v_repel * r_hat[0]
                        Vy = -v_repel * r_hat[1]
                        Vz = v_repel * r_hat[2]

                                            # will stop the script if the obstacle is touching the robot arm as this is
                    # dangerous
                    if(mag_r != 0):
                        w_repel = self.execute_cartesian_velocity([Vx, Vy, Vz, 0, 0, 0], int(joint), link_dist)
                    else:
                        print("Obstacle too close to arm, ignoring obstacle")

                    
                    if ws_bool:
                        if(joint == 5):
                            w_bound_end = w_bound_end + w_repel
                        else:
                            w_bound = w_bound + w_repel
                        
                    else:
                        w_body = w_body + w_repel


                if(np.all(w_bound == 0) and np.all(w_body==0)):
                    # print("No obstacles nearby, ignore workspace obstacles near end effector")
                    w_total_repel = w_bound   #
                else:
                    # print("Person or other obstacles are within proximity of cobot")
                    w_total_repel = w_bound+ w_bound_end + w_repel

            w_total = w_total_repel + w_attr


            error_X = 0
            if(self.current_cPose[3]<3.14159 and self.current_cPose[3]>0):
                error_X = -3.14159 + self.current_cPose[3]
            else:
                error_X = 3.14159 + self.current_cPose[3]


            error_Y = 0
            if(self.current_cPose[4]<3.14159 and self.current_cPose[4]>0):
                error_Y = 0 - self.current_cPose[4]
            else:
                error_Y = 0 - self.current_cPose[4]

            w = self.execute_cartesian_velocity([0, 0, 0, error_Y, error_X, 0], 5, 0.0996)


            w_total = w_total + w

            status = Int8MultiArray()

            if(abs_dist<0.001):
                w_zero = np.array([0, 0, 0, 0, 0, 0])

                status.data.append(0)

            else:
                self.execute_joint_position(w_total, (1/75))
                status.data.append(1)

            if (((np.sum(np.abs(w_total_repel))) > 1) and (np.sum(np.abs(w_total))<2.5) and (abs_dist>0.1)):
                local_min_counter = local_min_counter + 1
    
            else:
                local_min_counter = 0

            if(local_min_counter>10):
                status.data.append(1)
            else:
                status.data.append(0)

            self.pub_ag.publish(status)

            
            rate.sleep()
         
         
     def servo(self, vx, vy, vz, wx, wy, wz, joint, link_Dist, time):
        self.current_cPose=self.joint_to_cart(self.current_pose2)
        self.current_cPose[0] = self.current_cPose[0] * -1
        self.current_cPose[1] = self.current_cPose[1] * -1

        w = self.execute_cartesian_velocity([vx, vy, vz, wx, wy, wz], joint, link_Dist)
        self.execute_joint_position(w, time)
        print("Done Executing")
        self.current_cPose=self.joint_to_cart(self.current_pose2)
        self.current_cPose[0] = self.current_cPose[0] * -1
        self.current_cPose[1] = self.current_cPose[1] * -1

     def cart_to_joint(self, cartesian_pose):
        # convert joint pose(in radians) to cartisian point
        pose_joint = self.ur5e_arm.inverse(cartesian_pose, False, self.current_pose2)
        return pose_joint

def main_loop():
    robot = Avoidance()
    jac = urj()

    # wy = -wx

    rate = rospy.Rate(20)


    # need to update states as having joint angles at zero may cause unexpected behaviour is cartesian velocity is evaluated
    while robot.updated_joints_at_startup is False:
        rospy.sleep(0.1)

    robot.avoidance_server()


    print("done")

        

if __name__ == "__main__":
    rospy.init_node("UR5e_Mover")
    main_loop()



    



