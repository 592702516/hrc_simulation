import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import Constraints, OrientationConstraint, JointConstraint
from geometry_msgs.msg import PoseStamped
import sys
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
import numpy as np
# from moveit_commander import IterativeParabolicTimeParameterization
from ur_ikfast import ur_kinematics
import warnings
from scipy.spatial.transform import Rotation
from std_msgs.msg import Float64MultiArray, Float32MultiArray, Bool, Int8MultiArray
import time


# Suppress all warnings
warnings.filterwarnings('ignore')

min_x = 0.19
max_x = 0.8
min_y = -0.61
max_y = 0.43       #0.6
min_z = 0.132
max_z = 0.6

requested_cPose = None
local_miniumum_flag = False

def _quat2vec(x, y, z, w):
    # Create a rotation object from rotation vector in radians
    rot = Rotation.from_quat([x, y, z, w])
    # Convert to quaternions and print
    rr = rot.as_rotvec()
    return rr[0], rr[1], rr[2]


def callback_rp(data):
    global requested_cPose
    if ((data.data[0] > min_x) and (data.data[0] < max_x) and (data.data[1] > min_y) and (data.data[1] < max_y) and (data.data[2] > min_z) and (data.data[2] < max_z)):
        requested_cPose = data.data
        print(data.data)
    else:
        print(f"invalid position passed into avoidance server: {data.data}") 
        print(f"invalid position passed into avoidance server: {data.data}") 

def callback_ag(data):
    global local_miniumum_flag
    local_miniumum_flag = bool(data.data[1])
    # print(local_miniumum_flag)

def plan_with_orientation_constraint():
    # Initialize ROS node
    rospy.init_node('moveit_orientation_constraint_example', anonymous=True)

    # Initialize the DisplayTrajectory publisher
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=20)

    sub_rp = rospy.Subscriber("/required_pos", Float32MultiArray, callback_rp)
    sub_ag = rospy.Subscriber("/active_avoidance_goal", Int8MultiArray, callback_ag)
    pub_rp = rospy.Publisher('/required_pos', Float32MultiArray, queue_size=1)

    

    
    # Initialize MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    
    # Initialize the MoveGroupCommander for the arm
    group = MoveGroupCommander("manipulator")  # Replace with your group name
    # Scale down the velocity and acceleration
    group.set_max_velocity_scaling_factor(1)  # 10% of maximum speed
    group.set_max_acceleration_scaling_factor(1)  # 10% of maximum acceleration.
    group.set_goal_position_tolerance(0.001)  # Set goal position tolerance

    group.set_planning_time(5)  # Example: Set to 10 seconds

    # Set the planner ID
    group.set_planner_id("RRTConnect")  # Example planner




    while not rospy.is_shutdown():

        # print(group.get_current_pose())
        # print(local_miniumum_flag)
        if (requested_cPose is not None) and (local_miniumum_flag):
            print("Planning new trajectory")

            pose = group.get_current_pose().pose

            desired_pos = requested_cPose[:3]


            # Define the target pose for the end effector
            target_pose = PoseStamped()
            target_pose.header.frame_id = "base_link"  # Reference frame
            target_pose.pose.orientation.x = pose.orientation.x
            target_pose.pose.orientation.y = pose.orientation.y
            target_pose.pose.orientation.z = pose.orientation.z
            target_pose.pose.orientation.w = pose.orientation.w
            target_pose.pose.position.x = -desired_pos[0]
            target_pose.pose.position.y = -desired_pos[1]
            target_pose.pose.position.z = desired_pos[2]
            group.set_pose_target(target_pose)

            print(group.get_planning_frame())

            # Define orientation constraint
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.frame_id = "base_link"  # Reference frame
            orientation_constraint.link_name = group.get_end_effector_link()
            orientation_constraint.orientation = target_pose.pose.orientation
            orientation_constraint.absolute_x_axis_tolerance = 0.01  # Set tolerances
            orientation_constraint.absolute_y_axis_tolerance = 0.01
            orientation_constraint.absolute_z_axis_tolerance = 0.01
            orientation_constraint.weight = 1.0  # Full weight to enforce the constraint

        
            # Create a constraints object and add the orientation constraint
            constraints = Constraints()
            constraints.orientation_constraints.append(orientation_constraint)
            
            # Set the path constraints for the group
            group.set_path_constraints(constraints)

            # Plan the motion with the constraint
            plan = group.plan()




            # Execute the plan
            if plan:

                traj = []
                prev_time  = 0
                for point in plan[1].joint_trajectory.points:
                    pose_quat = ur5e_arm.forward(point.positions)
                    vec = _quat2vec(pose_quat[3],pose_quat[4], pose_quat[5], pose_quat[6])

                    traj_req = Float32MultiArray()
                    traj_req.data = ([-pose_quat[0], -pose_quat[1], pose_quat[2], vec[1], -vec[0], vec[2]])
                    pub_rp.publish(traj_req)
                    traj.append([-pose_quat[0], -pose_quat[1], pose_quat[2], vec[1], -vec[0], vec[2]])
                    

                    new_time  = point.time_from_start.secs + float(point.time_from_start.nsecs/1000000000)
                    print(new_time - prev_time)
                    rospy.sleep((new_time - prev_time)/4)
                    prev_time = new_time
                print(traj)

                
                # Create a DisplayTrajectory message and fill it
                display_trajectory = DisplayTrajectory()
                display_trajectory.trajectory_start = group.get_current_state()
                display_trajectory.trajectory.append(plan[1])
                display_trajectory_publisher.publish(display_trajectory)

            else:
                print("couldnt generate plan")


if __name__ == '__main__':
    try:
        ur5e_arm = ur_kinematics.URKinematics('ur5e')
        plan_with_orientation_constraint()
    except rospy.ROSInterruptException:
        pass