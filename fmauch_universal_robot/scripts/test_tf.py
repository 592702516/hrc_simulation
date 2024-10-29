import rospy
import sys
import actionlib
import moveit_commander
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import Float64
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def send_trajectory(controller_name, joint_names, positions, duration=5.0):
    client = actionlib.SimpleActionClient(f'/{controller_name}/follow_joint_trajectory', FollowJointTrajectoryAction)
    rospy.loginfo(f'Waiting for {controller_name} action server...')
    client.wait_for_server()
    rospy.loginfo(f'{controller_name} action server is up. Sending trajectory...')

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = joint_names

    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = rospy.Duration(duration)

    goal.trajectory.points.append(point)
    goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)

    client.send_goal(goal)
    client.wait_for_result()

    if client.get_result():
        rospy.loginfo(f'{controller_name} executed successfully.')
    else:
        rospy.logwarn(f'{controller_name} did not execute successfully.')

def send_gripper_command(controller_name, joint_names, positions, duration=2.0):
    client = actionlib.SimpleActionClient(f'/{controller_name}/follow_joint_trajectory', FollowJointTrajectoryAction)
    rospy.loginfo(f'Waiting for {controller_name} action server...')
    client.wait_for_server()
    rospy.loginfo(f'{controller_name} action server is up. Sending trajectory...')

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = joint_names

    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = rospy.Duration(duration)

    goal.trajectory.points.append(point)
    goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)

    client.send_goal(goal)
    client.wait_for_result()

    if client.get_result():
        rospy.loginfo(f'{controller_name} executed successfully.')
    else:
        rospy.logwarn(f'{controller_name} did not execute successfully.')

def get_end_effector_position():
    group = moveit_commander.MoveGroupCommander("ur5e")
    current_pose = group.get_current_pose().pose
    rospy.loginfo(f"End-Effector Position: x={current_pose.position.x}, y={current_pose.position.y}, z={current_pose.position.z}")
    rospy.loginfo(f"End-Effector Orientation: x={current_pose.orientation.x}, y={current_pose.orientation.y}, z={current_pose.orientation.z}, w={current_pose.orientation.w}")
    print(f"End-Effector Position: x={current_pose.position.x}, y={current_pose.position.y}, z={current_pose.position.z}")
    print(f"End-Effector Orientation: x={current_pose.orientation.x}, y={current_pose.orientation.y}, z={current_pose.orientation.z}, w={current_pose.orientation.w}")

def compute_inverse_kinematics(target_position):
    group = moveit_commander.MoveGroupCommander("ur5e")
    target_pose = group.get_current_pose().pose
    target_pose.position.x = target_position[0]
    target_pose.position.y = target_position[1]
    target_pose.position.z = target_position[2]

    # Set the pose target with increased tolerance
    group.set_goal_position_tolerance(0.05)
    group.set_goal_orientation_tolerance(0.1)
    group.set_pose_target(target_pose)

    # Plan the trajectory to the target pose
    rospy.loginfo("Planning to target pose...")
    success, plan, planning_time, error_code = group.plan()
    rospy.loginfo(f"Planning result: success={success}, planning_time={planning_time}, error_code={error_code}")

    if success and plan.joint_trajectory.points:
        joint_positions = plan.joint_trajectory.points[-1].positions
        rospy.loginfo(f"Computed joint positions: {joint_positions}")
        print(f"Computed joint positions: {joint_positions}")
        return joint_positions
    else:
        rospy.logwarn("No valid joint positions found for the given target position.")
        return None

def main():
    rospy.init_node('test_ur5e_gripper_controllers')

    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("ur5e")

    try:
        target_position = [0.2,0.2,1.5]
        joint_positions = compute_inverse_kinematics(target_position)

        if joint_positions:
            # Move the robot to the computed joint positions
            send_trajectory(
                controller_name='ur5e_controller',
                joint_names=[
                    'workbench_joint',
                    'shoulder_pan_joint',
                    'shoulder_lift_joint',
                    'elbow_joint',
                    'wrist_1_joint',
                    'wrist_2_joint',
                    'wrist_3_joint'
                ],
                positions=joint_positions,
                duration=2.0
            )

            # Adding a delay before testing the gripper
            time.sleep(2)
        

            # Close the gripper to pick up the cube
            send_gripper_command(
                controller_name='gripper_controller',
                joint_names=['gripper_finger1_joint'],
                positions=[0.4],  # Assuming the gripper is closing
                duration=1.0
            )
                       
            # Testing Gripper controller using action client
            time.sleep(1)
            send_gripper_command(
                controller_name='gripper_controller',
                joint_names=['gripper_finger1_joint'],
                positions=[0],  # Assuming the gripper is opening
                duration=1.0
            )

    except rospy.ROSInterruptException:
        rospy.logerr('Interrupted during execution')

if __name__ == '__main__':
    main()

