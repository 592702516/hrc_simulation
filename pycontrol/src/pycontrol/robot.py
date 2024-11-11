import rospy
import actionlib
import sys
import moveit_commander
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTrajectoryControllerState
import numpy as np
from geometry_msgs.msg import Pose

JOINT_NAMES=[
                'workbench_joint',
                'shoulder_pan_joint',
                'shoulder_lift_joint',
                'elbow_joint',
                'wrist_1_joint',
                'wrist_2_joint',
                'wrist_3_joint'
            ]
class UR5eRobot:
    def __init__(self):
        rospy.Subscriber("/ur5e_controller/state", JointTrajectoryControllerState, self._update_robot_stat, queue_size=10)
        self._robot_pub = rospy.Publisher('/ur5e_controller/command', JointTrajectory, queue_size=10)

        self._robot_stat = JointTrajectoryControllerState()
        self._r = rospy.Rate(1)

    def _update_robot_stat(self, stat):
        self._robot_stat = stat

    def execute_joint_trajectory(self, pose_list):

        if pose_list:
            for i, pose in enumerate(pose_list):
                if len(pose) == 7:

                    client = actionlib.SimpleActionClient(f'/ur5e_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
                    # Wait for action server to be ready
                    timeout = rospy.Duration(5)
                    if not client.wait_for_server(timeout):
                        rospy.logerr("Could not reach controller action server.")
                        sys.exit(-1)

                    # Create and fill trajectory goal
                    goal = FollowJointTrajectoryGoal()
                    goal.trajectory.joint_names = JOINT_NAMES

                    point = JointTrajectoryPoint()
                    point.positions = pose
                    point.time_from_start = rospy.Duration(1)
                    goal.trajectory.points.append(point)

                    client.send_goal(goal)
                    client.wait_for_result()

                    result = client.get_result()
                    rospy.loginfo("Rotating joints to position {} finished in state {}".format(pose, result.error_code))
                else:
                    rospy.logerr("Each action should have 6 elements, this one has {}".format(len(pose)))

        else:
            rospy.logerr("Action list is empty")
    
    def execute_cartesian_trajectory(self, pose_list):
        
        group.set_planner_id("RRTConnectConfigDefault")  
        group.set_planning_time(10)    
        if pose_list:  
            for i, pose in enumerate(pose_list):
                if len(pose) == 6:
                    target_pose = Pose()
                    target_pose.position.x = pose[0]
                    target_pose.position.y = pose[1]
                    target_pose.position.z = pose[2]
                    target_pose.orientation.x = pose[3]
                    target_pose.orientation.y = pose[4]
                    target_pose.orientation.z = pose[5]
                    waypoints = [target_pose]
                    (plan, fraction) = group.compute_cartesian_path(
                        waypoints,           
                        eef_step=0.02,
                        avoid_collisions=False, 
                    )       
                    
                    if fraction == 1.0:
                        rospy.loginfo(f"cartesian path planning success: {fraction}")
                    
                        result = group.execute(plan)
                    else:
                        rospy.logwarn(f"cartesian path planning unsucces: {fraction}")
                        return False    
        else:
            rospy.logerr("error")
    
    def go_home(self):
        home_pose = [0.4, 0.297, -0.132, 0.200, 2.231, -2.216, 0.0]
        client = actionlib.SimpleActionClient(f'/ur5e_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        # Wait for action server to be ready
        timeout = rospy.Duration(5)
        if not client.wait_for_server(timeout):
            rospy.logerr("Could not reach controller action server.")
            sys.exit(-1)

        # Create and fill trajectory goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = home_pose
        point.time_from_start = rospy.Duration(1)
        goal.trajectory.points.append(point)

        client.send_goal(goal)
        client.wait_for_result()

        result = client.get_result()
        rospy.loginfo("home already")


