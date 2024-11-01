import rospy
import sys
import actionlib
import moveit_commander
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import Float64
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Pose, PointStamped
from moveit_commander import MoveGroupCommander

target_position = None

def move_cartesian_path(group, waypoints):
    # 设置规划器和规划时间
    group.set_planner_id("RRTConnectConfigDefault")  # 使用 RRTConnect 规划器
    group.set_planning_time(5)  # 设置规划时间为 10 秒


    # 规划笛卡尔路径，增加步长来减少路径点数量
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,            # 要跟随的 Pose 对象列表
        eef_step=0.05,
        avoid_collisions=False,  # 末端执行器的步长，单位为米
    )

    # 检查规划的成功率
    if fraction == 1.0:
        rospy.loginfo(f"Cartesian path planning success: {fraction}")
        # 执行轨迹
        group.execute(plan)
    else:
        rospy.logwarn(f"Cartesian path planning incomplete: {fraction}")
        return False

def main():
    rospy.init_node('test_ur5e_gripper_controllers')

    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("ur5e")

    try:
        target_position = [0.5, -0.4, 1.5]

        # 设置目标姿态
        target_pose = Pose()
        target_pose.position.x = target_position[0]
        target_pose.position.y = target_position[1]
        target_pose.position.z = target_position[2]
        target_pose.orientation = group.get_current_pose().pose.orientation  # 保持当前姿态

        # 创建路点列表，不包含初始位置
        waypoints = [target_pose]

        # 调用笛卡尔路径规划
        move_cartesian_path(group, waypoints)


       

    except rospy.ROSInterruptException:
        rospy.logerr('执行过程中断')

if __name__ == '__main__':
    main()


