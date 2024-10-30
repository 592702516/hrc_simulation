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
from moveit_msgs.srv import GetMotionPlan, GetMotionPlanRequest

bridge = CvBridge()
depth_image = None
target_position = None
position_logged = False

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

def depth_callback(msg):
    global depth_image
    try:
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

def pixel_to_world(x, y):
    if depth_image is None:
        rospy.logwarn("Depth data not available.")
        return None, None, None

    if int(y) >= depth_image.shape[0] or int(x) >= depth_image.shape[1]:
        rospy.logwarn("Coordinates out of range.")
        return None, None, None

    depth_value = depth_image[int(y), int(x)]
    if np.isnan(depth_value) or depth_value <= 0:
        rospy.logwarn("Invalid depth value at given pixel coordinates.")
        return None, None, None

    fx, fy = 525.0, 525.0  # Example focal lengths
    cx, cy = 320.0, 240.0  # Example principal points

    camera_x = (x - cx) * depth_value / fx
    camera_y = (y - cy) * depth_value / fy
    camera_z = depth_value

    world_x = -camera_y + 0.5
    world_y = -camera_x
    world_z = 2.0684 - camera_z

    return world_x, world_y, world_z

def image_callback(msg):
    global target_position, position_logged
    if position_logged:
        return

    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return

    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 120, 70])
    upper_red = np.array([10, 255, 255])
    mask = cv2.inRange(hsv_image, lower_red, upper_red)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(c)

        if radius > 10:
            rospy.loginfo(f"Detected red cube at pixel coordinates ({x}, {y}), radius: {radius}")
            real_world_x, real_world_y, real_world_z = pixel_to_world(x, y)
            if real_world_x is not None:
                rospy.loginfo(f"Red cube world coordinates: ({real_world_x}, {real_world_y}, {real_world_z})")
                target_position = [real_world_x, real_world_y, real_world_z + 0.12]
                position_logged = True

def move_cartesian_path(group, waypoints, time_increment=1.0):
    # 确保 waypoints 是 geometry_msgs/Pose 类型的列表
    if not all(isinstance(waypoint, Pose) for waypoint in waypoints):
        rospy.logerr("所有 waypoints 的元素必须是 Pose 类型。")
        return False

    # 设置规划器和规划时间
    group.set_planner_id("RRTconnect")  # 使用 RRTstar 规划器
    group.set_planning_time(3)    # 设置规划时间为 10 秒

    # 规划笛卡尔路径，增加步长来减少路径点数量
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,            # 要跟随的 Pose 对象列表
        eef_step=0.1,
        avoid_collisions=False # 末端执行器的步长，单位为米
    )
    
    # 检查规划的成功率
    if fraction > 0.9:
        rospy.loginfo(f"cartesian path planning success: {fraction}")

        # 更新每个点的时间戳，确保 time_from_start 严格递增，增加时间跨度减少抖动
        for idx, point in enumerate(plan.joint_trajectory.points):
            point.time_from_start = rospy.Duration((idx + 1) * time_increment)

        # 执行轨迹
        result = group.execute(plan, wait=True)

    else:
        rospy.logwarn(f"cartesian path planning unsucces: {fraction}")
        return False

def main():
    rospy.init_node('test_ur5e_gripper_controllers')

    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("ur5e")

    group.set_max_velocity_scaling_factor(1)
    group.set_max_acceleration_scaling_factor(1)

    rospy.Subscriber('/azure_kinect_camera_1/color/image_raw', Image, image_callback)
    rospy.Subscriber('/azure_kinect_camera_1/depth/image_raw', Image, depth_callback)

    try:
        rospy.sleep(0.5)
        while target_position is None or depth_image is None:
            rospy.sleep(0.5)

        # 移动到目标位置上方
        target_position[2] += 0.15

        current_pose = group.get_current_pose().pose
        target_pose = Pose()
        target_pose.position.x = target_position[0]
        target_pose.position.y = target_position[1]
        target_pose.position.z = target_position[2]
        target_pose.orientation = current_pose.orientation

        waypoints = [current_pose, target_pose]
        move_cartesian_path(group, waypoints)

        rospy.sleep(0.5)

        # 向下移动以抓取
        target_position[2] -= 0.15
        target_pose.position.z = target_position[2]
        waypoints = [group.get_current_pose().pose, target_pose]

        move_cartesian_path(group, waypoints)

        rospy.sleep(0.5)

        # 关闭夹爪
        send_gripper_command(
            controller_name='gripper_controller',
            joint_names=['gripper_finger1_joint'],
            positions=[0.4],
            duration=1.0
        )

        rospy.sleep(1)

        # 抓取后抬高
        target_position[2] += 0.3
        target_pose.position.z = target_position[2]
        waypoints = [group.get_current_pose().pose, target_pose]

        move_cartesian_path(group, waypoints)

        rospy.sleep(0.5)

        # 打开夹爪
        send_gripper_command(
            controller_name='gripper_controller',
            joint_names=['gripper_finger1_joint'],
            positions=[0],
            duration=1.0
        )

    except rospy.ROSInterruptException:
        rospy.logerr('执行过程中断')

if __name__ == '__main__':
    main()
