import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState

class Robotiq85Gripper:
    def __init__(self):
        rospy.Subscriber("/gripper_controller/state", JointTrajectoryControllerState, self._update_gripper_stat, queue_size=10)
        self._gripper_pub = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10)

        self._gripper_stat = JointTrajectoryControllerState()
        self._r = rospy.Rate(1)
        self.open()

    def _update_gripper_stat(self, stat):
        self._gripper_stat = stat

    def close(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = ['gripper_finger1_joint']  # 修改为 URDF 或控制器中定义的实际关节名称
        point = JointTrajectoryPoint()
        point.positions = [0.5]  # close
        point.time_from_start = rospy.Duration(2.0)  # time for excuse

        trajectory.points.append(point)
        trajectory.header.stamp = rospy.Time.now()
        self._gripper_pub.publish(trajectory)
        rospy.loginfo('关闭命令已发送。')
        return True

    def open(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = ['gripper_finger1_joint']  
        point = JointTrajectoryPoint()
        point.positions = [0.0]  # open
        point.time_from_start = rospy.Duration(2.0)  # time for excuse

        trajectory.points.append(point)
        trajectory.header.stamp = rospy.Time.now()
        self._gripper_pub.publish(trajectory)
        rospy.loginfo('打开命令已发送。')
        return True

    def get_stat(self):
        return self._gripper_stat

