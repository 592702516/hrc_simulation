import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState

class Robotiq85Gripper:
    def __init__(self):
        rospy.Subscriber("/gripper_controller/state", JointTrajectoryControllerState, self._update_gripper_stat, queue_size=10)
        self._gripper_pub = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10)

        self._gripper_stat = JointTrajectoryControllerState()
        self._r = rospy.Rate(1)

    def _update_gripper_stat(self, stat):
        self._gripper_stat = stat

    def close(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = ['gripper_finger1_joint']  
        point = JointTrajectoryPoint()
        point.positions = [0.5]  # close
        point.time_from_start = rospy.Duration(2.0)  # time for excuse

        trajectory.points.append(point)
        trajectory.header.stamp = rospy.Time.now()
        self._gripper_pub.publish(trajectory)
        rospy.loginfo('close')
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
        rospy.loginfo('open')
        return True

    def get_stat(self):
        return self._gripper_stat

