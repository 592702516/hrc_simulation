# Transforms the camera coordinate frame w.r.t the global bench_frame

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler

if __name__ == '__main__':
    rospy.init_node('kinect_tf_broadcaster')
    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        # Example transform from kinect to your desired frame
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'bench_frame'  # Frame published by Kinect driver
        t.child_frame_id = 'depth_camera_link'  # Your desired frame
        t.transform.translation.x = 0.56 #0.566
        t.transform.translation.y = 1.036 # 1.022
        t.transform.translation.z = 1.223

        roll = -0.13 + 0.05 # Rotation around X-axis (degrees)
        pitch = 3.14159 - 0.05 # No rotation around Y-axis
        yaw = 1.57079  # Rotation around Z-axis (degrees)  1.57079
        quaternion = quaternion_from_euler(roll, pitch, yaw)


      # Set the rotation in the transform message
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        # print(quaternion)
        br.sendTransform(t)
        rate.sleep()

        # [ -0.0363443, 0.8782985, 0.4391479, -0.1855261 ]
        # [ 0.7056113, 0.7056158, -0.0459288, 0.0459304 ]





