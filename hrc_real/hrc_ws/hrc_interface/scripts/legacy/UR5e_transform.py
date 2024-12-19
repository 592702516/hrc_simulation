# Transforms the UR5e base coordinate frame w.r.t the global bench_frame

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler

if __name__ == '__main__':
    rospy.init_node('UR5e_tf_broadcaster')
    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        # Example transform from kinect to your desired frame
        t = TransformStamped()  
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'bench_frame'  # Frame published by Kinect driver
        t.child_frame_id = 'base_link'  # Your desired frame
        t.transform.translation.x = -0.09
        t.transform.translation.y = 0.812   #0.405
        t.transform.translation.z = 0.13

        roll = 0.0  # Rotation around X-axis (degrees)
        pitch = 0.0 # No rotation around Y-axis
        yaw = 3.14159   # Rotation around Z-axis (degrees)  #3.14159   
        quaternion = quaternion_from_euler(roll, pitch, yaw)


      # Set the rotation in the transform message
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]
        br.sendTransform(t)
        rate.sleep()

# 0, 0, 0.9999997, 0.0007963 

# [ 0, 0, 1, 0.0000013 ]
# [ 0, 0, 1, 0.0000013 ]


