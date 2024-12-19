import rospy
import numpy
import geometry_msgs.msg
from std_msgs.msg import Float64MultiArray, Float32MultiArray, Bool
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose
import time
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion, PointStamped
import tf2_ros
import tf2_geometry_msgs

import numpy as np
from collections import deque
import random  # Used here to simulate sensor data

import rospy
from visualization_msgs.msg import MarkerArray
from shape_msgs.msg import Mesh, MeshTriangle
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import numpy as np
from scipy.spatial import ConvexHull
import trimesh
import fcl
import math

maxdev_x = 0
maxdev_y = 0
maxdev_z = 0

body_points = {
    2,  #joint spine chest
    3, #joint _neck
    4, #joint_clavicle left
    5, #joint shoulder left
    6, #joint elbow left
    7, #joint wrist left
    8, #joint hand left
    # 9, #joint hand tip left
    10, #joint thumb left
    11, #joint clavicle right
    12, #joint shoulder right
    13, #joint elbow right
    14, #joint wrist right
    15, #joint hand right
    # 16, #joint handtip right
    17, #joint hand thumb right
    26, #joint head
}

chain_left_arm = {
    4,
    5,
    6,
    7,
    8,
    10,
}

chain_right_arm = {
    11,
    12,
    13,
    14,
    15,
    17
}

chain_middle = {
    2,
    3,
    26,

}

startTime = 0

ignore_hand = False

def transform_coords(x,y,z):
    # Transforms coordiantes according to camera calibration
    tx = x 
    ty = y 
    tz = z
    return [tx, ty, tz]

def extract_sphere_data(marker_array):
    spheres = []
    for marker in marker_array.markers:
        if marker.type == Marker.SPHERE:
            x, y, z = marker.pose.position.x, marker.pose.position.y, marker.pose.position.z
            radius = marker.scale.x / 2  # Assuming uniform scaling for sphere
            spheres.append((x, y, z, radius))
    return spheres

def create_bounding_sphere_mesh(spheres):
    centers = np.array([(x, y, z) for x, y, z, r in spheres])
    hull = ConvexHull(centers)
    vertices = hull.points
    faces = hull.simplices
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces)
    return mesh

def publish_mesh(mesh, frame_id):
    mesh_msg = Mesh()
    mesh_msg.header = Header(frame_id=frame_id, stamp=rospy.Time.now())
    mesh_msg.vertices = [Point(x, y, z) for x, y, z in mesh.vertices]
    mesh_msg.triangles = [MeshTriangle(*triangle) for triangle in mesh.faces]
    mesh_pub = rospy.Publisher('/mesh_topic', Mesh, queue_size=10)
    mesh_pub.publish(mesh_msg)

def marker_array_callback(marker_array_msg):
    spheres = extract_sphere_data(marker_array_msg)
    mesh = create_bounding_sphere_mesh(spheres)
    publish_mesh(mesh, 'base_frame')

    # Function to compute the quaternion representing the orientation of a cylinder
def get_orientation(start, end):
    direction = np.array(end) - np.array(start)
    norm_direction = direction / np.linalg.norm(direction)

    z_axis = np.array([0, 0, 1])  # Cylinder default orientation along Z-axis
    axis = np.cross(z_axis, norm_direction)
    angle = math.acos(np.dot(z_axis, norm_direction))
    
    # Quaternion representing rotation around `axis` by `angle`
    qx = axis[0] * math.sin(angle / 2)
    qy = axis[1] * math.sin(angle / 2)
    qz = axis[2] * math.sin(angle / 2)
    qw = math.cos(angle / 2)
    return (qx, qy, qz, qw)


def joint_coordinates_callback(data):
    global startTime
    global maxdev_x, maxdev_y, maxdev_z
    global ignore_hand
    # print(ignore_hand)
    new_marker_array = MarkerArray()
    # global joint_coords
    if data.markers:

        left_arm_positions = []
        right_arm_positions = []
        middle_positions = []

        for original_marker in data.markers:
            if ((original_marker.id%100) in body_points):
                if not(ignore_hand and ((original_marker.id%100 == 17) or (original_marker.id%100 == 15))):
                    # Create a PointStamped message for the input point
                    input_point = PointStamped()
                    input_point.header.frame_id = 'depth_camera_link'  # Replace with your source frame
                    input_point.header.stamp = rospy.Time.now()
                    input_point.point.x = original_marker.pose.position.x
                    input_point.point.y = original_marker.pose.position.y
                    input_point.point.z = original_marker.pose.position.z
                    try:
                        # Get the transform from source_frame to target_frame
                        transform = tf_buffer.lookup_transform('base_link', input_point.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
                        
                        # Transform the point to the target frame
                        transformed_point = tf2_geometry_msgs.do_transform_point(input_point, transform)

                        if((original_marker.id%100) in chain_left_arm):
                            left_arm_positions.append([transformed_point.point.x, transformed_point.point.y, transformed_point.point.z])

                        if((original_marker.id%100) in chain_right_arm):
                            right_arm_positions.append([transformed_point.point.x, transformed_point.point.y, transformed_point.point.z])

                        if((original_marker.id%100) in chain_middle):
                            middle_positions.append([transformed_point.point.x, transformed_point.point.y, transformed_point.point.z])

                        # Create a Marker message
                        marker = Marker()
                        marker.header.frame_id = 'base_link'
                        marker.header.stamp = rospy.Time.now()
                        marker.ns = "basic_shapes"
                        marker.id = (original_marker.id%100) 
                        marker.type = Marker.SPHERE
                        marker.action = Marker.ADD
                        marker.lifetime = rospy.Duration(0.5)

                                # Set the pose of the marker
                        marker.pose.position.x = transformed_point.point.x
                        marker.pose.position.y = transformed_point.point.y
                        marker.pose.position.z = transformed_point.point.z
                        marker.pose.orientation.x = 0.0
                        marker.pose.orientation.y = 0.0
                        marker.pose.orientation.z = 0.0
                        marker.pose.orientation.w = 1.0
                        
                        # Set the scale of the marker
                        marker.scale.x = 0.05
                        marker.scale.y = 0.05
                        marker.scale.z = 0.05
                        
                        # Set the color of the marker
                        marker.color.a = 1.0  # Alpha (opacity)
                        marker.color.r = 1.0  # Red
                        marker.color.g = 0.0  # Green
                        marker.color.b = 0.0  # Blue

                        # # Publish the marker
                        new_marker_array.markers.append(marker)

                    except (tf2_ros.TransformException, tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                        rospy.logerr(f"Transform error: {e}")

        # Create cylindrical links between spheres
        for i in range(len(left_arm_positions) - 1):
            start = left_arm_positions[i]
            end = left_arm_positions[i + 1]
            
            # Compute midpoint
            mid_point = [(start[0] + end[0]) / 2, (start[1] + end[1]) / 2, (start[2] + end[2]) / 2]
            
            # Compute length of the cylinder
            length = math.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2 + (end[2] - start[2]) ** 2)
            
            # Get orientation quaternion
            orientation = get_orientation(start, end)
            
            # Create cylinder marker
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.type = Marker.CYLINDER
            marker.lifetime = rospy.Duration(0.5)
            marker.id = (len(left_arm_positions) + i + 200) * 2
            marker.pose.position.x = mid_point[0]
            marker.pose.position.y = mid_point[1]
            marker.pose.position.z = mid_point[2]
            marker.pose.orientation.x = orientation[0]
            marker.pose.orientation.y = orientation[1]
            marker.pose.orientation.z = orientation[2]
            marker.pose.orientation.w = orientation[3]
            marker.scale.x = 0.05  # Cylinder radius
            marker.scale.y = 0.05
            marker.scale.z = length  # Cylinder length
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            new_marker_array.markers.append(marker)

                    # Create cylindrical links between spheres
        for i in range(len(right_arm_positions) - 1):
            start = right_arm_positions[i]
            end = right_arm_positions[i + 1]
            
            # Compute midpoint
            mid_point = [(start[0] + end[0]) / 2, (start[1] + end[1]) / 2, (start[2] + end[2]) / 2]
            
            # Compute length of the cylinder
            length = math.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2 + (end[2] - start[2]) ** 2)
            
            # Get orientation quaternion
            orientation = get_orientation(start, end)
            
            # Create cylinder marker
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.type = Marker.CYLINDER
            marker.lifetime = rospy.Duration(0.5)
            marker.id = (len(right_arm_positions) + i+100) * 2 
            marker.pose.position.x = mid_point[0]
            marker.pose.position.y = mid_point[1]
            marker.pose.position.z = mid_point[2]
            marker.pose.orientation.x = orientation[0]
            marker.pose.orientation.y = orientation[1]
            marker.pose.orientation.z = orientation[2]
            marker.pose.orientation.w = orientation[3]
            marker.scale.x = 0.05  # Cylinder radius
            marker.scale.y = 0.05
            marker.scale.z = length  # Cylinder length
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            new_marker_array.markers.append(marker)


                    # Create cylindrical links between spheres
        for i in range(len(middle_positions) - 1):
            start = middle_positions[i]
            end = middle_positions[i + 1]
            
            # Compute midpoint
            mid_point = [(start[0] + end[0]) / 2, (start[1] + end[1]) / 2, (start[2] + end[2]) / 2]
            
            # Compute length of the cylinder
            length = math.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2 + (end[2] - start[2]) ** 2)
            
            # Get orientation quaternion
            orientation = get_orientation(start, end)
            
            # Create cylinder marker
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.type = Marker.CYLINDER
            marker.lifetime = rospy.Duration(0.5)
            marker.id = (len(middle_positions) + i+100) * 2 
            marker.pose.position.x = mid_point[0]
            marker.pose.position.y = mid_point[1]
            marker.pose.position.z = mid_point[2]
            marker.pose.orientation.x = orientation[0]
            marker.pose.orientation.y = orientation[1]
            marker.pose.orientation.z = orientation[2]
            marker.pose.orientation.w = orientation[3]
            marker.scale.x = 0.05  # Cylinder radius
            marker.scale.y = 0.05
            marker.scale.z = length  # Cylinder length
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            new_marker_array.markers.append(marker)


    sphere_publisher.publish(new_marker_array)
def listener():
    global tf_buffer, tf_listener
  
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    rospy.spin()

def ih_cb(data):
    global ignore_hand
    ignore_hand = data.data
    # print("recieved_msg")

    
if __name__ == '__main__':
    # global startTime
    # startTime = time.perf_counter()
    rospy.init_node('body_tracking_transform', anonymous=True)
    rospy.Subscriber('body_tracking_data', MarkerArray, joint_coordinates_callback)
    rospy.Subscriber('ignore_hand', Bool, ih_cb)
    tf_buffer = None
    tf_listener = None
    sphere_publisher = rospy.Publisher('/moving_sphere', MarkerArray, queue_size=5)    

    listener()

    

