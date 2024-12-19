import numpy as np
import rospy
from urdf_parser_py.urdf import URDF
import subprocess, tempfile, os

import rospy
import urdf_parser_py.urdf as urdf
import fcl
import numpy as np
from tf.transformations import quaternion_matrix, translation_matrix, concatenate_matrices
import trimesh

import rospy
from urdf_parser_py.urdf import URDF
import fcl
import numpy as np
from tf.transformations import quaternion_from_euler
from pathlib import Path
import rospkg
import trimesh
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion
import rospy
from std_msgs.msg import String, Float32MultiArray
import subprocess, tempfile, os
import rospy
import urdf_parser_py.urdf as urdf
import fcl
import numpy as np
from tf.transformations import quaternion_matrix, rotation_matrix, euler_matrix, euler_from_quaternion,  quaternion_from_matrix
import sys
import time
import tf2_ros
import tf2_geometry_msgs
from tf2_msgs.msg import TFMessage


min_x = 0.24
max_x = 0.8
min_y = -0.48
max_y = 0.43     #0.6
min_z = 0.25
max_z = 0.6


Joint_Names = {
    'base_link-base_link_inertia',
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint'
}

alt_joint_names = [
            "base_link",
            "shoulder_link",
            "upper_arm_link",
            "forearm_link",
            "wrist_1_link",
            "wrist_2_link",
            "wrist_3_link"
        ] 


def parse_xacro_content_to_urdf(xacro_content):
    with tempfile.NamedTemporaryFile(delete=False, suffix='.xacro') as temp_xacro_file:
        temp_xacro_file.write(xacro_content.encode())
        temp_xacro_file_path = temp_xacro_file.name
    
    urdf_file_path = temp_xacro_file_path.replace('.xacro', '.urdf')
    command = ["xacro", temp_xacro_file_path, "-o", urdf_file_path]
    subprocess.run(command, check=True)
    
    with open(urdf_file_path, 'r') as urdf_file:
        urdf_content = urdf_file.read()
    
    os.remove(temp_xacro_file_path)
    os.remove(urdf_file_path)
    
    return urdf_content


def resolve_package_path(package_path):
    package_name, relative_path = package_path.replace('package://', '').split('/', 1)
    rospack = rospkg.RosPack()
    package_path = rospack.get_path(package_name)
    full_path = Path(package_path) / relative_path
    return full_path

def check_file_accessibility(file_path):
    path = Path(file_path)
    if path.exists():
        if path.is_file():
            print(f"The path '{file_path}' exists and is a file.")
            return True
        else:
            print(f"The path '{file_path}' exists but is not a file.")
            return False
    else:
        print(f"The path '{file_path}' does not exist.")
        return False

def extract_collision_geometries(robot):
    geometries = []
    for link in robot.links:
        if link.collision:
            for collision in link.collisions:
                geometry = collision.geometry
                origin = collision.origin
                geometries.append((geometry, origin, link.name))
    return geometries

import numpy as np

def project_vector(A, B):
    """
    Projects vector A onto vector B.

    Args:
    A (numpy array): The vector to be projected.
    B (numpy array): The vector onto which A is projected.

    Returns:
    numpy array: The projection of A onto B.
    """
    # Convert to numpy arrays if not already
    A = np.array(A)
    B = np.array(B)
    
    # Calculate dot product of A and B
    dot_product = np.dot(A, B)
    
    # Calculate the magnitude squared of B
    B_magnitude_squared = np.dot(B, B)
    
    # Calculate the projection scalar
    projection_scalar = dot_product / B_magnitude_squared
    
    # Calculate the projection vector
    projection_vector = projection_scalar * B
    
    return projection_vector

# Example usage
A = np.array([3, 4, 5])
B = np.array([1, 0, 0])

projection = project_vector(A, B)
print("Projection of A onto B:", projection)


def create_transform_matrix(xyz, rpy):
    """
    Create a 4x4 homogeneous transformation matrix from XYZ translation and RPY rotation.
    
    :param xyz: A tuple or list of translation (x, y, z)
    :param rpy: A tuple or list of rotation (roll, pitch, yaw)
    :return: A 4x4 homogeneous transformation matrix
    """
    tx, ty, tz = xyz

    if(len(rpy) == 4):
        rpy = euler_from_quaternion(rpy)

    roll, pitch, yaw = rpy
    
    # Create rotation matrix from RPY
    R = euler_matrix(roll, pitch, yaw)
    
    # Initialize a 4x4 identity matrix
    T = np.eye(4)
    
    # Set the top-left 3x3 submatrix to the rotation matrix
    T[0:3, 0:3] = R[0:3, 0:3]
    
    # Set the top-right 3x1 submatrix to the translation vector
    T[0:3, 3] = [tx, ty, tz]
    
    return T

def rpy_to_rotation_matrix(roll, pitch, yaw):
    # Convert angles from degrees to radians if necessary
    roll, pitch, yaw = np.radians([roll, pitch, yaw])
    
    # Compute rotation matrices around x, y, z axes
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    # Combine rotations
    R = np.dot(R_z, np.dot(R_y, R_x))
    return R

def urdf_geometry_to_fcl(geometry):
    print(type(geometry))
    print(geometry.filename)
    file_path = geometry.filename
    full_path = resolve_package_path(file_path)
    if check_file_accessibility(full_path):
        print(f"File '{full_path}' is accessible.")
    else:
        print(f"File '{full_path}' is not accessible.")
    
    mesh = trimesh.load(full_path, process=False)

    faces = np.concatenate((3 * np.ones((len(mesh.faces), 1), dtype=np.int64), mesh.faces), axis=1).flatten()
    c = fcl.Convex(mesh.vertices, len(mesh.faces), faces)
    return c, full_path


def get_transform(tf_buffer, from_frame, to_frame):
    try:
        trans = tf_buffer.lookup_transform(to_frame, from_frame, rospy.Time(0), rospy.Duration(1.0))
        return trans
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("Could not transform %s to %s" % (from_frame, to_frame))
        return None
    
def generateMarker(mesh_resource, position, orientation, marker_id, type):
    marker = Marker()
    marker.header.frame_id = "base_link"  # Frame in which the marker is defined
    marker.header.stamp = rospy.Time.now()
    marker.ns = "my_namespace"
    marker.id = marker_id

    if type == "mesh":
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = "file://" + str(mesh_resource)
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
    elif type == "cylinder":
        marker.type = Marker.CYLINDER
        # Set the scale of the marker (height and diameter)
        marker.scale.x = mesh_resource[0]*2  # Diameter along the X-axis
        marker.scale.y = mesh_resource[0]*2  # Diameter along the Y-axis
        marker.scale.z = mesh_resource[1]  # Height of the cylinder

    marker.action = Marker.ADD
    marker.pose.position = Point(*position)
    marker.pose.orientation = Quaternion(*orientation)
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 0.5
    marker.lifetime = rospy.Duration()

    return marker



sphere_obstacle = None

def marker_callback(data):
    global sphere_obstacle

    sphere_obstacle = []

    for marker in data.markers:
        if marker.type == Marker.SPHERE:
            position = marker.pose.position
            orientation = marker.pose.orientation

            radius = marker.scale.x/2
            # print(position)


            s = fcl.Sphere(radius)
            t1=fcl.Transform([orientation.w, orientation.x, orientation.y, orientation.z], [position.x, position.y, position.z])
            # sphere_obstacle = fcl.CollisionObject(s, t1)
            sphere_obstacle.append(fcl.CollisionObject(s, t1))
        elif (marker.type == Marker.CYLINDER): 
            position = marker.pose.position
            orientation = marker.pose.orientation

            c = fcl.Cylinder((marker.scale.x/4), marker.scale.z)
            t1=fcl.Transform([orientation.w, orientation.x, orientation.y, orientation.z], [position.x, position.y, position.z])
            sphere_obstacle.append(fcl.CollisionObject(c, t1))
            



def publish_arrow(start_point, end_point, t, p, frame_id='base_link'):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "line_strip"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD

    # Define the points for the line strip
    points = [
        t,
        p,
        start_point,
        end_point
    ]
    marker.points = points

    # Define the scale and color of the line strip
    marker.scale.x = 0.005  # Line width

    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    marker.lifetime = rospy.Duration(0)  # 0 means the marker is persistent
    return marker



 

# sphere_obstacle = fcl.CollisionObject()
if __name__ == "__main__":
    
    # LOAD UR5E FROM ROS PARAM SERVER
    rospy.init_node('load_urdf_from_param_server')

    # Publishers for nearest obstacle data, and for debugging
    marker_pub = rospy.Publisher('robot_marker_array', MarkerArray, queue_size=1)
    boundary_pub = rospy.Publisher('boundary_obstacles', MarkerArray, queue_size=1)
    arrow_pub = rospy.Publisher('min_dist_c', MarkerArray, queue_size=1)
    dist_pub = rospy.Publisher('obs_dist', Float32MultiArray, queue_size=5)
    rospy.Subscriber('moving_sphere', MarkerArray, marker_callback)

    #intialises transform listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)


    #parses xacro content into a usable URDF for loading robot collision geometries
    xacro_file  = rospy.get_param('/robot_description')
    urdf_content = parse_xacro_content_to_urdf(xacro_file)

    robot = URDF.from_xml_string(urdf_content)

    rate = rospy.Rate(30)



#   imports collision geometres from links
    elements = extract_collision_geometries(robot)
    robot_shapes = []
    meshes = []
    link_transforms = []
    for (geometry, pose, link_name) in  elements:
        print(link_name)
        if(link_name == "ft300_sensor"):
            break
        shape, mesh = urdf_geometry_to_fcl(geometry)
        robot_shapes.append(shape)
        meshes.append(mesh)
        link_trans = create_transform_matrix(pose.xyz, pose.rpy)
        link_transforms.append(link_trans)
        # print(pose)
        # extract link transforms
    # Define the cylinder geometry
    gc_radius = 0.075  
    gc_height = 0.23
    gc = fcl.Cylinder(gc_radius, gc_height)

    count = 0
    robot_co = []

    # updates pose of each object
    for joint in robot.joints:
        if(joint.name in Joint_Names):
            trans = create_transform_matrix(joint.origin.xyz, joint.origin.rpy)
            new_transform = np.dot(trans, link_transforms[count])
            new_translation = new_transform[0:3, 3]
            new_rotation_matrix = new_transform[0:3, 0:3]
            tf = fcl.Transform(new_rotation_matrix, new_translation)
            robot_co.append(fcl.CollisionObject(robot_shapes[count], tf))
            count = count + 1
            # print(robot_co)

            # add cylinder object to end effector
            if (joint.name == 'wrist_3_joint'):
                robot_co.append(fcl.CollisionObject(gc, tf))

    # print(robot_co)

    # INITIALISES WORKSPACE OBSTACLES 
    boundary_obstacles= []
    boundary_Markers = MarkerArray()
    # create workspace boundary collision objects
    bench_geometry = fcl.Box(0.9, 1.666, 0.5)
    box_t = [-0.54, 0 ,-0.12-0.25]
    box_q = [0,0,0,1]
    bench_tranforms= fcl.Transform(box_q, box_t)
    bench  = fcl.CollisionObject(bench_geometry, bench_tranforms)
    boundary_obstacles.append(bench)

    bmark = Marker()
    bmark.header.frame_id = "base_link"  # Frame in which the marker is displayed
    bmark.header.stamp = rospy.Time.now()
    bmark.ns = "bench_marker"
    bmark.id = 0
    bmark.type = Marker.CUBE  # Type of the marker
    bmark.action = Marker.ADD
    bmark.pose.position.x = box_t[0]  # Position in X
    bmark.pose.position.y = box_t[1]  # Position in Y   
    bmark.pose.position.z = box_t[2]  # Position in Z
    bmark.pose.orientation.x = 0.0
    bmark.pose.orientation.y = 0.0
    bmark.pose.orientation.z = 0.0
    bmark.pose.orientation.w = 1.0
    bmark.scale.x = 0.9  # Width
    bmark.scale.y = 1.666  # Height
    bmark.scale.z = 0.5  # Depth
    bmark.color.r = 1  # Red
    bmark.color.g = 0  # Green
    bmark.color.b = 0  # Blue
    bmark.color.a = 0.25  # Alpha (opacity)
    bmark.lifetime = rospy.Duration(0)
    boundary_Markers.markers.append(bmark)


    lw_geometry = fcl.Box(0.9, 1, 1)
    box_t = [-0.54, 1.333 ,0.62]
    box_q = [0,0,0,1]
    lw_tranforms= fcl.Transform(box_q, box_t)
    lw  = fcl.CollisionObject(lw_geometry, lw_tranforms)
    boundary_obstacles.append(lw)

    bmark = Marker()
    bmark.header.frame_id = "base_link"  # Frame in which the marker is displayed
    bmark.header.stamp = rospy.Time.now()
    bmark.ns = "lw_marker"
    bmark.id = 0
    bmark.type = Marker.CUBE  # Type of the marker
    bmark.action = Marker.ADD
    bmark.pose.position.x = box_t[0]  # Position in X
    bmark.pose.position.y = box_t[1]  # Position in Y
    bmark.pose.position.z = box_t[2]  # Position in Z
    bmark.pose.orientation.x = 0.0
    bmark.pose.orientation.y = 0.0
    bmark.pose.orientation.z = 0.0
    bmark.pose.orientation.w = 1.0
    bmark.scale.x = 0.9  # Width
    bmark.scale.y = 1  # Height
    bmark.scale.z = 1  # Depth
    bmark.color.r = 1  # Red
    bmark.color.g = 0  # Green
    bmark.color.b = 0  # Blue
    bmark.color.a = 0.25  # Alpha (opacity)
    bmark.lifetime = rospy.Duration(0)
    boundary_Markers.markers.append(bmark)


    rw_geometry = fcl.Box(0.9, 1, 1)
    box_t = [-0.54, -1.2 ,0.62]
    box_q = [0,0,0,1]
    rw_tranforms= fcl.Transform(box_q, box_t)
    rw  = fcl.CollisionObject(rw_geometry, rw_tranforms)
    boundary_obstacles.append(rw)

    bmark = Marker()
    bmark.header.frame_id = "base_link"  # Frame in which the marker is displayed
    bmark.header.stamp = rospy.Time.now()
    bmark.ns = "rw_marker"
    bmark.id = 0
    bmark.type = Marker.CUBE  # Type of the marker
    bmark.action = Marker.ADD
    bmark.pose.position.x = box_t[0]  # Position in X
    bmark.pose.position.y = box_t[1]  # Position in Y
    bmark.pose.position.z = box_t[2]  # Position in Z
    bmark.pose.orientation.x = 0.0
    bmark.pose.orientation.y = 0.0
    bmark.pose.orientation.z = 0.0
    bmark.pose.orientation.w = 1.0
    bmark.scale.x = 0.9  # Width
    bmark.scale.y = 1  # Height
    bmark.scale.z = 1  # Depth
    bmark.color.r = 1  # Red
    bmark.color.g = 0  # Green
    bmark.color.b = 0  # Blue
    bmark.color.a = 0.25  # Alpha (opacity)
    bmark.lifetime = rospy.Duration(0)
    boundary_Markers.markers.append(bmark)


    # Proximity Checks to constantly update nearest obstaclle to robot arm
    while not rospy.is_shutdown():
        # start_time = time.perf_counter()
        base_link = "base_link"
        robot_markers = MarkerArray()
        new_transforms = []
        # Transforms robot arm to new new joint angles 
        for index, joint in enumerate(alt_joint_names):
            transform = get_transform(tf_buffer, joint, base_link)
            # updates pose in both rviz and FCL if transform now exists
            if transform:
                # rospy.loginfo("Transform from %s to %s: %s" % (joint, base_link, transform))
                # update fcl objects
                xyz = transform.transform.translation
                quart = transform.transform.rotation
                trans = np.array([xyz.x, xyz.y, xyz.z])
                q = [quart.x, quart.y, quart.z, quart.w]
                trans = create_transform_matrix(trans, q)
                new_transform = np.dot(trans, link_transforms[index])
                new_translation = new_transform[0:3, 3]
                new_rotation_matrix = new_transform[0:3, 0:3]
                new_transforms.append(new_translation)

                new_T = fcl.Transform(new_rotation_matrix, new_translation)
                robot_co[index].setTransform(new_T)
                new_q = quaternion_from_matrix(new_transform)
                # updata marker arrays for visualisation
                marker = generateMarker(meshes[index], new_translation, new_q, index, "mesh")

                robot_markers.markers.append(marker)


                if joint == 'wrist_3_link':
                    gc_offset = np.array([0, 0.215, 0])
                    rotated_offset = np.dot(new_rotation_matrix, gc_offset)
                    gc_t = new_translation + rotated_offset
                    # print(gc_t)
                    new_transforms.append(gc_t)

                    R_rotation = np.array([[0, 0, -1],
                       [0, 1, 0],
                       [1, 0, 0]])

                    gc_R = np.dot(new_rotation_matrix, R_rotation)
                    robot_co[index+1].setRotation(gc_R)
                    robot_co[index+1].setTranslation(gc_t)

                    marker = generateMarker([gc_radius, gc_height], gc_t, q, index+1, "cylinder")
                    robot_markers.markers.append(marker)
        # print("check")

        # print(new_transforms)
        marker_pub.publish(robot_markers)
        boundary_pub.publish(boundary_Markers)

        min_distance = 100
        closest_line = None
        link_index = 6
        near_boundary_flag = False
        # These thresholds should be the same as the distance thresholds in the potential function
        body_distance_threshold = 0.2
        workspace_distance_threshold = 0.1
        nearby_obstacles = []

        # Proximty Checks
        if sphere_obstacle:
            for obstacle in sphere_obstacle:
                for index, co in enumerate(robot_co):
                    # CHecks if robot is near person and updates vector
                    if(index>1):
                            request = fcl.DistanceRequest(enable_nearest_points=True)
                            result = fcl.DistanceResult()
                            ret = fcl.distance(robot_co[index], obstacle, request, result)
                            # print(result)
                            if(result.min_distance<min_distance):
                                min_distance = result.min_distance
                                closest_line = result.nearest_points
                                link_index = index
            if(min_distance<body_distance_threshold):
                nearby_obstacles.append([min_distance, closest_line, link_index, False])
        #checks if robot is near boundary obstacles and updates vector
        min_distance = 100
        closest_line = None
        link_index = 6

        for boundary_object in boundary_obstacles:
            for index,co in enumerate(robot_co):
                if(index>2):
                    request = fcl.DistanceRequest(enable_nearest_points=True)
                    result = fcl.DistanceResult()
                    ret = fcl.distance(robot_co[index], boundary_object, request, result)
                    # print(result.min_distance)
                # if(result.min_distance<min_distance):
                    if(result.min_distance < min_distance): #and (not near_boundary_flag):
                        min_distance = result.min_distance
                        closest_line = result.nearest_points
                        link_index = index
        if(min_distance<workspace_distance_threshold):
            near_boundary_flag = True
            nearby_obstacles.append([min_distance, closest_line, link_index, True])
        else:
            near_boundary_flag = False


        if not near_boundary_flag:
            print("Clear of WS")
        else:
            print("Near Boundary")
        # print(nearby_obstacles)

        v_pos = Float32MultiArray()
        v_pos.data =[]
        # obstacle_array = []
        all_arrows = MarkerArray()
        for (min_dist, closest_line, link_index, near_boundary_flag) in nearby_obstacles:
            # calculate the projected distance from joint
            # print(link_index)
            pn_v = 0
            if (link_index <=5):
                print(link_index)
                # rospy.sleep(1)
                normal_v = np.array(new_transforms[link_index+1]) - np.array(new_transforms[link_index])
                proj_v = np.array(closest_line[0]) - np.array(new_transforms[link_index])
                pn_v = project_vector(proj_v, normal_v)
                # print(normal_v)
                # print(proj_v)
                # print(pn_v)
                v_pos.data.append(float(link_index))
                print(np.linalg.norm(pn_v))
                v_pos.data.append(float(np.linalg.norm(pn_v)))
            else:
                if(link_index == 7):
                    # print("Link is 7")
                    normal_v = np.array(new_transforms[link_index]) - np.array(new_transforms[link_index-1])
                    proj_v = np.array(closest_line[0]) - np.array(new_transforms[link_index])
                    pn_v = project_vector(proj_v, normal_v)
                    v_pos.data.append(float(5))
                    v_pos.data.append(float(0.0996 + np.linalg.norm(pn_v)))
                else:
                    v_pos.data.append(float(5))
                    v_pos.data.append(float(0.0996))
            dist = ((closest_line[0] - closest_line[1]))
            for component in dist:
                v_pos.data.append(float(component))
            v_pos.data.append(float(int(near_boundary_flag)))
            arrow = publish_arrow(Point(*closest_line[0]), Point(*closest_line[1]), Point(*new_transforms[link_index]), Point(*new_transforms[link_index] + pn_v))
            all_arrows.markers.append(arrow)
        # arrow_pub.publish(all_arrows)
        
        dist_pub.publish(v_pos)
        # rate.sleep()

        # rospy.sleep(1)

# for index, joint in enumerate(alt_joint_names):