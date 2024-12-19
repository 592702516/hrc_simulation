#!/usr/bin/env python

import rospy
from visualization_msgs.msg import MarkerArray
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from moveit_commander import PlanningSceneInterface, RobotCommander
import moveit_commander
from moveit_commander import PlanningSceneInterface, RobotCommander, MoveGroupCommander
from moveit_commander import RobotCommander, PlanningSceneInterface, AllowedCollisionEntry
import sys
from geometry_msgs.msg import PoseStamped


def marker_array_callback(msg):
    # Create a PlanningScene message
    planning_scene = PlanningScene()
    planning_scene.is_diff = True  # Set is_diff flag to True

    # Iterate through existing markers and mark them for removal
    for marker in msg.markers:
        collision_object = CollisionObject()
        collision_object.header = marker.header
        collision_object.id = "marker_" + str(marker.id)
        collision_object.operation = CollisionObject.REMOVE

        # Add CollisionObject to PlanningScene
        planning_scene.world.collision_objects.append(collision_object)

    # Publish PlanningScene to remove existing collision objects
    publish_planning_scene(planning_scene)

    # Clear the list of collision objects for the next update
    planning_scene.world.collision_objects = []

    # Iterate through new markers and add them as collision objects
    for marker in msg.markers:
        # Extract pose and dimensions from marker
        pose = marker.pose
        dimensions = [marker.scale.x, marker.scale.y, marker.scale.z]

        # Create CollisionObject
        collision_object = CollisionObject()
        collision_object.header = marker.header
        collision_object.id = "marker_" + str(marker.id)
        collision_object.operation = CollisionObject.ADD    
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = dimensions
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)

        # Set frame ID relative to which objects will be published
        collision_object.header.frame_id = "base_link"  # Change to your desired frame ID

        # Add CollisionObject to PlanningScene
        planning_scene.world.collision_objects.append(collision_object)

    # Publish PlanningScene to add new collision objects
    publish_planning_scene(planning_scene)

def publish_planning_scene(planning_scene):
    try:
        rospy.wait_for_service('/apply_planning_scene')
        apply_planning_scene = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
        response = apply_planning_scene(planning_scene)
        rospy.loginfo("Planning scene updated successfully!")
    except rospy.ServiceException as e:
        rospy.logerr("Failed to update planning scene: %s", str(e))

if __name__ == '__main__':
    rospy.init_node('marker_to_collision_object', anonymous=True)

    # Initialize MoveIt Commander
    moveit_commander.roscpp_initialize(sys.argv)
    
    # Initialize Robot Commander and Planning Scene Interface
    robot = RobotCommander()
    scene = PlanningSceneInterface()


    # Define the bitmask for components you want to retrieve
    # This is an integer where each bit represents a component
    # For example, WORLD_OBJECT = 1, ROBOT_STATE = 2, and so on
    COMPONENTS_WORLD_OBJECT = 1
    COMPONENTS_ROBOT_STATE = 2
    COMPONENTS_ATTACHED_OBJECTS = 4

    components = COMPONENTS_WORLD_OBJECT | COMPONENTS_ROBOT_STATE | COMPONENTS_ATTACHED_OBJECTS
    
    # Get the planning scene
    planning_scene = scene.get_planning_scene(0)
    print(planning_scene)


    test = AllowedCollisionEntry()
    test.enabled = [False, False, False, False, False, False, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True]


    planning_scene.allowed_collision_matrix.entry_names.append("ft300_mounting_plate")
    planning_scene.allowed_collision_matrix.entry_names.append("gripper_finger1_inner_knuckle_link")
    planning_scene.allowed_collision_matrix.entry_names.append("gripper_finger1_finger_tip_link")
    planning_scene.allowed_collision_matrix.entry_names.append("gripper_finger1_finger_link")
    planning_scene.allowed_collision_matrix.entry_names.append("gripper_finger1_knuckle_link")
    planning_scene.allowed_collision_matrix.entry_names.append("gripper_finger2_inner_knuckle_link")
    planning_scene.allowed_collision_matrix.entry_names.append("gripper_finger2_finger_tip_link")
    planning_scene.allowed_collision_matrix.entry_names.append("gripper_finger2_finger_link")
    planning_scene.allowed_collision_matrix.entry_names.append("gripper_finger2_knuckle_link")
    planning_scene.allowed_collision_matrix.entry_names.append("gripper_base_link")
    planning_scene.allowed_collision_matrix.entry_names.append("camera_visor")
    planning_scene.allowed_collision_matrix.entry_names.append("camera_base")
    planning_scene.allowed_collision_matrix.entry_names.append("ft300_sensor")
    planning_scene.allowed_collision_matrix.entry_names.append("camera_body")
    planning_scene.allowed_collision_matrix.entry_names.append("gripper_coupler_link")


    print(planning_scene.allowed_collision_matrix.entry_values)
    for entry_value in planning_scene.allowed_collision_matrix.entry_values:
        entry_value.enabled.append(True)
        entry_value.enabled.append(False)
        entry_value.enabled.append(False)
        entry_value.enabled.append(False)
        entry_value.enabled.append(False)
        entry_value.enabled.append(False)
        entry_value.enabled.append(False)
        entry_value.enabled.append(False)
        entry_value.enabled.append(False)
        entry_value.enabled.append(False)
        entry_value.enabled.append(False)
        entry_value.enabled.append(False)
        entry_value.enabled.append(False)
        entry_value.enabled.append(False)
        entry_value.enabled.append(False)

    
    planning_scene.allowed_collision_matrix.entry_values.append(test)
    planning_scene.allowed_collision_matrix.entry_values.append(test)
    planning_scene.allowed_collision_matrix.entry_values.append(test)
    planning_scene.allowed_collision_matrix.entry_values.append(test)
    planning_scene.allowed_collision_matrix.entry_values.append(test)
    planning_scene.allowed_collision_matrix.entry_values.append(test)
    planning_scene.allowed_collision_matrix.entry_values.append(test)
    planning_scene.allowed_collision_matrix.entry_values.append(test)
    planning_scene.allowed_collision_matrix.entry_values.append(test)
    planning_scene.allowed_collision_matrix.entry_values.append(test)
    planning_scene.allowed_collision_matrix.entry_values.append(test)
    planning_scene.allowed_collision_matrix.entry_values.append(test)
    planning_scene.allowed_collision_matrix.entry_values.append(test)
    planning_scene.allowed_collision_matrix.entry_values.append(test)
    planning_scene.allowed_collision_matrix.entry_values.append(test)
    print(planning_scene)
    scene.apply_planning_scene(planning_scene)


    # Define Bench
    box_pose = PoseStamped()
    box_pose.header.frame_id = "base_link"  # Reference frame
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = -0.54  # x-coordinate of the box center
    box_pose.pose.position.y = 0.0  # y-coordinate of the box center
    box_pose.pose.position.z = -0.07 - 0.05  # z-coordinate of the box center

    # Define the box dimensions
    box_size = (0.9, 1.666, 0.05)  # (length, width, height)

    # Add the box to the planning scene
    scene.add_box("Bench", box_pose, size=box_size)


    # Define LW
    box_pose = PoseStamped()
    box_pose.header.frame_id = "base_link"  # Reference frame
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = -0.54  # x-coordinate of the box center
    box_pose.pose.position.y = 1.333  # y-coordinate of the box center
    box_pose.pose.position.z = 0.62  # z-coordinate of the box center

    # Define the box dimensions
    box_size = (0.9, 1, 1)  # (length, width, height)

    # Add the box to the planning scene
    scene.add_box("LW", box_pose, size=box_size)

    # Define RW
    box_pose = PoseStamped()
    box_pose.header.frame_id = "base_link"  # Reference frame
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = -0.54  # x-coordinate of the box center
    box_pose.pose.position.y = -1.2  # y-coordinate of the box center
    box_pose.pose.position.z = 0.62  # z-coordinate of the box center

    # Define the box dimensions
    box_size = (0.9, 1, 1)  # (length, width, height)

    # Add the box to the planning scene
    scene.add_box("RW", box_pose, size=box_size)


    # Define TW
    box_pose = PoseStamped()
    box_pose.header.frame_id = "base_link"  # Reference frame
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = -0.54  # x-coordinate of the box center
    box_pose.pose.position.y = 0.0  # y-coordinate of the box center
    box_pose.pose.position.z = 0.8  # z-coordinate of the box center

    # Define the box dimensions
    box_size = (0.9, 1.666, 0.05)  # (length, width, height)

    # Add the box to the planning scene
    scene.add_box("TW", box_pose, size=box_size)


    # Define BW
    box_pose = PoseStamped()
    box_pose.header.frame_id = "base_link"  # Reference frame
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0.32-0.05  # x-coordinate of the box center
    box_pose.pose.position.y = 0.0  # y-coordinate of the box center
    box_pose.pose.position.z = 0.5  # z-coordinate of the box center

    # Define the box dimensions
    box_size = (0.05, 1.666, 1)  # (length, width, height)

    # Add the box to the planning scene
    scene.add_box("BW", box_pose, size=box_size)



    rospy.Subscriber('/moving_sphere', MarkerArray, marker_array_callback)
    rospy.spin()
