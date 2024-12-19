cmake_minimum_required(VERSION 3.0.2)
project(hrc_interface)

# Find catkin and any other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  visualization_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  actionlib
  controller_manager_msgs
  control_msgs
  trajectory_msgs
)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS rospy std_msgs visualization_msgs geometry_msgs tf2_ros tf2_geometry_msgs actionlib controller_manager_msgs control_msgs trajectory_msgs
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/hp_test.py
  scripts/azure_full_body_publishing.py
  scripts/joint_position_movement.py
  scripts/UR5EJacobian.py
  scripts/legacy/camera_transform.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
