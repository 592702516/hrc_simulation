#!/bin/bash

# Open a new gnome-terminal window and run the first ROS launch file
# gnome-terminal -- bash -c "sleep 2; source /opt/ros/noetic/setup.bash; source ~/hrc/hrc_ws/devel/setup.bash; roslaunch ur_robot_driver ur5e_bringup.launch use_tool_communication:=true tool_voltage:=24 tool_baud_rate:=115200 tool_stop_bits:=1 tool_rx_idle_chars:=1.5 tool_tx_idle_chars:=3.5 tool_device_name:=/tmp/ttyUR robot_ip:=192.168.12.100 limited:=true \kinematics_config:=$HOME/my_robot_calibration.yaml; exec bash"

# # Open another new gnome-terminal window and run a different ROS launch file
# gnome-terminal -- bash -c "sleep 3; source /opt/ros/noetic/setup.bash; source ~/hrc/hrc_ws/devel/setup.bash; roslaunch azure_kinect_ros_driver driver.launch sensor_sn:=000283101212 depth_mode:=NFOV_2X2BINNED color_resolution:=1536P fps:=30 body_tracking_enabled:=true body_tracking_smoothing_factor:=0; exec bash"

# # # Open another new gnome-terminal window and run another ROS launch file
# gnome-terminal -- bash -c "sleep 4; source /opt/ros/noetic/setup.bash; source ~/hrc/hrc_ws/devel/setup.bash; roslaunch hrc_interface setup_coordinate_system.launch; exec bash"
# gnome-terminal -- bash -c "source /opt/ros/noetic/setup.bash; source ~/hrc/hrc_ws/devel/setup.bash; roslaunch hrc_interface run_UR_azure.launch; exec bash"

# sourec hrc/hrc_ws/devel/setup.bash

# echo "Launching UR driver and Azure Kinect driver with body tracking"

# roslaunch hrc_interface run_UR_azure.launch


# Open a new terminal to ask for user input and perform actions
gnome-terminal -- bash -c "
    echo 'Start external control in UR Pendant under URCap then press ENTER' && \
    read -p 'Input: ' && \
    sudo chmod -t /tmp && \
    sudo chmod 777 /tmp/ttyUR && \
    sudo chmod +t /tmp && \
    sudo chmod 777 /dev/ttyUSB0 && \
    roslaunch hrc_interface run_gripper_transform_rviz.launch && \
    exec bash
"

# Open a new terminal to launch ROS nodes
gnome-terminal -- bash -c "
    source /home/rui/352LAB/devel/setup.bash && \
    echo 'Launching UR driver and Azure Kinect driver with body tracking' && \
    roslaunch hrc_interface run_UR_azure.launch && \
    exec bash
"



