# ROS Control
This project is used to control the UR5e + Linear rail completed by Industrial AI Research Team.
[![Watch the video](https://i.ytimg.com/vi/O54NXTF1nhs/maxresdefault.jpg?sqp=-oaymwEmCIAKENAF8quKqQMa8AEB-AH-CYAC0AWKAgwIABABGGUgYShXMA8=&rs=AOn4CLD-y80i6C_E7O8E1oOmRDOmnVWYMA)](https://www.youtube.com/watch?v=O54NXTF1nhs)

Follow the instruction step by step to build the project, system requirement: Ubuntu 20.04

## Clone the project
```bash
# create a catkin workspace
mkdir -p HRC_robot/src && cd HRC_robot/src

# clone the project noetic branch
git clone https://github.com/WanqingXia/HRC_robot.git .
```

## Dependencies
### For ur_ikfast package
Original from https://github.com/cambel/ur_ikfast

```bash
sudo apt-get install libblas-dev liblapack-dev

git clone https://github.com/cambel/ur_ikfast.git
cd ur_ikfast
pip install -e .
```

### For using Azure Kinect Sensor SDK
Original from https://gist.github.com/madelinegannon/c212dbf24fc42c1f36776342754d81bc

1.Add source using curl

```bash
sudo apt install curl
curl https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod
```

2.Modify `/etc/apt/sources.list`. At the bottom of the file, change from:

```bash
deb https://packages.microsoft.com/ubuntu/18.04/prod bionic main
# deb-src https://packages.microsoft.com/ubuntu/18.04/prod bionic main
```

to:

```bash
deb [arch=amd64] https://packages.microsoft.com/ubuntu/18.04/prod bionic main
# deb-src [arch=amd64] https://packages.microsoft.com/ubuntu/18.04/prod bionic main
```

3.Rerun `sudo apt-get update`

4.Install Kinect Packages

```bash
sudo apt install k4a-tools
sudo apt install libk4a1.4-dev
# sudo apt install libk4a1.3-dev
# sudo apt install libk4abt1.0-dev
# sudo apt install k4a-tools=1.3.0

```

5.Finish Device Setup

[Finish device setup](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md#linux-device-setup) by setting up udev rules:

- Copy '[scripts/99-k4a.rules](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/scripts/99-k4a.rules)' into '/etc/udev/rules.d/'.

```bash
sudo cp ~/HRC_robot/src/Azure_Kinect_ROS_Driver/99-k4a.rules /etc/udev/rules.d/99-k4a.rules
```
- Detach and reattach Azure Kinect devices if attached during this process.


6.Verify installation by call it directly in the terminal:
```bash
k4aviewer
```

### For Python packages
```bash
pip install -r requirements.txt

### For ROS packages

```bash
# install dependencies
sudo apt update -qq
rosdep update
cd /HRC_robot
rosdep install --from-paths src/Universal_Robots_ROS_Driver/ --ignore-src -y
rosdep install --from-paths src/fmauch_universal_robot/ --ignore-src -y
rosdep install --from-paths src/robotiq_85_gripper/ --ignore-src -y
```

## Building the project
```
# default building type is RelWithDebInfo, set in the toplevel CMakeList.txt
catkin build 

# activate the workspace (ie: source it)
echo 'source $HOME/HRC_robot/devel/setup.bash' >> ~/.bashrc 
source ~/.bashrc
```

## Run following commands in sequence
```bash
roslaunch azure_kinect_ros_driver driver.launch

roslaunch yolo_run yolov5.launch

roslaunch ur_robot_driver ur5e_bringup.launch

roslaunch robotiq_85_bringup robotiq_85.launch

roslaunch robotiq_ft_sensor ft_sensor.launch

roslaunch vention_conveyor_bringup vention_conveyor.launch 

rviz -d src/workbench/workbench_bringup/launch/combine.rviz

rosrun pycontrol ros_demo.py
```
## Run following commands in sequence for simulation
```
roslaunch ur_sim run_sim.launch

python3 python3 ~/HRC_robot/src/pycontrol/scripts/sim/detect_object_pick.py
```
## Note for hrc simulation
Ensure that the simulation environment correctly loads a red cube and that the camera can detect it properly. There are two camera topics available: `azure_kinect_camera_1`, which is mounted in the air, and `azure_kinect_camera`, mounted on the robot. When adapting real-world code for the simulation environment, double-check that the interface names match; in the simulation, `pycontrol` modules have a `_sim` suffix. You can disregard PID configuration as it isn’t an issue. Also, the green highlights are meant for the gripper plugin to aid in grasping; feel free to ignore these as well.

## Run following commands in sequence for HRC real pick 
```
cd

cd hrc_real

chmod +X /run_HRC

./run_HRC

python3 ~/hrc_interface\scripts\azure_full_body_publishing.py

python3 ~/hrc_interface\scripts\joint_position_movement.py

python3 ~/hrc_interface\scripts\hp_test.py

python3 ~/hrc_interface\scripts\global_planning.py

python3 ~/hrc_interface\scripts\test_moveit.py

python3 ~/hrc_interface\scripts\HRC_system.py
```
## Note for hrc real pick
The `./run_hrc.sh` script conveniently starts all necessary servers for operating the robot in the real-world setup. Open a new terminal window to launch RViz, and load the `hrc_scene.rviz` file within RViz to visualize the setup. The robot should now be ready for control via Python scripts. For more detailed information, refer to the "Running the HRC System" guide located in `~/HRC_robot/hrc_real/Running the HRC System`.


## Trouble shooting
1.Having trouble with communicate with robot

```bash
# disable firewall to communicate with robot
sudo ufw disable
```

2.Having trouble with controlling the gripper

```bash
# change permission for /tmp/ttyUR
sudo chmod -t /tmp
sudo chmod 777 /tmp/ttyUR
sudo chmod +t /tmp
```

3.Having trouble with controlling the Force/Torque sensor

```bash
# change permission for /dev/ttyUSB0
sudo chmod 777 /dev/ttyUSB0
```

## Installing
1.Config installation space

```bash
catkin config --install-space /opt/my_workspace
```

2.Normally build

```bash
catkin build
```

3.Activate the workspace (ie: source it)

```bash
echo 'source $HOME/HRC_robot/install/setup.bash' >> ~/.bashrc 
source ~/.bashrc
```
4.Make files executable

```bash
cd HRC_robot/install
sudo chmod -R 777 share/
```

