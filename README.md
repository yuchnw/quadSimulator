# Iris Quadcopter Simulation
**Yuchen Wang**

*Northwestern University*


## Introduction
This main goal of this project is to simulate the behavior of an Iris quadcopter based on PX4 firmware in a Gazebo world. The quadcopter is supposed to accomplish an 3D motion planning. At first, the quad will be given a map containing all the obstacles and a destination point. It will use RRT algorithm to generate an optimized path to reach the goal and avoid collisions. The whole motion of the drone should be smooth, safe and robust.

## Background
### PX4
[PX4](https://github.com/PX4/Firmware) is the autopilot control platfrom used in this project. It establishes the connection between ROS and Gazebo simulation via MAVROS. PX4 supports both Software In the Loop (SITL) simulation, where the flight stack runs on computer (either the same computer or another computer on the same network) and Hardware In the Loop (HITL) simulation using a simulation firmware on a real flight controller board.

### MAVROS--MAVLink
[MAVROS](https://github.com/mavlink/mavros) is a ROS package which provides communication driver for various autopilots with MAVLink communication protocal. MAVROS can be used to communicate with any MAVLink enabled autopilot, and for this specific project it will be only used to enable communication between the PX4 flight stack and a ROS enabled companion computer.

### ROS with Gazebo
ROS can be used with PX4 and the Gazebo simulator. As the picture below indicates, PX4 communicates with Gazebo to receive sensor data from the simulated world and send motor and actuator values. It communicates with the GCS and ROS to send telemetry from the simulated environment and receive commands.

![flowchart](/img/px4_sitl_overview.png)

## Installation
### Prerequisite
The system should have ROS(*Indigo, Kinetic, Lunar or Melodic*), relative workspace and Gazebo(*8 or later*) installed.

### Install PX4 Firmware
Fork from PX4 [Git repo](https://github.com/PX4/Firmware) and clone it to local working directory. Build the platform with following commands:
```bash
cd <Firmware_clone>
make px4_sitl_default gazebo
source ~/catkin_ws/devel/setup.bash    // (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 mavros_posix_sitl.launch
```
It will launch both SITL and MAVROS in ROS.

### Install MAVROS Package
:bangbang: **In case of getting errors about missing dependencies, run `rosdep update` often.** :bangbang:

Build the package with following commands:
```bash
cd ~/catkin_ws
# 1. Install MAVLink:
rosinstall_generator --rosdistro melodic mavlink | tee /tmp/mavros.rosinstall

# 2. Install MAVROS:
rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall

# 3. Create workspace & dependencies:
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src -j4
rosdep install --from-paths src --ignore-src -y

# 4. Install GeographicLib datasets:
./src/mavros/mavros/scripts/install_geographiclib_datasets.sh

# 5. Build source
catkin build
source devel/setup.bash
```
:bangbang: **MAVROS only support `catkin build` instead of `catkin_make` otherwise it will raise an error saying "Workspace contains non-catkin packages".** :bangbang:

*TO BE CONTINUE*