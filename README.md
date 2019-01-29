# Iris Quadcopter Simulation
**Yuchen Wang**

*Northwestern University*


## Introduction
This main goal of this project is to simulate the behavior of an Iris quadcopter based on PX4 firmware in a Gazebo world. The quadcopter is supposed to accomplish an 3D motion planning. At first, the quad will be given a map containing all the obstacles and a destination point. It will use RRT algorithm to generate an optimized path to reach the goal and avoid collisions. The whole motion of the drone should be smooth, safe and robust.

## Background
### PX4
[PX4](https://github.com/PX4/Firmware) is the autopilot control platfrom used in this project. It establishes the connection between ROS and Gazebo simulation via [MAVROS](https://github.com/mavlink/mavros). PX4 supports both Software In the Loop (SITL) simulation, where the flight stack runs on computer (either the same computer or another computer on the same network) and Hardware In the Loop (HITL) simulation using a simulation firmware on a real flight controller board.

### MAVROS--MAVLink
MAVROS is a ROS package which provides communication driver for various autopilots with MAVLink communication protocal. MAVROS can be used to communicate with any MAVLink enabled autopilot, and for this specific project it will be only used to enable communication between the PX4 flight stack and a ROS enabled companion computer.

### ROS with Gazebo
ROS can be used with PX4 and the Gazebo simulator. As the picture below indicates, PX4 communicates with Gazebo to receive sensor data from the simulated world and send motor and actuator values. It communicates with the GCS and ROS to send telemetry from the simulated environment and receive commands.
![flowchart](/img/px4_sitl_overview.png)

## Installation

*TO BE CONTINUE*