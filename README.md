# Iris Quadcopter Simulation
**Yuchen Wang**
*Northwestern University*


## Introduction
This main goal of this project is to simulate the behavior of an Iris quadcopter based on PX4 firmware in a Gazebo world. The quadcopter is supposed to accomplish an 3D motion planning. At first, the quad will be given a map containing all the obstacles and a destination point. It will use RRT algorithm to generate an optimized path to reach the goal and avoid collisions. The entire movement of the drone should be smooth, safe and robust.

## Background
[PX4](https://github.com/PX4/Firmware) is the autopilot control platfrom used in this project. It establishes the connection between ROS and Gazebo simulation via [MAVROS](https://github.com/mavlink/mavros).

*TO BE CONTINUE*