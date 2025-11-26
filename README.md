# 🛸 HMW3 - Fly your drone

## 🧩 Overview
This projects implements Homework 3 of the Robotics Lab 2025 course. The goal is to create, fly and control our customized drone.

## Installation Instructions

This repository is structured as an **overlay** to facilitate the integration of the project files into standard PX4 and ROS 2 development environments.

The directory structure provided here exactly mirrors the original structure of the frameworks.

## Integration Procedure (Merge)

To install the project, you need to **merge** the contents of these folders into your local installations.

## 🔨 Build
```bash
cd /home/user/ros2_ws/src/ros2_ws/src

colcon build

. install/setup.bash
```

## 1. Fly the drone


To launch the drone:
```bash
cd /home/user/ros2_ws/src/ros2_ws/src/PXP-Autopilot

make px4_sitl gz_quad_plus
```
When launching the drone, remember to **always** run in another terminal:
```bash
cd /home/user/ros2_ws/src/ros2_ws/src

. install/setup.bash

. DDS_run.sh
```	

## 2. Forced land control

To run the force_land node:
```bash
cd /home/user/ros2_ws/src/ros2_ws/src

ros2 run force_land force_land
```

To plot the drone altitude and the manual control setpoint:
```bash
cd /home/user/ros2_ws/src/ros2_ws/src/results_force_land 

python3 plot_force_land.py 
```
## 3. Trajectory planning
To run the trajectory node:

```bash
cd /home/user/ros2_ws/src/ros2_ws/src

ros2 run offboard_rl trajectory_planner
```

To plot the requested data aquired during the simulation:
```bash
cd /home/user/ros2_ws/src/ros2_ws/src/results_plan_trajectory/results
	
python3 plot_trajectory.py testfinale
```
