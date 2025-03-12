# FRA532 EXAM1 : AMR in warehouse

"เดี่ยวมาเขียนอีกทีนะ ยังไม่ Final"

## Table of Contents
- [FRA532 EXAM1 : AMR in warehouse](#fra532-exam1--amr-in-warehouse)
  - [Table of Contents](#table-of-contents)
  - [Description](#description)
  - [System requirements](#system-requirements)
  - [Installation](#installation)
    - [1.Clone ROS 2 Workspace](#1clone-ros-2-workspace)
    - [2.Clone MiR Robot Package](#2clone-mir-robot-package)
    - [3.Clone AWS Warehouse World](#3clone-aws-warehouse-world)
    - [4.Add Custom MiR100 Robot with Camera](#4add-custom-mir100-robot-with-camera)
  - [Usage](#usage)
    - [1.Launch Simulation](#1launch-simulation)
    - [2.Start Navigation Stack](#2start-navigation-stack)
    - [How to use](#how-to-use)
    - [3.Run Robot Controller system](#3run-robot-controller-system)
    - [Expected Behavior:](#expected-behavior)
    - [4.Launch Battery Simulation and Monitor](#4launch-battery-simulation-and-monitor)
    - [Expected Behavior:](#expected-behavior-1)
  - [Future plan](#future-plan)
  - [Developer Members](#developer-members)

## Description
This project focuses on the implementation of an Autonomous Mobile Robot (AMR) in a warehouse environment using ROS2. It includes navigation, inspection, Aruco marker detection, and battery simulation for efficient warehouse automation."

## System requirements
* ROS2 Humble ([installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html))
* Gazebo ([installation guide](https://gazebosim.org/docs/latest/ros_installation/))

## Installation
### 1.Clone ROS 2 Workspace
Clone the ROS 2 workspace:
```bash
git clone https://github.com/ThepokKung/FRA532_EXAM1_ws -b main
```
### 2.Clone MiR Robot Package
```bash
cd ~/FRA532_EXAM1_ws/

# Clone mir_robot into the ROS2 workspace
git clone -b humble-devel https://github.com/relffok/mir_robot src/mir_robot

# Fetch linked repositories using vcs
vcs import < src/mir_robot/ros2.repos src --recursive

# Install dependencies using rosdep (including ROS)
sudo apt update
sudo apt install -y python3-rosdep
rosdep update --rosdistro=humble
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

# Build all packages in the workspace
cd ~/FRA532_EXAM1_ws/
colcon build
```

Test MiR robot:

```bash
ros2 launch mir_gazebo mir_gazebo_launch.py world:=maze rviz_config_file:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/rviz/mir_nav.rviz
```

### 3.Clone AWS Warehouse World

```bash
cd ~/FRA532_EXAM1_ws/

# Clone AWS-warehouse into the ROS2 workspace
git clone https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git -b ros2 src/aws-robomaker-small-warehouse-world

# Install dependencies using rosdep (including ROS)
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

# Build all packages in the workspace
cd ~/FRA532_EXAM1_ws/
colcon build
```

Test AWS Warehouse:

```bash
ros2 launch aws_robomaker_small_warehouse_world small_warehouse.launch.py
```

### 4.Add Custom MiR100 Robot with Camera

```bash
cd ~/FRA532_EXAM1_ws/

# Copy file from MiR100_cam to mir_robot_description
cp MiR100_cam/mir_100_v1.urdf.xacro src/mir_robot/mir_description/urdf/include/mir_100_v1.urdf.xacro 
```

## Usage
### 1.Launch Simulation

Launches the warehouse simulation environment and MIR robot in Gazebo.

```bash
source install/setup.bash
ros2 launch robot_gazebo sim.launch.py
```

### 2.Start Navigation Stack

Starts the navigation stack, which allows the robot to move autonomously using SLAM and path planning.

```bash
source install/setup.bash
ros2 launch robot_nav navigation.launch.py
```
### How to use
* When RViz2 opens, select `"2D Pose Estimate"` and click on the map to initialize the robot's position
* Use `"2D Goal Pose"` to command the robot to navigate to a target location.

### 3.Run Robot Controller system

The Robot Controller System is responsible for managing the robot's movement to specified locations, detecting Aruco Markers, and performing Docking at designated stations.

```bash
source install/setup.bash
ros2 launch robot_controller robot_con.launch.py
```

### Expected Behavior:

* The robot will operate in different modes, such as multi-station inspection, single-destination navigation, or Aruco marker detection.
* The robot will automatically perform Docking upon reaching the designated station.

### 4.Launch Battery Simulation and Monitor

The Battery Simulation and Monitoring System consists of three main nodes that simulate the battery behavior, manage charging stations, and monitor the robot’s battery level.

```bash
source install/setup.bash
ros2 launch robot_controller robot_battery_monitor.launch.py
```

###  Expected Behavior:

* The system simulates battery consumption and charging behavior.

* The robot will autonomously navigate to the charging station when the battery is low and perform Docking for recharging.

## Future plan

* Fix existing bugs
* Improve navigation accuracy.
* Add object detection functionality.
* Fix issue with camera calibration.
  
## Developer Members
* 6534050000 Kraiwich Vichakhot
* Plan
* Gun