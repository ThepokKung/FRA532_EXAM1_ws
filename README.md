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
    - [5 Node navigate station](#5-node-navigate-station)
    - [6. Robot Interaction \& Service Calls](#6-robot-interaction--service-calls)
      - [6.1 Sending Station Navigation Goals](#61-sending-station-navigation-goals)
      - [6.2 Sending Multistation Navigation Goals](#62-sending-multistation-navigation-goals)
      - [6.3  "Check \& Update Robot State and Station"](#63--check--update-robot-state-and-station)
        - [6.3.1 robot update state](#631-robot-update-state)
        - [6.3.2 robot state check](#632-robot-state-check)
        - [6.3.3 robot update station](#633-robot-update-station)
        - [6.3.4 robot check state](#634-robot-check-state)
  - [Demo](#demo)
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
> [!NOTE]
> We will install the vsstool and rosdep packages to ensure the system functions smoothly without any issues.

```bash
cd ~/FRA532_EXAM1_ws/

# Clone mir_robot into the ROS2 workspace
git clone -b humble-devel https://github.com/relffok/mir_robot src/mir_robot

# Fetch linked repositories using vcs
vcs import < src/mir_robot/ros2.repos src --recursive

# Install dependencies using rosdep (including ROS)
sudo apt update
sudo apt install -y python3-rosdep python3-vcstool
rosdep init
rosdep update --rosdistro=humble
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

# Build all packages in the workspace
cd ~/FRA532_EXAM1_ws/
colcon build
```

Test MiR robot:

```bash
source install/setup.bash

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
source install/setup.bash

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

### 5 Node navigate station
```bash
source install/setup.bash
ros2 run robot_nav station_navigator.py
```
> [!CAUTION]
> Sometimes, this node is running a server instance. If an error occurs, please restart the node.

### 6. Robot Interaction & Service Calls
This section provides an overview of service calls and topic commands that can be used to interact with the robot.

#### 6.1 Sending Station Navigation Goals
To navigate the robot to a specific station in docking mode, use the following command:

```bash
ros2 service call /station_2go robot_interfaces/srv/Station2GO "station: 'example'"
```

> [!WARNING]
> Can use only staion_name on database
> * ChangeStation
> * Station2
> * Station3
> * Station4

> [!CAUTION]
> Sometimes, this node is running a server instance. If an error occurs, please restart the node.

#### 6.2 Sending Multistation Navigation Goals
To send the robot to multiple stations sequentially in docking mode, use the following command:

```bash
ros2 service call /multi_station_2go robot_interfaces/srv/MultiTarget2GO "target_names: [example,example]" 
```

> [!WARNING]
> Can use only staion_name on database
> * ChangeStation
> * Station2
> * Station3
> * Station4


#### 6.3  "Check & Update Robot State and Station"
For check or update state-station on for robot_state

##### 6.3.1 robot update state
```bash
ros2 service call /update_robot_state robot_interfaces/srv/RobotStateUpdate "state: 'state_update'"
```

##### 6.3.2 robot state check
```bash
ros2 service call /check_robot_state robot_interfaces/srv/RobotStateCheck "checkstate: true"
```

##### 6.3.3 robot update station
```bash
ros2 service call /update_robot_station robot_interfaces/srv/RobotStationUpdate "station: 'station_update'"
```

##### 6.3.4 robot check state
```bash
ros2 service call /check_robot_station robot_interfaces/srv/RobotStationCheck "checkstation: true"
```

## Demo

[Usage launch workspace](https://youtu.be/PenU0T-6HBk) - Demonstrates how to launch the simulation workspace.

[Demo Auto charger Station](https://youtu.be/CqbIpWp4YWE) - Shows the robot automatically navigating to a charging station.

[Demo Station Navigation](https://youtu.be/eEjESY5RBzQ) - Demonstrates the robot moving to a single station.

[Demo MultiStationNavigation](https://youtu.be/MkLPEeSIWDY) - Shows the robot navigating through multiple stations.

## Future plan

* Fix existing bugs
* Improve navigation accuracy.
* Add object detection functionality.
* Fix issue with camera calibration.
  
## Developer Members
* 65340500004 Kraiwich Vichakhot
* 65340500009 Chayanin Napia  
* 65340500023 Natthaphat Sookpanya