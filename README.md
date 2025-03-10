# FRA532 EXAM1 : AMR in warehourse

"เดี่ยวมาเขียนอีกทีนะ ยังไม่ Final"

## Table of Contents
- [FRA532 EXAM1 : AMR in warehourse](#fra532-exam1--amr-in-warehourse)
  - [Table of Contents](#table-of-contents)
  - [Description](#description)
  - [Systeno req](#systeno-req)
  - [Installation](#installation)
    - [1.Clone ws](#1clone-ws)
    - [2.Clone MIR robot](#2clone-mir-robot)
    - [3.Clone AWS Warehourse](#3clone-aws-warehourse)
    - [4.Add cam to MIR](#4add-cam-to-mir)
  - [Usage](#usage)
    - [Run Simulation](#run-simulation)
    - [Run naviation](#run-naviation)
    - [Run robot\_controll systemp](#run-robot_controll-systemp)
    - [Run inspection\_node](#run-inspection_node)
  - [Contributing](#contributing)
  - [License](#license)

## Description
A brief description of what this project does and who it's for.

## Systeno req
```bash
sudo apt install ros-humble-nav2-bringup
```

## Installation
### 1.Clone ws
Clone ws
```bash
git clone https://github.com/ThepokKung/FRA532_EXAM1_ws -b main

```
### 2.Clone MIR robot
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

Test MIR is work
```bash
ros2 launch mir_gazebo mir_gazebo_launch.py world:=maze rviz_config_file:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/rviz/mir_nav.rviz
```

### 3.Clone AWS Warehourse

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

Test AWS Warehourse

```bash
ros2 launch aws_robomaker_small_warehouse_world small_warehouse.launch.py
```

### 4.Add cam to MIR

```bash
cd ~/FRA532_EXAM1_ws/

# Copy file from MiR100_cam to mir_robot_description
cp MiR100_cam/mir_100_v1.urdf.xacro src/mir_robot/mir_description/urdf/include/mir_100_v1.urdf.xacro 
```

## Usage
### Run Simulation
```bash
ros2 launch robot_gazebo sim.launch.py
```

### Run naviation
```bash
ros2 launch robot_nav navigation.launch.py
```

ใน RVIZ using 2D pose Estimate ด้วย

### Run robot_controll systemp
```bash
ros2 launch robot_controller robot_con.launch.py
```

### Run inspection_node
**BUG อยู่ รี Node ด้วยหลังรีันแต่ละรอบ กำลังแก้**
```bash
ros2 run robot_nav inspection.launch.py
```

## Contributing
Guidelines for contributing to the project.

## License
Information about the project's license.