# FRA532 EXAM1 : AMR in warehourse

"เดี่ยวมาเขียนอีกทีนะ ยังไม่ Final"

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
Examples of how to use the project.

## Contributing
Guidelines for contributing to the project.

## License
Information about the project's license.