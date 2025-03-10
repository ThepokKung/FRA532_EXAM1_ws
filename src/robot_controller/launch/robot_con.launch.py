import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
import xacro


def generate_launch_description():
    robot_nav_pkg = 'robot_nav'
    robot_con_pkg = 'robot_controller'
    robot_det_pkg = 'robot_detection'

    launch_description = LaunchDescription()

    nav2go_node = Node(
            package=robot_nav_pkg,
            namespace='',
            executable='nav2_go.py',
            name='nav2_go_node',
        )
    launch_description.add_action(nav2go_node)

    scheduler_node = Node(
            package=robot_con_pkg,
            namespace='',
            executable='scheduler_node.py',
            name='scheduler_node',
        )
    launch_description.add_action(scheduler_node)

    aruco_detect_node = Node(
            package=robot_det_pkg,
            namespace='',
            executable='aruco_detection.py',
            name='aruco_detect_node',
        )
    launch_description.add_action(aruco_detect_node)

    return launch_description
