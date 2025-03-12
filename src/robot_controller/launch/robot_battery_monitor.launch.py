from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # robot_nav_pkg = 'robot_nav'
    robot_con_pkg = 'robot_controller'
    # robot_det_pkg = 'robot_detection'
    robot_bat_pkg = 'robot_battery'

    launch_description = LaunchDescription()

    battery_simulator_node = Node(
            package=robot_bat_pkg,
            namespace='',
            executable='battery_simulator_node.py',
            name='battery_simulator_node',
        )
    launch_description.add_action(battery_simulator_node)

    charging_station_node = Node(
            package=robot_bat_pkg,
            namespace='',
            executable='charging_station_node.py',
            name='charging_station_node',
        )
    launch_description.add_action(charging_station_node)

    robot_battery_monitor = Node(
            package=robot_con_pkg,
            namespace='',
            executable='battery_monitor_node.py',
            name='robot_battery_monitor',
        )
    launch_description.add_action(robot_battery_monitor)
    
    return launch_description