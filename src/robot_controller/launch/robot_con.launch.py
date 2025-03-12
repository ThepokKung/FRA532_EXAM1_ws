from launch import LaunchDescription
from launch_ros.actions import Node


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

    inspection_node = Node(
            package=robot_nav_pkg,
            namespace='',
            executable='inspection_node.py',
            name='inspection_node',
        )
    launch_description.add_action(inspection_node)

    # station_navigator_node = Node(
    #         package=robot_nav_pkg,
    #         namespace='',
    #         executable='station_navigator.py',
    #         name='station_navigator',
    #     )
    # launch_description.add_action(station_navigator_node)

    aruco_detect_node = Node(
            package=robot_det_pkg,
            namespace='',
            executable='aruco_detection.py',
            name='aruco_detect_node',
        )
    launch_description.add_action(aruco_detect_node)

    robot_state_node = Node(
            package=robot_con_pkg,
            namespace='',
            executable='robot_state.py',
            name='robot_state_node',
        )
    launch_description.add_action(robot_state_node)
    
    return launch_description