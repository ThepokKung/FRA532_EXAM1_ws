import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1) Find the path to your fra532_gazebo package
    fra532_gazebo_pkg_dir = get_package_share_directory('fra532_gazebo')
    # Path to the goofy_office.launch.py file
    goofy_office_launch_path = os.path.join(fra532_gazebo_pkg_dir, 'launch', 'warehouse_bringup.launch.py')

    # 2) Include the launch file that starts Gazebo with goofy_office.world
    goofy_office_world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(goofy_office_launch_path)
    )

    # 3) Get paths to the MiR description and Gazebo packages
    mir_description_dir = get_package_share_directory('mir_description')
    mir_gazebo_dir = get_package_share_directory('mir_gazebo')

    # Include the MiR description launch file
    launch_mir_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_description_dir, 'launch', 'mir_launch.py')
        )
    )

    # Include the MiR Gazebo common launch file (sensors, controllers, etc.)
    launch_mir_gazebo_common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_gazebo_dir, 'launch', 'include', 'mir_gazebo_common.py')
        )
    )

    # 4) Define a teleop node to control the robot via keyboard
    launch_teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e'
    )

    # 5) Define the spawn_entity node to actually place the MiR robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'mir_robot',
            '-topic', 'robot_description',
            '-b'  # bond node to gazebo model
        ],
        output='screen'
    )

    # 6) Create the overall LaunchDescription
    ld = LaunchDescription()

    # 7) Add each action to the LaunchDescription in a logical order
    #    1) Start the goofy_office world
    #    2) Launch MiR description
    #    3) Launch MiR Gazebo common
    #    4) Spawn the robot
    #    5) Launch teleop
    ld.add_action(goofy_office_world_cmd)
    ld.add_action(launch_mir_description)
    ld.add_action(launch_mir_gazebo_common)
    ld.add_action(spawn_robot)
    ld.add_action(launch_teleop)

    return ld
