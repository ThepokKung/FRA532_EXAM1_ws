import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    warehouse_pkg_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
    warehouse_launch_path = os.path.join(warehouse_pkg_dir, 'launch')

    robot_gazebo_pkg_dir = get_package_share_directory('robot_gazebo')
    robot_gazebo_launch_path = os.path.join(robot_gazebo_pkg_dir, 'launch')

    # Add Here
    mir_description_dir = get_package_share_directory('mir_description')
    mir_gazebo_dir = get_package_share_directory('mir_gazebo')


    # warehouse_world_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([warehouse_launch_path, '/no_roof_small_warehouse.launch.py'])
    # )

    warehouse_world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_gazebo_launch_path, '/worlds_aruco.launch.py'])
    )

    # Add Here
    launch_mir_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_description_dir, 'launch', 'mir_launch.py')
        )
    )

    launch_mir_gazebo_common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_gazebo_dir, 'launch',
                         'include', 'mir_gazebo_common.py')
        )
    )

    # for launch file dummy_robot.launch.py
    launch_cart_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("cart_description"),
                    "launch",
                    "cart.launch.py"
                )
            ]
        ),
        launch_arguments={"use_sim_time":"true"}.items()
    )

    launch_teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        namespace='',
        output='screen',
        prefix='xterm -e')
    
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        arguments=['-entity', 'mir_robot',
                '-topic', 'robot_description',
                '-b'],  # bond node to gazebo model,
        namespace='',
        output='screen',
        parameters=[{'verbose': 'true'}]  # Add verbose argument here
    )

    # create robot in gazebo
    spawn_cart = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_cart",
        arguments=[
            "-topic", "/cart/robot_description",
            "-entity", "cart",
            '-x', '-2.0',
            '-y', '-2.0',
            '-z', '0.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        output = "screen"
    )

    ld = LaunchDescription()

    ld.add_action(launch_teleop)
    ld.add_action(warehouse_world_cmd)
    ld.add_action(launch_mir_description)
    ld.add_action(launch_mir_gazebo_common)
    ld.add_action(launch_cart_description)
    ld.add_action(spawn_robot)
    ld.add_action(spawn_cart)

    return ld