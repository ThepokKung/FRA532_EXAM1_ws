import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Path to the gazebo_ros package (for gzserver and gzclient launch files)
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')

    # Path to your own package that contains the models folder
    pkg_share = FindPackageShare(package='robot_gazebo').find('robot_gazebo')

    # Path to the .world file you want to load
    world_file_name = 'warehouse_aruco_no_actor.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)

    # Path to the models folder you want Gazebo to find
    gazebo_models_path = os.path.join(pkg_share, 'models')

    # Declare some LaunchConfigurations
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')

    # Launch arguments
    declare_simulator_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to start the simulator')
    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model file to load')

    # Action that sets the GAZEBO_MODEL_PATH
    # so that Gazebo can find your new model folder
    set_model_path_cmd = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_models_path
    )

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world,'verbose': 'true'}.items()
    )

    # Start Gazebo client
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless]))
    )

    # Create launch description
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_world_cmd)

    # Export GAZEBO_MODEL_PATH
    ld.add_action(set_model_path_cmd)

    # Add Gazebo server/client
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    return ld
