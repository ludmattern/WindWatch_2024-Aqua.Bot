import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare argument for selecting the world to load in Gazebo
    default_world_name = 'aquabot_regatta'
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world_name,
        description='Specify the world to load in Gazebo'
    )

    # Path to the launch files for each package
    sensors_launch_path = os.path.join(get_package_share_directory('sensors'), 'launch', 'sensors_launch.py')
    navigation_launch_path = os.path.join(get_package_share_directory('navigation'), 'launch', 'navigation_launch.py')
    mission_manager_launch_path = os.path.join(get_package_share_directory('mission_manager'), 'launch', 'mission_manager_launch.py')
    visualization_launch_path = os.path.join(get_package_share_directory('visualization'), 'launch', 'visualization_launch.py')
    aquabot_gz_launch_path = os.path.join(get_package_share_directory('aquabot_gz'), 'launch', 'competition.launch.py')

    # Include launch descriptions
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensors_launch_path),
        launch_arguments={'debug_message': 'Launching Sensors Package'}.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
        launch_arguments={'debug_message': 'Launching Navigation Package'}.items(),
    )

    mission_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mission_manager_launch_path),
        launch_arguments={'debug_message': 'Launching Mission Manager Package'}.items(),
    )

    visualization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(visualization_launch_path),
        launch_arguments={'debug_message': 'Launching Visualization Package'}.items(),
    )

    # Add Gazebo world launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(aquabot_gz_launch_path),
        launch_arguments={'world': launch.substitutions.LaunchConfiguration('world')}.items(),
    )

    # Build LaunchDescription
    return LaunchDescription([
        world_arg,                   # Argument for world name
        gazebo_launch,               # Launch Gazebo with specified world
        sensors_launch,              # Launch Sensors Package
        navigation_launch,           # Launch Navigation Package
        mission_manager_launch,      # Launch Mission Manager Package
        visualization_launch         # Launch Visualization Package
    ])
