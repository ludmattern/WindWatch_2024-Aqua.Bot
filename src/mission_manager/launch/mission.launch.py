# mission_launch.py
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the launch files for each package
    navigation_launch_path = os.path.join(get_package_share_directory('navigation'), 'launch', 'navigation_launch.py')
    sensors_launch_path = os.path.join(get_package_share_directory('sensors'), 'launch', 'sensors_launch.py')
    mission_manager_launch_path = os.path.join(get_package_share_directory('mission_manager'), 'launch', 'mission_manager_launch.py')
    visualization_launch_path = os.path.join(get_package_share_directory('visualization'), 'launch', 'visualization_launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch_path),
            launch_arguments={'debug_message': 'Launching Navigation Package'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensors_launch_path),
            launch_arguments={'debug_message': 'Launching Sensors Package'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mission_manager_launch_path),
            launch_arguments={'debug_message': 'Launching Mission Manager Package'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(visualization_launch_path),
            launch_arguments={'debug_message': 'Launching Visualization Package'}.items(),
        ),
    ])
