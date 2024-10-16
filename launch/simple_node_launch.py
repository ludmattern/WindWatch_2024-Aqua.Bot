from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # Déclaration d'un argument pour spécifier le monde
    world_arg = DeclareLaunchArgument(
            'world',
            default_value='aquabot_regatta',
            description='Nom du monde Gazebo à charger')

    # Inclure un fichier de lancement existant pour lancer la compétition Gazebo
    aquabot_competition_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('aquabot_gz'), 
                         'launch/competition.launch.py')),
            launch_arguments={'world': 'aquabot_regatta'}.items()
    )

    # Lancer le nœud simple_node
    simple_node = Node(
        package='simple_package',
        executable='simple_node',  # Correspond à l'exécutable dans CMakeLists.txt
    )

    # Ajouter les actions au LaunchDescription
    ld.add_action(world_arg)
    ld.add_action(aquabot_competition_launch_file)
    ld.add_action(simple_node)

    return ld
