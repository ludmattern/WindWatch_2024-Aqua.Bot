# launch/mission_manager_launch.py

import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Créer les actions pour lancer les nœuds
    mission_manager_node = Node(
        package='mission_manager',
        executable='mission_manager_node',
        name='mission_manager_node',
        output='screen'
    )

    navigation_server = Node(
        package='mission_manager',
        executable='navigation_server',
        name='navigation_server',
        output='screen'
    )

    inspection_server = Node(
        package='mission_manager',
        executable='inspection_server',
        name='inspection_server',
        output='screen'
    )

    stabilization_server = Node(
        package='mission_manager',
        executable='stabilization_server',
        name='stabilization_server',
        output='screen'
    )

    rotation_server = Node(
        package='mission_manager',
        executable='rotation_server',
        name='rotation_server',
        output='screen'
    )

    # Créer et retourner la description du lancement
    return LaunchDescription([
        mission_manager_node,
        navigation_server,
        inspection_server,
        stabilization_server,
        rotation_server
    ])
