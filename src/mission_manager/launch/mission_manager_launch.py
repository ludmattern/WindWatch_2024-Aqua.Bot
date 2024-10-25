# mission_manager_launch.py
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mission_manager',
            executable='mission_manager_node',
            name='mission_manager_node',
            output='screen',
            parameters=[],
            on_exit=[launch.actions.LogInfo(msg="Mission Manager node shut down")],
        ),
        Node(
            package='mission_manager',
            executable='target_manager_node',
            name='target_manager_node',
            output='screen',
            parameters=[],
            on_exit=[launch.actions.LogInfo(msg="Target Manager node shut down")],
        ),
        Node(
            package='mission_manager',
            executable='navigation_server',
            name='navigation_server',
            output='screen',
            parameters=[],
            on_exit=[launch.actions.LogInfo(msg="Navigation Server node shut down")],
        ),
        Node(
            package='mission_manager',
            executable='inspection_server',
            name='inspection_server',
            output='screen',
            parameters=[],
            on_exit=[launch.actions.LogInfo(msg="Inspection Server node shut down")],
        ),
        Node(
            package='mission_manager',
            executable='stabilization_server',
            name='stabilization_server',
            output='screen',
            parameters=[],
            on_exit=[launch.actions.LogInfo(msg="Stabilization Server node shut down")],
        ),
        Node(
            package='mission_manager',
            executable='rotation_server',
            name='rotation_server',
            output='screen',
            parameters=[],
            on_exit=[launch.actions.LogInfo(msg="Rotation Server node shut down")],
        ),
    ])
