# mission_manager_launch.py
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mission_manager',
            executable='mission_coordinator_node',
            name='mission_coordinator_node',
            output='screen',
            parameters=[],
            on_exit=[launch.actions.LogInfo(msg="Mission Coordinator node shut down")],
        ),
        Node(
            package='mission_manager',
            executable='target_manager_node',
            name='target_manager_node',
            output='screen',
            parameters=[],
            on_exit=[launch.actions.LogInfo(msg="Target Manager node shut down")],
        ),
    ])
