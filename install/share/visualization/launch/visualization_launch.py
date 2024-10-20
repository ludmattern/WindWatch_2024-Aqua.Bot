# visualization_launch.py
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='visualization',
            executable='visualization_node',
            name='visualization_node',
            output='screen',
            parameters=[],
            on_exit=[launch.actions.LogInfo(msg="Visualization node shut down")],
        ),
    ])
