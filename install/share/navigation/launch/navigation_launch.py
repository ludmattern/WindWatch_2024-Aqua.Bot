# Launch file for navigation nodes
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation',
            executable='navigation_node',
            name='navigation_node',
            output='screen',
            parameters=[],
            on_exit=[launch.actions.LogInfo(msg="Navigation node shut down")],
        ),
        Node(
            package='navigation',
            executable='obstacle_avoidance_node',
            name='obstacle_avoidance_node',
            output='screen',
            parameters=[],
            on_exit=[launch.actions.LogInfo(msg="Obstacle Avoidance node shut down")],
        ),
    ])
