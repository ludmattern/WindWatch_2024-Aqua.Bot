# Launch file for navigation nodes
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation',
            executable='path_planning_node',
            name='path_planning_node',
            output='screen',
            parameters=[],
            on_exit=[launch.actions.LogInfo(msg="Path planning node shut down")],
        ),
        Node(
            package='navigation',
            executable='obstacle_avoidance_node',
            name='obstacle_avoidance_node',
            output='screen',
            parameters=[],
            on_exit=[launch.actions.LogInfo(msg="Obstacle Avoidance node shut down")],
        ),
		Node(
			package='navigation',
			executable='control_node',
			name='control_node',
			output='screen',
			parameters=[],
			on_exit=[launch.actions.LogInfo(msg="Control node shut down")],
		),
		Node(
			package='navigation',
			executable='propulsion_control_node',
			name='propulsion_control_node',
			output='screen',
			parameters=[],
			on_exit=[launch.actions.LogInfo(msg="Propulsion Control node shut down")],
		),
    ])
