# Launch file for sensor nodes
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='sensors',
			executable='sensor_fusion_node',
			name='sensor_fusion_node',
			output='screen',
			parameters=[],
			on_exit=[launch.actions.LogInfo(msg="Sensor Fusion node shut down")],
		),
		Node(
			package='sensors',
			executable='camera_control_node',
			name='camera_control_node',
			output='screen',
			parameters=[],
			on_exit=[launch.actions.LogInfo(msg="Camera Control node shut down")],
		),
		Node(
			package='sensors',
			executable='tgt_pos_update_node',
			name='tgt_pos_update_node',
			output='screen',
			parameters=[],
			on_exit=[launch.actions.LogInfo(msg="Target Position Update node shut down")],
		),
	])
