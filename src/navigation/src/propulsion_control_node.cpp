/*
Node propulsion_control_node
Role: Converts high-level propulsion commands into low-level control signals for the motors, handling the physical interaction with the hardware.
Subscribed Topics:
/propulsion/command: Commands from the control_node.
Published Topics:
/aqua_bot/thrusters/left/pos: Steering angle command for the left thruster.
/aqua_bot/thrusters/right/pos: Steering angle command for the right thruster.
/aqua_bot/thrusters/left/thrust: Thrust command for the left thruster.
/aqua_bot/thrusters/right/thrust: Thrust command for the right thruster.
*/

#include "navigation/propulsion_control_node.hpp"

PropulsionControlNode::PropulsionControlNode() : Node("propulsion_control_node")
{
	RCLCPP_INFO(this->get_logger(), " Propulsion control node has started");
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
	auto node = std::make_shared<PropulsionControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

