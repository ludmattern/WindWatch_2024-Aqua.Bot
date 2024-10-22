// tgt_pos_update_node.cpp
#include "sensors/tgt_pos_update_node.hpp"
#include <iostream>

TgtPosUpdateNode::TgtPosUpdateNode() : Node("tgt_pos_update_node")
{
	RCLCPP_INFO(this->get_logger(), "TGT Pos Update Node has started");
	tgtPos_subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
		"/aquabot/ais_sensor/windturbines_positions", 10,
		std::bind(&TgtPosUpdateNode::tgtPosCallBack, this, std::placeholders::_1));

}

void TgtPosUpdateNode::tgtPosCallBack(geometry_msgs::msg::PoseArray msg)
{
	RCLCPP_INFO(this->get_logger(), "Target messages received : Nb target %ld", msg.poses.size());
	for (int i = 0; i < msg.poses.size(); ++i)
	{
		RCLCPP_INFO(this->get_logger(), "target : %d x: %f, y: %f, z: %f", 
		i ,msg.poses[i].position.x ,msg.poses[i].position.y, msg.poses[i].position.z);
		RCLCPP_INFO(this->get_logger(), "target : %d : w :%f, x: %f, y: %f, z: %f", 
		i ,msg.poses[i].orientation.w ,msg.poses[i].orientation.x, msg.poses[i].orientation.y, msg.poses[i].orientation.z);
	}
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<TgtPosUpdateNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
