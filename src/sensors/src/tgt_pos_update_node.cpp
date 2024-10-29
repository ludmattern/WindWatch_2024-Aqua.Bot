// tgt_pos_update_node.cpp
#include <iostream>
#include "sensors/tgt_pos_update_node.hpp"
#include "ConvertCoordinates.hpp"

TgtPosUpdateNode::TgtPosUpdateNode() : Node("tgt_pos_update_node")
{
	RCLCPP_INFO(this->get_logger(), "TGT Pos Update Node has started");

	//Create subscrition to get target positions
	tgtPos_subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
		"/aquabot/ais_sensor/windturbines_positions", 10,
		std::bind(&TgtPosUpdateNode::tgtPosCallBack, this, std::placeholders::_1));

	//Create Service
	tgtPos_Service_ = this->create_service<sensors::srv::TargetPositions>(
		"mission/target_positions", std::bind(&TgtPosUpdateNode::ServerCallback, this, std::placeholders::_1, std::placeholders::_2));

}

void TgtPosUpdateNode::tgtPosCallBack(geometry_msgs::msg::PoseArray msg)
{
	if (current_positions_.poses.empty())
	{
		current_positions_.poses.resize(msg.poses.size()); //resize the vector
		for (int i = 0; i < msg.poses.size(); ++i)
		{
			double x,y,z;
			latLonToENU(msg.poses[i].position.x ,msg.poses[i].position.y, msg.poses[i].position.z, x, y, z); //convert coordinates

			//Set position to target position
			current_positions_.poses[i].position.x = x;
			current_positions_.poses[i].position.y = y;
			current_positions_.poses[i].position.z = z;

			//Set rotation to 0
			current_positions_.poses[i].orientation.w = 0;
			current_positions_.poses[i].orientation.x = 0;
			current_positions_.poses[i].orientation.y = 0;
			current_positions_.poses[i].orientation.z = 0;
		}
	}
}

void TgtPosUpdateNode::ServerCallback(const std::shared_ptr<sensors::srv::TargetPositions::Request> request,
	const std::shared_ptr<sensors::srv::TargetPositions::Response> response)
{
	//Set response to the target positions.
	response->poses = current_positions_;
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<TgtPosUpdateNode>();
	
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node);
	executor.spin();

	rclcpp::shutdown();
	return 0;
}
