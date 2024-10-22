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

	//Create publisher
	tgtPos_Publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
		"/mission/target_positions", 10);

}

void TgtPosUpdateNode::tgtPosCallBack(geometry_msgs::msg::PoseArray msg)
{
	geometry_msgs::msg::PoseArray msgPublisher;

	msgPublisher.poses.resize(msg.poses.size()); //resize the vector
	for (int i = 0; i < msg.poses.size(); ++i)
	{
		double x,y,z;
		latLonToENU(msg.poses[i].position.x ,msg.poses[i].position.y, msg.poses[i].position.z, x, y, z); //convert coordinates

		//Set topic msg position to target position
		msgPublisher.poses[i].position.x = x;
		msgPublisher.poses[i].position.y = y;
		msgPublisher.poses[i].position.z = z;

		//Set topic msg rotation to 0
		msgPublisher.poses[i].orientation.w = 0;
		msgPublisher.poses[i].orientation.x = 0;
		msgPublisher.poses[i].orientation.y = 0;
		msgPublisher.poses[i].orientation.z = 0;
	}
	tgtPos_Publisher_->publish(msgPublisher); //Publish message on topic
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<TgtPosUpdateNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
