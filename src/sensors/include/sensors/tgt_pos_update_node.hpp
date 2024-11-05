#ifndef TGT_POS_UPDATE_NODE_HPP
#define TGT_POS_UPDATE_NODE_HPP

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensors/srv/target_positions.hpp"

class TgtPosUpdateNode : public rclcpp::Node
{
	public:
		TgtPosUpdateNode();

	private:

		//Subscription
		rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr tgtPos_subscription_;

		rclcpp::CallbackGroup::SharedPtr callback_group_;

		//Service
		rclcpp::Service<sensors::srv::TargetPositions>::SharedPtr tgtPos_Service_;

		void tgtPosCallBack(geometry_msgs::msg::PoseArray msg);

		void ServerCallback(const std::shared_ptr<sensors::srv::TargetPositions::Request> request,
			const std::shared_ptr<sensors::srv::TargetPositions::Response> response);

		geometry_msgs::msg::PoseArray current_positions_; //Save current pos
		bool positionsInitialized_;
};

#endif // TGT_POS_UPDATE_NODE_HPP
