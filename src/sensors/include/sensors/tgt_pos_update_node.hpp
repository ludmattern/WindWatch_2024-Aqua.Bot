#ifndef TGT_POS_UPDATE_NODE_HPP
#define TGT_POS_UPDATE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_array.hpp"

class TgtPosUpdateNode : public rclcpp::Node
{
	public:
		TgtPosUpdateNode();

	private:

		//Subscription
		rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr tgtPos_subscription_;

		void tgtPosCallBack(geometry_msgs::msg::PoseArray msg);
};

#endif // TGT_POS_UPDATE_NODE_HPP
