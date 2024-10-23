/*
File: path_planning_node.cpp
1. Node : path_planning_node
Role: Calculates the optimal path for the Aqua.Bot to reach its mission objectives while avoiding obstacles and complying with mission constraints.
Subscribed Topics:
/mission/odometry: Fused odometry data providing the current position and orientation of the boat.
/mission/objective_positions: Positions of the targets (wind turbines) to be reached.
/mission/avoidance_course: Suggested avoidance paths from the obstacle_avoidance_node.
/aqua_bot/ais_sensor/vessel_positions: Positions of nearby vessels for collision avoidance.
/mission/mission_goal: Overall mission goals and parameters set by the mission_coordinator_node.
/aqua_bot/ais_sensor/obstacle_positions: Positions of static obstacles (e.g., rocks, islands, lighthouses).
Published Topics:
/navigation/desired_trajectory: The planned trajectory for the Aqua.Bot to follow.
/navigation/navigation_status: Status updates on the navigation process for monitoring and debugging purposes.
*/

#include "navigation/path_planning_node.hpp"

PathPlanningNode::PathPlanningNode() : Node("path_planning_node"), ShipAdded(false)
{
	RCLCPP_INFO(this->get_logger(), "Path Planning Node has started");

	//Initialise subscriptions
	odometry_Subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"/mission/odometry", 10,
		std::bind(&PathPlanningNode::AddShipToPtsList, this, std::placeholders::_1));

	//Initialise client for targets positions
	TgtPos_Client_ = this->create_client<sensors::srv::TargetPositions>("mission/target_positions");
}

void PathPlanningNode::SendServiceRequest(void)
{

}

void PathPlanningNode::AddTgtToPtsList(geometry_msgs::msg::PoseArray TgtPos)
{
	RCLCPP_INFO(this->get_logger(), "AA TEST 3 %ld", TgtPos.poses.size());
	for (int i = 0; i < TgtPos.poses.size(); ++i)
	{
		sPoint point;

		//Fill point with the target position
		point.x = TgtPos.poses[i].position.x;
		point.y = TgtPos.poses[i].position.y;
		point.isTarget = true;

		PointList.push_back(point); //Add to point list
	}
	for (int i = 0; i < PointList.size(); ++i)
	{
		RCLCPP_INFO(this->get_logger(), "AA x: %f y: %f", PointList[i].x, PointList[i].y);
	}
}

void PathPlanningNode::AddShipToPtsList(nav_msgs::msg::Odometry ShipPos)
{
	if (ShipAdded == false) //If ship not already added
	{
		sPoint point;

		//Fill point with the ship position
		point.x = ShipPos.pose.pose.position.x;
		point.y = ShipPos.pose.pose.position.y;
		point.isTarget = false;

		PointList.push_back(point); //Add to point list
		ShipAdded = true;
	}
}

void PathPlanningNode::AddObstaclePtsList(void)
{
	
}

geometry_msgs::msg::PoseArray MakeRequest(std::shared_ptr<PathPlanningNode> node, 
	sensors::srv::TargetPositions::Request::SharedPtr request)
{


	//Send request to the service 
	rclcpp::Client<sensors::srv::TargetPositions>::FutureAndRequestId future = node->TgtPos_Client_->async_send_request(request);

	//Wait until a response
	RCLCPP_INFO(node->get_logger(), "AA TEST");
	if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) //If success
		return(future.get()->poses);
	else
		RCLCPP_ERROR(node->get_logger(), "Failed to call service target_positions");
	
	RCLCPP_INFO(node->get_logger(), "AA TEST2");
	return (future.get()->poses);
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<PathPlanningNode>();

	//Create the request
	sensors::srv::TargetPositions::Request::SharedPtr request = std::make_shared<sensors::srv::TargetPositions::Request>();

	//Wait until the service is available
	while (node->TgtPos_Client_->wait_for_service(std::chrono::seconds(1)) == false)
	{
		if (rclcpp::ok() == false)
		{
			RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
			return (1);
		}
	}

	geometry_msgs::msg::PoseArray TgtPos;
	while (TgtPos.poses.empty())
		TgtPos = MakeRequest(node, request);

	node->AddTgtToPtsList(TgtPos);

	rclcpp::shutdown();
	return (0);
}
