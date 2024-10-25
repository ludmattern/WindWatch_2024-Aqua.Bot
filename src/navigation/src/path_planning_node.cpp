/*
File: path_planning_node.cpp
1. Node : path_planning_node
Role: Calculates the optimal path for the Aqua.Bot to reach its mission objectives while avoiding obstacles and complying with mission constraints.
Subscribed Topics:
/mission/odometry: Fused odometry data providing the current position and orientation of the boat.
/mission/objective_positions: Positions of the targets (wind turbines) to be reached.
/mission/avoidance_course: Suggested avoidance paths from the obstacle_avoidance_node.
/aqua_bot/ais_sensor/vessel_positions: Positions of nearby vessels for collision avoidance.
/mission/mission_goal: Overall mission goals and parameters set by the mission_manager_node.
/aqua_bot/ais_sensor/obstacle_positions: Positions of static obstacles (e.g., rocks, islands, lighthouses).
Published Topics:
/navigation/desired_trajectory: The planned trajectory for the Aqua.Bot to follow.
/navigation/navigation_status: Status updates on the navigation process for monitoring and debugging purposes.
*/

#include "navigation/path_planning_node.hpp"

PathPlanningNode::PathPlanningNode() : Node("path_planning_node"), TgtAdded(false), ShipAdded(false)
{
	RCLCPP_INFO(this->get_logger(), "Path Planning Node has started");

	//initialise subscriptions
	tgtPos_Subscirption_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
		"/mission/target_positions", 10,
		std::bind(&PathPlanningNode::AddTgtToPtsList, this, std::placeholders::_1));

	odometry_Subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"/mission/odometry", 10,
		std::bind(&PathPlanningNode::AddShipToPtsList, this, std::placeholders::_1));


	
}

void PathPlanningNode::AddTgtToPtsList(geometry_msgs::msg::PoseArray TgtPos)
{
	if (TgtAdded == false) //If targets not already added
	{
		for (int i = 0; i < TgtPos.poses.size(); ++i)
		{
			sPoint point;

			//Fill point with the ship position
			point.x = TgtPos.poses[i].position.x;
			point.y = TgtPos.poses[i].position.y;
			point.isTarget = true;

			PointList.push_back(point); //Add to point list
		}
		TgtAdded = true;
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

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<PathPlanningNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
