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

#include <fstream>
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

void PathPlanningNode::AddTgtToPtsList(geometry_msgs::msg::PoseArray TgtPos)
{
	for (int i = 0; i < TgtPos.poses.size(); ++i)
	{
		sPoint point;

		//Fill point with the target position
		point.x = TgtPos.poses[i].position.x;
		point.y = TgtPos.poses[i].position.y;
		point.IsGoal = true;
		point.PolygonId = -1;

		PointList.push_back(point); //Add to point list
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
		point.IsGoal = true;
		point.PolygonId = -1;

		PointList.push_back(point); //Add to point list
		ShipAdded = true;
	}
}

void AddPointToPolygon(sPolygon *polygon, std::vector<sPoint> *PointList, std::string PointString, int PolygonId)
{
	std::stringstream PointStream(PointString);
	sPoint point;
	char delimiter;

	PointStream >> point.x >> delimiter >> point.y;
	point.IsGoal = false;
	point.PolygonId = PolygonId;

	polygon->Points.push_back(point);
	PointList->push_back(point);
}

int PathPlanningNode::AddObstaclePtsList(void)
{
	std::ifstream ObstaclePos(OBSTACLE_FILE);
	std::string line;
	int i = 0;

	if (ObstaclePos.is_open() == 0)
		return (1);
	
	while (std::getline(ObstaclePos, line))
	{
		std::stringstream LineStream(line);
		std::string point;

		ObstacleList.resize(ObstacleList.size() + 1);
		while (std::getline(LineStream, point, ' '))
			AddPointToPolygon(&ObstacleList[i], &PointList, point, i);
		++i;
	}
	return (0);	
}

geometry_msgs::msg::PoseArray MakeRequest(std::shared_ptr<PathPlanningNode> node, 
	sensors::srv::TargetPositions::Request::SharedPtr request)
{


	//Send request to the service 
	rclcpp::Client<sensors::srv::TargetPositions>::FutureAndRequestId future = node->TgtPos_Client_->async_send_request(request);

	//Wait until a response
	if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) //If success
		return(future.get()->poses);
	else
		RCLCPP_ERROR(node->get_logger(), "Failed to call service target_positions");
	
	return (future.get()->poses);
}

void PathPlanningNode::InitGraphSize(int size)
{
	Graph.resize(size);
	for (int i = 0; i < size; ++i)
		Graph[i].resize(size);
}

int orientation(sPoint p, sPoint q, sPoint r)
{
	double val;

	//Calculate orientation 
	val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

	if (val == 0)
		return 0; //collinear
	else if (val > 0)
		return (1); //clockwise
	else
		return (2); //Counter clockwise
}

bool onSegment(sPoint p, sPoint q, sPoint r)
{
	//Check if q is on pr
	if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
		q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
		return (true);
	return (false);
}

bool CheckInterLines(sPoint p1, sPoint q1, sPoint p2, sPoint q2)
{
	int o1 = orientation(p1, q1, p2);
	int o2 = orientation(p1, q1, q2);
	int o3 = orientation(p2, q2, q1);
	int o4 = orientation(p2, q2, q1);

	//General case
	if (o1 != o2 && o3 != o4)
		return (true);
	
	//If colinear
	if (o1 == 0 && onSegment(p1, p2, q1))
		return (true);
	if (o2 == 0 && onSegment(p1, q2, q1))
		return (true);
	if (o3 == 0 && onSegment(p2, p1, q2))
		return (true);
	if (o4 == 0 && onSegment(p2, q1, q2))
		return (true);
	
	return (false);
}

bool PathPlanningNode::CheckInterPoly(sPoint FirstPoint, sPoint SecondPoint)
{
	int i = 0;
	//Check if the line cross any polygon side
	while (i < ObstacleList.size());
	{
		for (int j = 0; j < ObstacleList[i].Points.size(); ++j)
		{
			int nextPoint = (j + 1) % (ObstacleList[i].Points.size() + 1); //If j = end check with point 0

			if (CheckInterLines(FirstPoint, SecondPoint, ObstacleList[i].Points[j], 
				ObstacleList[i].Points[nextPoint]) == true)
				return (true);
		}
		++i;
	}
}

void PathPlanningNode::CreateGraph(void)
{
	InitGraphSize(PointList.size());

	//Connect all points together
	for (int i = 0; i < PointList.size(); ++i)
	{
		for (int j = 0; j < PointList.size(); ++j)
		{
			int PointIndex = i * PointList.size() + j;

			//If the points are the same point 
			if (i == j)
				Graph[i][j] = std::numeric_limits<double>::infinity();

			//If the points are in the same polygon
			else if (PointList[i].PolygonId != -1 && PointList[i].PolygonId == PointList[PointIndex].PolygonId)
				Graph[i][j] = std::numeric_limits<double>::infinity();
			

		}
	}
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
			rclcpp::shutdown();
			return (1);
		}
	}

	geometry_msgs::msg::PoseArray TgtPos;
	while (TgtPos.poses.empty())
		TgtPos = MakeRequest(node, request);

	node->AddTgtToPtsList(TgtPos);
	if (node->AddObstaclePtsList() == 1)
	{
		rclcpp::shutdown();
		return (1);
	}

	rclcpp::shutdown();
	return (0);
}
