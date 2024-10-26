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
#include <queue>
#include "navigation/path_planning_node.hpp"

PathPlanningNode::PathPlanningNode() : Node("path_planning_node"), ShipAdded(false), NbObjectives(0)
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

		++NbObjectives;
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

		++NbObjectives;
		PointList.insert(PointList.begin(), point); //Add to point list at first element
		ShipAdded = true;
	}
}

static void AddPointToPolygon(sPolygon *polygon, std::vector<sPoint> *PointList, std::string PointString, int PolygonId)
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


std::pair<double, std::vector<int>> PathPlanningNode::Dijkstra(int start, int end)
{
	//Distance from the starting point
	std::vector<double> dist(Graph.size(), std::numeric_limits<double>::infinity());

	//Predecessor of each point
	std::vector<int> previous(Graph.size(), -1);

	dist[start] = 0;

	//Priority queue to store the points to explore sort by distance
	std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, 
		std::greater<>> PriorityQueue;

	PriorityQueue.push({0, start});

	while (PriorityQueue.empty() == false)
	{
		int Point = PriorityQueue.top().second;
		int Distance = PriorityQueue.top().first;
		PriorityQueue.pop();

		if (Point == end) //If reach the end
			break;

		if (Distance != dist[Point]) //If a best path was found for this point
			continue ;

		for (int i = 0; i < Graph.size(); ++i)
		{
			//Distance vers ele point voisin i
			double WeightNeighbour = Graph[Point][i];

			//If shortest path is found
			if (dist[Point] + WeightNeighbour < dist[i])
			{
				dist[i] = dist[Point] + WeightNeighbour;
				previous[i] = Point;
				PriorityQueue.push({dist[i], i});
			}
		}
		
	}

	//Path to go from start to end
	std::vector<int> Path;

	for (int CurrentPoint = end; CurrentPoint != -1; CurrentPoint = previous[CurrentPoint])
		Path.push_back(CurrentPoint);
	std::reverse(Path.begin(), Path.end());

	return {dist[end], Path};
}

void PathPlanningNode::CreateObjectivesGraph(void)
{
	for (int i = 0; i < NbObjectives; ++i)
	{
		for (int j = i; j < NbObjectives; ++j)
		{

		}
	}
}

void PathPlanningNode::CreatePath(void)
{
	std::pair<double, std::vector<int>> Path = Dijkstra(0, 1);

	std::ostringstream str;

	str << "Dist: " << Path.first << '\n';

	for(int i = 0; i < Path.second.size(); ++i)
		str << Path.second[i] << " : " << PointList[Path.second[i]].x << "," << PointList[Path.second[i]].y << '\n';
	
	RCLCPP_INFO(this->get_logger(), "%s", str.str().c_str());
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

	RCLCPP_INFO(node->get_logger(), "Creating Graph");
	node->CreateGraph();

	node->CreatePath();

	rclcpp::shutdown();
	return (0);
}
