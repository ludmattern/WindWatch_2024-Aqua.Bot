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

PathPlanningNode::PathPlanningNode() : Node("path_planning_node"),timer_(nullptr), ShipAdded(false),TargetsAdded(false),ObstaclesAdded(false)
										,PathFinded(false) ,NbObjectives(0)
{
	RCLCPP_INFO(this->get_logger(), "Path Planning Node has started");

	callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

	//Initialise the service
	PathService_ = this->create_service<navigation::srv::Path>(
		"/navigation/path", std::bind(&PathPlanningNode::ServerCallback, this,
			std::placeholders::_1, std::placeholders::_2),
			rmw_qos_profile_services_default, callback_group_);

	//Initialise subscriptions
	odometry_Subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"/mission/odometry", 10,
		std::bind(&PathPlanningNode::AddShipToPtsList, this, std::placeholders::_1));

	//Initialise client for targets positions
	TgtPos_Client_ = this->create_client<sensors::srv::TargetPositions>("mission/target_positions");
	timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PathPlanningNode::launch, this));
}

void PathPlanningNode::ServerCallback(const std::shared_ptr<navigation::srv::Path::Request> &request,
	const std::shared_ptr<navigation::srv::Path::Response> &response) const
{
	if (ShipAdded == false || TargetsAdded == false || ObstaclesAdded == false || PathFinded == false)
		return;

	response->path = Path;
	response->pose_array = Targets;
}


void PathPlanningNode::launch(void)
{
	timer_->cancel();

	while (this->TgtPos_Client_->wait_for_service(std::chrono::seconds(1)) == false)
		RCLCPP_ERROR(this->get_logger(), "Service 'mission/target_positions' not available");

	const auto request = std::make_shared<sensors::srv::TargetPositions::Request>();
	auto future = this->TgtPos_Client_->async_send_request(request,
		std::bind(&PathPlanningNode::serviceResponseCallback, this, std::placeholders::_1));
}

void PathPlanningNode::serviceResponseCallback(rclcpp::Client<sensors::srv::TargetPositions>::SharedFuture future)
{
	if (ShipAdded == false)
	{
		// Set a timer to retry when ship_added becomes true
		auto retry_timer = this->create_wall_timer(std::chrono::seconds(1),
			[this, future]() {
				this->serviceResponseCallback(future);  // Retry processing the response
			});
		return;  // Exit the callback to wait until ship_added becomes true
	}

	const auto &response = future.get();

	if (response->poses.poses.empty() == true)
	{
		RCLCPP_WARN(this->get_logger(), "Received empty poses from service. Retrying...");
		auto retry_timer = this->create_wall_timer(std::chrono::seconds(2),
			[this]() {
					this->launch();
			});
	}
	else
	{
		this->AddTgtToPtsList(response->poses);
		if (this->AddObstaclePtsList() == 1)
		{
			RCLCPP_ERROR(this->get_logger(), "Can't open obstacles file");
			return;
		}

		//Create the graph
		this->CreateGraph();

		//Find the path
		Path = this->CreatePath();
		PathFinded = true;
	}
}


void PathPlanningNode::AddTgtToPtsList(const geometry_msgs::msg::PoseArray &TgtPos)
{
	for (size_t i = 0; i < TgtPos.poses.size(); ++i)
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
	TargetsAdded = true;
}

void PathPlanningNode::AddShipToPtsList(const nav_msgs::msg::Odometry &ShipPos)
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
	odometry_Subscription_.reset();
}

static void AddPointToPolygon(sPolygon *polygon, std::vector<sPoint> *PointList, const std::string &PointString, const int PolygonId)
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
	ObstaclesAdded = true;
	return (0);	
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	const auto node = std::make_shared<PathPlanningNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return (0);
}
