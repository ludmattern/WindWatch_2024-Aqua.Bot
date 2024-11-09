#include <fstream>
#include "navigation/path_planning_node.hpp"

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