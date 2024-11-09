/*
Creating the graph for path finding.
try to link each point with each others:
	Step 1: Check if the point is ne himself
	Step 2: Check if the points aren't in the same polygon and not adjacent
	Step 3: Check if the line don't cross any polygon
	If the line is valid : calculate the distance between the two points.
	else : the distance between the two points will be infinite   
*/

#include "navigation/path_planning_node.hpp"

static double FindDistancePoints(const sPoint &point1, const sPoint &point2)
{
	const double x = point2.x - point1.x;
	const double y = point2.y - point1.y;

	return (sqrt((x * x) + (y * y)));
}

void PathPlanningNode::InitGraphSize(const size_t size, std::vector<std::vector<double>> &Graph)
{
	//Resize the full graph
	Graph.resize(size);
	for (int i = 0; i < size; ++i)
		Graph[i].resize(size);

	//Resize the graph with only objectives
	GraphObjectives.resize(NbObjectives);
	for (int i = 0; i < NbObjectives; ++i)
		GraphObjectives[i].resize(NbObjectives);
}

bool PathPlanningNode::IsPointsAdjacent(const sPoint &FirstPoint, const sPoint &SecondPoint) const
{
	const sPolygon Polygon = ObstacleList[FirstPoint.PolygonId];
	const std::vector<sPoint>::const_iterator it = std::find(Polygon.Points.begin(), Polygon.Points.end(), FirstPoint);
	std::vector<sPoint>::const_iterator NextPoint = it + 1;
	std::vector<sPoint>::const_iterator PreviousPoint;

	if (NextPoint == Polygon.Points.end())
		NextPoint = Polygon.Points.begin();
	if (it == Polygon.Points.begin())
		PreviousPoint = Polygon.Points.end() - 1;
	else
		PreviousPoint = it - 1;
	if (*PreviousPoint == SecondPoint ||  *NextPoint == SecondPoint)
		return (true);
	return (false);
}

void PathPlanningNode::CreateGraph(std::vector<std::vector<double>> &Graph, std::vector<sPoint> &PointList)
{
	const size_t NBPoint = PointList.size();
	InitGraphSize(NBPoint, Graph);

	//Connect all points together
	for (int i = 0; i < NBPoint; ++i)
	{
		for (int j = i; j < NBPoint; ++j)
		{
			//If the points are the same point
			if (i == j)
				Graph[i][j] = std::numeric_limits<double>::infinity();

			//If the points are in the same polygon but not adjacent
			else if (PointList[i].PolygonId != -1 && PointList[i].PolygonId == PointList[j].PolygonId 
				&& IsPointsAdjacent(PointList[i], PointList[j]) == false)
			{
				Graph[i][j] = std::numeric_limits<double>::infinity();
				Graph[j][i] = std::numeric_limits<double>::infinity();
			}

			//If the line cross an obstacle
			else if (CheckInterPoly(PointList[i], PointList[j]) == true)
			{
				Graph[i][j] = std::numeric_limits<double>::infinity();
				Graph[j][i] = std::numeric_limits<double>::infinity();
			}

			//If don't cross an obstacle calculate distance
			else
			{
				const double distance = FindDistancePoints(PointList[i], PointList[j]);
				Graph[i][j] = distance;
				Graph[j][i] = distance;
			}
		}
	}
}