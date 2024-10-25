/*
Creating the graph for path finding.
try to link each point with each others:
	Step 1: Check if the point is ne himself
	Step 2: Check if the points aren't in the same polygon
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

void PathPlanningNode::InitGraphSize(const int size)
{
	Graph.resize(size);
	for (int i = 0; i < size; ++i)
		Graph[i].resize(size);
}

void PathPlanningNode::CreateGraph(void)
{
	const size_t NBPoint = PointList.size();
	InitGraphSize(NBPoint);

	//Connect all points together
	for (int i = 0; i < NBPoint; ++i)
	{
		for (int j = 0; j < NBPoint; ++j)
		{
			//If the points are the same point
			if (i == j)
				Graph[i][j] = std::numeric_limits<double>::infinity();

			//If the points are in the same polygon
			else if (PointList[i].PolygonId != -1 && PointList[i].PolygonId == PointList[j].PolygonId)
				Graph[i][j] = std::numeric_limits<double>::infinity();
			
			//If the line cross an obstacle
			else if (CheckInterPoly(PointList[i], PointList[j]) == true)
				Graph[i][j] = std::numeric_limits<double>::infinity();

			//If don't cross an obstacle calculate distance
			else
				Graph[i][j] = FindDistancePoints(PointList[i], PointList[j]);
		}
	}
}