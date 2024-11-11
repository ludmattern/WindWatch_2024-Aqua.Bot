#include "navigation/path_planning_node.hpp"

static bool CheckIfValidLine(const sPoint &p1, const sPoint &p2, const sPoint &p3, const sPoint &p4)
{
	//Check if p1p2 and p3p4 have a point in common
	if (p1 == p3)
		return (false);
	if (p1 == p4)
		return (false);
	if (p2 == p3)
		return (false);
	if (p2 == p4)
		return (false);
	return (true);
}

bool PathPlanningNode::CheckInterPoly(const sPoint &FirstPoint, const sPoint &SecondPoint) const
{
	size_t i = 0;
	const size_t NbObstacle = ObstacleList.size();

	//Points adjacent in a polygon
	if (FirstPoint.PolygonId != -1 && FirstPoint.PolygonId == SecondPoint.PolygonId)
		return (false);

	//Check if the line cross any polygon side
	while (i < NbObstacle)
	{
		const size_t NbPoint = ObstacleList[i].Points.size();
		for (size_t j = 0; j < NbPoint; ++j)
		{
			const size_t nextPoint = (j + 1) % NbPoint; //If j = end check with point 0

			//Dont check with sides of the point
			if (CheckIfValidLine(FirstPoint, SecondPoint, ObstacleList[i].Points[j], 
				ObstacleList[i].Points[nextPoint]) == true)
			{
				if (CheckInterLines(FirstPoint, SecondPoint, ObstacleList[i].Points[j], 
					ObstacleList[i].Points[nextPoint]) == true)
					return (true);
			}
		}
		++i;
	}
	return (false);
}