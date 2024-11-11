#include "navigation/path_planning_node.hpp"

static int orientation(const sPoint &p, const sPoint &q, const sPoint &r)
{
	//Calculate orientation 
	const double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);;

	if (val == 0)
		return 0; //collinear
	else if (val > 0)
		return (1); //clockwise
	else
		return (2); //Counter clockwise
}

static bool onSegment(const sPoint &p, const sPoint &q, const sPoint &r)
{
	//Check if q is on pr
	if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
		q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
		return (true);
	return (false);
}

bool CheckInterLines(const sPoint &p1, const sPoint &q1, const sPoint &p2, const sPoint &q2)
{
	const int o1 = orientation(p1, q1, p2);
	const int o2 = orientation(p1, q1, q2);
	const int o3 = orientation(p2, q2, p1);
	const int o4 = orientation(p2, q2, q1);

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