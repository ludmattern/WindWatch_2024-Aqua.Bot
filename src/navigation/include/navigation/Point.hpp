#ifndef POINT_HPP
#define POINT_HPP

struct sPoint
{
	double x;
	double y;
	bool IsGoal;
	int PolygonId;

	bool operator==(const sPoint &point) const
	{
		return (x == point.x && y == point.y);
	}
};



#endif
