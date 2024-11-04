#include "navigation/path_planning_node.hpp"

void PathPlanningNode::CreateObjectivesGraph(void)
{
	for (int i = 0; i < NbObjectives; ++i)
	{
		for (int j = i; j < NbObjectives; ++j)
		{
			if (i == j)
				GraphObjectives[i][j].first = std::numeric_limits<double>::infinity();

			else
			{
				std::pair<double, std::vector<int>> Path = Dijkstra(i, j);

				GraphObjectives[i][j] = Path;

				std::reverse(Path.second.begin(), Path.second.end());
				GraphObjectives[j][i] = Path;
			}
		}
	}
}

std::vector<sPoint> PathPlanningNode::GetPath(std::vector<std::vector<int>> &next)
{
	std::vector<int> ObjectivesOrder;
	std::vector<sPoint> Path;
	int pos = 0;
	int mask = 1 << 0;

	//Recreate the path made by the TSP in ObjectivesOrder
	while (mask != ((1 << NbObjectives) - 1))
	{
		ObjectivesOrder.push_back(pos);
		const int next_pos = next[mask][pos];
		mask |= (1 << next_pos);
		pos = next_pos;
	}
	ObjectivesOrder.push_back(pos);

	//Fill path with all the point where the ship need to pass to pass through all the objectives
	for (int i = 1; i < ObjectivesOrder.size(); ++i)
	{
		for (int j = 0; j < GraphObjectives[ObjectivesOrder[i - 1]][ObjectivesOrder[i]].second.size(); ++j)
		{
			sPoint NextPoint;
			NextPoint.x = PointList[GraphObjectives[ObjectivesOrder[i - 1]][ObjectivesOrder[i]].second[j]].x;
			NextPoint.y = PointList[GraphObjectives[ObjectivesOrder[i - 1]][ObjectivesOrder[i]].second[j]].y;
			Path.push_back(NextPoint);
		}
	}
	return (Path);
}

std::vector<sPoint> PathPlanningNode::CreatePath(void)
{
	//Fill GraphObjectives
	CreateObjectivesGraph();

	//Init dp vector for the tsp
	std::vector<std::vector<double>> dp(1 << NbObjectives,
		std::vector<double>(NbObjectives, std::numeric_limits<double>::infinity()));

	//Init next vector for the tsp
	std::vector<std::vector<int>> next(1 << NbObjectives, std::vector<int>(NbObjectives, -1));

	//Find the shortest path with a TSP algorithm
	tsp(0, 1 << 0, dp, next);

	return(GetPath(next));
}