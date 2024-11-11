#include <queue>
#include "navigation/path_planning_node.hpp"

std::pair<double, std::vector<int>> PathPlanningNode::Dijkstra(int start, const int end, std::vector<std::vector<double>> &Graph) const
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
		const int Point = PriorityQueue.top().second;
		const double CurrentDistance = PriorityQueue.top().first;
		PriorityQueue.pop();

		if (Point == end) //If reach the end
			break;

		if (CurrentDistance != dist[Point]) //If a best path was found for this point
			continue ;

		for (int i = 0; i < Graph[Point].size(); ++i)
		{
			//Distance vers le point voisin i
			const double WeightNeighbour = Graph[Point][i];

			//If shortest path is found
			if (WeightNeighbour != std::numeric_limits<double>::infinity() &&
				dist[Point] + WeightNeighbour < dist[i])
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