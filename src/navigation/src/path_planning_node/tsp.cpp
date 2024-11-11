#include "navigation/path_planning_node.hpp"

double PathPlanningNode::tsp(const int pos, const int mask, std::vector<std::vector<double>> &dp, std::vector<std::vector<int>> &next)
{
	//All objectives are visited
	if (mask == (1 << NbObjectives) - 1)
		return (0);

	//Solutions already memorised
	if (dp[mask][pos] != std::numeric_limits<double>::infinity())
		return (dp[mask][pos]);

	double Answer = std::numeric_limits<double>::infinity();

	for (int NextPoint = 0; NextPoint < NbObjectives; ++NextPoint)
	{
		if ((mask & (1 << NextPoint)) == 0) //If NextPoint isn't visited
		{
			double NewAnswer = GraphObjectives[pos][NextPoint].first + tsp(NextPoint, mask | (1 << NextPoint), dp, next);
			if (NewAnswer < Answer) //Better path found
			{
				Answer = NewAnswer;
				next[mask][pos] = NextPoint;
			}
		}
	}
	dp[mask][pos] = Answer;
	return (Answer);
}