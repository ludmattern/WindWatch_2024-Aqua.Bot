#ifndef PATH_PLANNING_NODE_HPP
#define PATH_PLANNING_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "Point.hpp"
#include "sensors/srv/target_positions.hpp"
#include "Polygon.hpp"
#include "navigation/srv/path.hpp"
#include "navigation/srv/path_last.hpp"

#define OBSTACLE_FILE "src/navigation/ObstaclePos"

class PathPlanningNode : public rclcpp::Node
{
	public:
		PathPlanningNode();

		void launch(void);

	private:

		rclcpp::TimerBase::SharedPtr timer_;
		void serviceResponseCallback(rclcpp::Client<sensors::srv::TargetPositions>::SharedFuture future);

		//List of all the points of the graph
		std::vector<sPoint> PointList;

		//List of all the polygons that represent the obtacles
		std::vector<sPolygon> ObstacleList;

		//Graph with all the points
		std::vector<std::vector<double>> Graph;

		//Graph with only the objectives
		std::vector<std::vector<std::pair<double, std::vector<int>>>> GraphObjectives;

		//Subscription of targets positions and ship pos
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_Subscription_;

		//Service for the Path to all targets
		rclcpp::Service<navigation::srv::Path>::SharedPtr PathService_;
		rclcpp::CallbackGroup::SharedPtr callback_group_path;
		
		void ServerCallback(const std::shared_ptr<navigation::srv::Path::Request> &request,
			const std::shared_ptr<navigation::srv::Path::Response> &response) const;

		//Service for the Path to the last target
		rclcpp::Service<navigation::srv::PathLast>::SharedPtr PathLastService_;
		rclcpp::CallbackGroup::SharedPtr callback_group_pathlast;

		void ServerPathLastCallback(const std::shared_ptr<navigation::srv::PathLast::Request> &request,
			const std::shared_ptr<navigation::srv::PathLast::Response> &response);


		//Client for target position
		rclcpp::Client<sensors::srv::TargetPositions>::SharedPtr TgtPos_Client_;
		void CreateGraph(std::vector<std::vector<double>> &Graph, std::vector<sPoint> &PointList);

		void AddTgtToPtsList(const geometry_msgs::msg::PoseArray &TgtPos);
		int AddObstaclePtsList(void);
		void AddShipToPtsList(const nav_msgs::msg::Odometry &ShipPos);

		void InitGraphSize(const size_t size, std::vector<std::vector<double>> &Graph);
		bool CheckInterPoly(const sPoint &FirstPoint, const sPoint &SecondPoint) const;
		bool IsPointsAdjacent(const sPoint &FirstPoint, const sPoint &SecondPoint) const;

		void CreateObjectivesGraph(void);
		std::pair<double, std::vector<int>> Dijkstra(int start, const int end, std::vector<std::vector<double>> &Graph) const;
		double tsp(int pos, int mask, std::vector<std::vector<double>> &dp, std::vector<std::vector<int>> &next);
		nav_msgs::msg::Path CreatePath(void);
		nav_msgs::msg::Path GetPath(const std::vector<std::vector<int>> &next);

		nav_msgs::msg::Path Path;
		geometry_msgs::msg::PoseArray Targets;

		bool ShipAdded;
		bool TargetsAdded;
		bool ObstaclesAdded;
		bool PathFinded;
		int NbObjectives;
};

bool CheckInterLines(const sPoint &p1, const sPoint &q1, const sPoint &p2, const sPoint &q2);

#endif // PATH_PLANNING_NODE_HPP
