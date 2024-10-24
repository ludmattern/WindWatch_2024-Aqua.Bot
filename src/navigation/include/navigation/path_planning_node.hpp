#ifndef PATH_PLANNING_NODE_HPP
#define PATH_PLANNING_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "Point.hpp"
#include "sensors/srv/target_positions.hpp"
#include "Polygon.hpp"

#define OBSTACLE_FILE "src/navigation/ObstaclePos"

class PathPlanningNode : public rclcpp::Node
{
    public:
        PathPlanningNode();

        //Client for target position
        rclcpp::Client<sensors::srv::TargetPositions>::SharedPtr TgtPos_Client_;

        void AddTgtToPtsList(geometry_msgs::msg::PoseArray TgtPos);
        int AddObstaclePtsList(void);
        void CreateGraph(void);

    private:

        std::vector<sPoint> PointList;
        std::vector<sPolygon> ObstacleList;
        std::vector<std::vector<double>> Graph;

        //Subscription of targets positions and ship pos
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_Subscription_;

        void AddShipToPtsList(nav_msgs::msg::Odometry ShipPos);
        void InitGraphSize(int size);
        bool CheckInterPoly(sPoint FirstPoint, sPoint SecondPoint);

        bool ShipAdded;
};

#endif // PATH_PLANNING_NODE_HPP
