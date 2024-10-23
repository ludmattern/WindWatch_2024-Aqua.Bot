#ifndef PATH_PLANNING_NODE_HPP
#define PATH_PLANNING_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "Point.hpp"
#include "sensors/srv/target_positions.hpp"

class PathPlanningNode : public rclcpp::Node
{
    public:
        PathPlanningNode();

        //Client for target position
        rclcpp::Client<sensors::srv::TargetPositions>::SharedPtr TgtPos_Client_;

        void AddTgtToPtsList(geometry_msgs::msg::PoseArray TgtPos);

    private:

        std::vector<sPoint> PointList;
        std::vector<std::vector<double>> Graph;

        //Subscription of targets positions and ship pos
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_Subscription_;


        void SendServiceRequest(void);
        void AddShipToPtsList(nav_msgs::msg::Odometry ShipPos);
        void AddObstaclePtsList(void);

        bool ShipAdded;
};

#endif // PATH_PLANNING_NODE_HPP
