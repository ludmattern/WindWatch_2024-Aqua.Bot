#ifndef PATH_PLANNING_NODE_HPP
#define PATH_PLANNING_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"

class PathPlanningNode : public rclcpp::Node
{
    public:
        PathPlanningNode();

    private:

        std::vector<std::pair<double, double>> PointList;
        std::vector<std::vector<double>> Graph;

        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr tgtPos_Subscirption_;
        rclcpp::Subscription<>

        void FillPtsList(void);
};

#endif // PATH_PLANNING_NODE_HPP
