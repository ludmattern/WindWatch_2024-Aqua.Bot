// navigation_node.cpp
#include "navigation/obstacle_avoidance_node.hpp"

ObstacleAvoidanceNode::ObstacleAvoidanceNode() : Node("obstacle_avoidance_node")
{
    RCLCPP_INFO(this->get_logger(), "Obstacle Avoidance Node has started");
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleAvoidanceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
