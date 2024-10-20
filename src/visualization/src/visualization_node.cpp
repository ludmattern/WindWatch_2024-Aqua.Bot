// obstacle_avoidance_node.cpp
#include "visualization/visualization_node.hpp"

VisualizationNode::VisualizationNode() : Node("visualization_node")
{
    RCLCPP_INFO(this->get_logger(), "Visualization Node has started");
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisualizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
