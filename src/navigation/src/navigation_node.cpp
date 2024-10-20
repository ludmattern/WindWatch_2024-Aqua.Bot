// navigation_node.cpp
#include "navigation/navigation_node.hpp"

NavigationNode::NavigationNode() : Node("navigation_node")
{
    RCLCPP_INFO(this->get_logger(), "Navigation Node has started");
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
