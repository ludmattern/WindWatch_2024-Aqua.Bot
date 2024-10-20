#include "mission_manager/target_manager_node.hpp"

TargetManagerNode::TargetManagerNode() : Node("target_manager_node")
{
    RCLCPP_INFO(this->get_logger(), "Target Manager Node has started");
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TargetManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
