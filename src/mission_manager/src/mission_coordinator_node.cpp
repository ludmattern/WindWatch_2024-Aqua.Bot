#include "mission_manager/mission_coordinator_node.hpp"

MissionCoordinatorNode::MissionCoordinatorNode() : Node("mission_coordinator_node")
{
    RCLCPP_INFO(this->get_logger(), "Mission Coordinator Node has started");
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionCoordinatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
