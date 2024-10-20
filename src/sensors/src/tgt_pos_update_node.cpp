// tgt_pos_update_node.cpp
#include "sensors/tgt_pos_update_node.hpp"

TgtPosUpdateNode::TgtPosUpdateNode() : Node("tgt_pos_update_node")
{
    RCLCPP_INFO(this->get_logger(), "TGT Pos Update Node has started");
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TgtPosUpdateNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
