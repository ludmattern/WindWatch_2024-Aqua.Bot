// sensor_fusion_node.cpp
#include "sensors/sensor_fusion_node.hpp"

SensorFusionNode::SensorFusionNode() : Node("sensor_fusion_node")
{
    RCLCPP_INFO(this->get_logger(), "Sensor Fusion Node has started");
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
