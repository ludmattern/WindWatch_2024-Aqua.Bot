#include "sensors/camera_processing_node.hpp"

CameraProcessingNode::CameraProcessingNode() : Node("camera_processing_node")
{
    RCLCPP_INFO(this->get_logger(), "Camera Processing Node has started");
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraProcessingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
