// obstacle_avoidance_node.cpp

#include "visualization/visualization_node.hpp"

VisualizationNode::VisualizationNode() : Node("visualization_node")
{
    // Subscriptions
    odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/mission/odometry", 10,
        std::bind(&VisualizationNode::odometryCallback, this, std::placeholders::_1));

    marker_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/minimaps/test", 10);

    geometry_msgs::msg::PointStamped Point;

	Point.header.frame_id = "odom";
	Point.header.stamp = this->get_clock()->now();
	Point.point.set__x(0.0);
	Point.point.set__y(0.0);
	Point.point.set__z(0.0);
	marker_publisher_->publish(Point);

	RCLCPP_INFO(this->get_logger(), "Visualization Node has started");
}

void VisualizationNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    return ;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisualizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
