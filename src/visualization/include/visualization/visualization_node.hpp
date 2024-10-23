#ifndef VISUALIZATION_NODE_HPP
#define VISUALIZATION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include "geometry_msgs/msg/point_stamped.hpp"

class VisualizationNode : public rclcpp::Node
{
public:
	VisualizationNode();

private:

	void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Subscribtions
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;

	// Publishers
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr marker_publisher_;

};

#endif // VISUALIZATION_NODE_HPP