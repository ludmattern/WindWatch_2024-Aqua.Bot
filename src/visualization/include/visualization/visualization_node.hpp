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

	void pointsPublisher();
	void setCoordinates(geometry_msgs::msg::PointStamped *Point, double x, double z);

    // Subscribtions
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;

	// Publishers
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr light_house_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr island_1_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr island_2_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr rock_island_0_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr rock_island_1_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr rock_0_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr rock_1_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr rock_2_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr rock_3_publisher_;

	// Timers
	rclcpp::TimerBase::SharedPtr point_timer_;

};

#endif // VISUALIZATION_NODE_HPP