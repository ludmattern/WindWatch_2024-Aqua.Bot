#ifndef VISUALIZATION_NODE_HPP
#define VISUALIZATION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <array>
#include "tf2/LinearMath/Quaternion.h"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "sensors/srv/target_positions.hpp"
#include "std_msgs/msg/float64.hpp"

class VisualizationNode : public rclcpp::Node
{
public:
	VisualizationNode();

    rclcpp::Client<sensors::srv::TargetPositions>::SharedPtr Visual_Client_;
    void VisualRegister(geometry_msgs::msg::PoseArray msg);

private:

	int	_nb_turbines;
	visualization_msgs::msg::Marker _markers[30];

	void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
	void cameraCallback(const std_msgs::msg::Float64::SharedPtr msg);

	void pointsPublisher();
	void addPolygonPoint(geometry_msgs::msg::PolygonStamped& Polygon, double x, double y);
	void addPathPoint(nav_msgs::msg::Path& path, double x, double y); // Test purpose function
	void setCoordinates(geometry_msgs::msg::PointStamped *Point, double x, double z);

    // Subscribtions
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr camera_subscription_;

	// Publishers
	std::array<rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr, 15> turbines_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr light_house_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr island_1_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr island_2_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr rock_island_0_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr rock_island_1_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr rock_0_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr rock_1_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr rock_2_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr rock_3_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr camera_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_publisher_1_;
	rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_publisher_2_;
	rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_publisher_3_;
	rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_publisher_4_;
	rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_publisher_5_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr full_path_publisher_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actual_path_publisher_;

	// Obstacles
	geometry_msgs::msg::PointStamped LightHouse;
	geometry_msgs::msg::PointStamped Island1;
	geometry_msgs::msg::PointStamped Island2;
	geometry_msgs::msg::PointStamped RockIsland0;
	geometry_msgs::msg::PointStamped RockIsland1;
	geometry_msgs::msg::PointStamped Rock0;
	geometry_msgs::msg::PointStamped Rock1;
	geometry_msgs::msg::PointStamped Rock2;
	geometry_msgs::msg::PointStamped Rock3;
	geometry_msgs::msg::PolygonStamped Polygon1;
	geometry_msgs::msg::PolygonStamped Polygon2;
	geometry_msgs::msg::PolygonStamped Polygon3;
	geometry_msgs::msg::PolygonStamped Polygon4;
	geometry_msgs::msg::PolygonStamped Polygon5;
	geometry_msgs::msg::PoseWithCovarianceStamped Camera;
	nav_msgs::msg::Path FullPath;
	nav_msgs::msg::Path ActualPath;

	// Timers
	rclcpp::TimerBase::SharedPtr point_timer_;

};

#endif // VISUALIZATION_NODE_HPP