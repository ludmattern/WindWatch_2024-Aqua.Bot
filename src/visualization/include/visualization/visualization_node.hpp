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
#include <opencv2/opencv.hpp>
#include <cstdarg>

#define MINIMAP_SIZE 900

class VisualizationNode : public rclcpp::Node
{
public:
	VisualizationNode();

    rclcpp::Client<sensors::srv::TargetPositions>::SharedPtr Visual_Client_;
    void VisualRegister(geometry_msgs::msg::PoseArray msg);

private:

	int	_nb_turbines;

	float _camAngle;

	void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
	void cameraCallback(const std_msgs::msg::Float64::SharedPtr msg);
	void wayPointCallback();

	void pointsPublisher();
	void addPolygonPoint(geometry_msgs::msg::PolygonStamped& Polygon, double x, double y);
	void addPathPoint(nav_msgs::msg::Path& path, double x, double y); // Test purpose function
	void setCoordinates(geometry_msgs::msg::PointStamped *Point, double x, double z);

	void createCircle(cv::Mat & mat, double x, double y, int radius);
	void createLine(cv::Mat & mat, double x1, double y1, double x2, double y2);
	void createPolygon(cv::Mat &mat, size_t pointsNumber, ...);

    // Subscribtions
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr camera_subscription_;

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