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
#include "navigation/srv/path.hpp"
#include "std_msgs/msg/float64.hpp"
#include <opencv2/opencv.hpp>
#include <cstdarg>

#define MINIMAP_SIZE 900

class VisualizationNode : public rclcpp::Node
{
public:
	VisualizationNode();
	
    void launch();
    

private:

    rclcpp::Client<navigation::srv::Path>::SharedPtr Visual_Client_;

	rclcpp::TimerBase::SharedPtr timer_;

	void service_response_callback(
    rclcpp::Client<navigation::srv::Path>::SharedFuture future);

	void VisualRegister(geometry_msgs::msg::PoseArray msg);
	void PathPlan(nav_msgs::msg::Path path);

	int	_nb_turbines;

	float _camAngle;

	int _center;

	void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
	void cameraCallback(const std_msgs::msg::Float64::SharedPtr msg);
	void wayPointCallback();

	void addPathPoint(nav_msgs::msg::Path& path, double x, double y);

	void createCircle(cv::Mat & mat, double x, double y, int radius);
	void createLine(cv::Mat & mat, double x1, double y1, double x2, double y2);
	void createPolygon(cv::Mat &mat, size_t pointsNumber, ...);

    // Subscribtions
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr camera_subscription_;

	// Paths
	nav_msgs::msg::Path FullPath;
	std::vector<cv::Point> ActualPath;

	// Timers
	rclcpp::TimerBase::SharedPtr point_timer_;

};

#endif // VISUALIZATION_NODE_HPP