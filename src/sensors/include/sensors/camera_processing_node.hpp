#ifndef CAMERA_PROCESSING_NODE_HPP
#define CAMERA_PROCESSING_NODE_HPP

// Camera processing node declarations
#include <rclcpp/rclcpp.hpp>
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>

class CameraProcessingNode : public rclcpp::Node
{
public:
    CameraProcessingNode();

private:
	void boatPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
	void targetPoseCallback(void); // no channel for the point to fix
	void controlCamera(double angle_in_radians);

    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr boat_pose_sub_;
	// add the point to fix

    // Publisher
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr camera_pub_;

	// Position and orientation (in rad) of the boat
	geometry_msgs::msg::Point boat_position_;
	double roll_, pitch_, yaw_;
};


#endif // CAMERA_PROCESSING_NODE_HPP
