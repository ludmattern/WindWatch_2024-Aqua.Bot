#ifndef CAMERA_CONTROL_NODE_HPP
#define CAMERA_CONTROL_NODE_HPP

// Camera control node declarations
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include <mutex>
#include <condition_variable>
#include "sensors/srv/camera_control_serv.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <zbar.h>

#define DIGITS "0123456789"

class CameraControlNode : public rclcpp::Node
{
public:
    CameraControlNode();

private:
	void boatPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
	void targetPoseCallback(void);
	void controlCamera(double angle_in_radians);
	void scanQRCode(const sensor_msgs::msg::Image::SharedPtr msg);

    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySub_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr ImageFeedSub_;
	// add the point to fix

    // Publisher
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr camera_pub_;

	// Position and orientation (in rad) of the boat
	geometry_msgs::msg::Point boat_position_;
	double roll_, pitch_, yaw_;

	// Previous angle to cumulate with the new one
	double previous_theta_;
	bool targetProcessed_;

	//CameraControlService_
	rclcpp::Service<sensors::srv::CameraControlServ>::SharedPtr CameraControlService_;
	void ServerCallback(const std::shared_ptr<sensors::srv::CameraControlServ::Request> request,
	const std::shared_ptr<sensors::srv::CameraControlServ::Response> response);

	geometry_msgs::msg::Point currentTarget_;
	std::mutex targetMutex_;
	std::condition_variable targetCondition_;
	
	//QR code processing data
	std_msgs::msg::Float32 orientation_;
	std_msgs::msg::String QRCodeData_;
	std_msgs::msg::Int32 id_;
	std_msgs::msg::Bool state_;
};


#endif // CAMERA_CONTROL_NODE_HPP
