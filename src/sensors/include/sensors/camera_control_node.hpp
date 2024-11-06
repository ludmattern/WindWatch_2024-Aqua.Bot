#ifndef CAMERA_CONTROL_NODE_HPP
#define CAMERA_CONTROL_NODE_HPP

// Camera control node declarations
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensors/action/camera_control.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <cmath>
#include <zbar.h>

#define DIGITS "0123456789"

class CameraControlNode : public rclcpp::Node
{
public:
	using CameraControl = sensors::action::CameraControl;
	using GoalHandleCameraControl = rclcpp_action::ServerGoalHandle<CameraControl>;

	CameraControlNode();

private:
	rclcpp_action::Server<CameraControl>::SharedPtr action_server_;

	rclcpp_action::GoalResponse handle_goal(
		const rclcpp_action::GoalUUID & uuid,
		std::shared_ptr<const CameraControl::Goal> goal);

	rclcpp_action::CancelResponse handle_cancel(
		const std::shared_ptr<GoalHandleCameraControl> goal_handle);

	void handle_accepted(
		const std::shared_ptr<GoalHandleCameraControl> goal_handle);

	void execute(
		const std::shared_ptr<GoalHandleCameraControl> goal_handle);

	void boatPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
	void targetPoseCallback(void);
	void controlCamera(double angle_in_radians);
	void scanQRCode(const sensor_msgs::msg::Image::SharedPtr msg);
	void QRcodePose(cv::Mat image);

	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySub_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageFeedSub_;

	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr camera_pub_;

	geometry_msgs::msg::Point boat_position_;
	geometry_msgs::msg::Point camera_position_;
	double roll_, pitch_, yaw_;

	double previous_theta_;
	bool targetProcessed_;
	bool inspecting_;

	geometry_msgs::msg::Point currentTarget_;
	std::mutex mutex_; // Mutex for thread safety

	std_msgs::msg::Float32 orientation_;
	std_msgs::msg::String QRCodeData_;
	std_msgs::msg::Int32 id_;
	std_msgs::msg::Bool state_;

	double QrCodeDecoded_;
	int QrCodeVisible_;
	double FirstOrientationQrCode_;
	double LastOrientationQrCode_;
};

#endif // CAMERA_CONTROL_NODE_HPP
