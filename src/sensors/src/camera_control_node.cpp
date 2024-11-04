#include "sensors/camera_control_node.hpp"

CameraControlNode::CameraControlNode()
	: Node("camera_control_node"),
	targetProcessed_(false),
	previous_theta_(0),
	inspecting_(false)
{
	action_server_ = rclcpp_action::create_server<CameraControl>(
		this,
		"camera_control",
		std::bind(&CameraControlNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&CameraControlNode::handle_cancel, this, std::placeholders::_1),
		std::bind(&CameraControlNode::handle_accepted, this, std::placeholders::_1));

	RCLCPP_INFO(this->get_logger(), "Camera Control Action Server has started");

	camera_pub_ = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/main_camera_sensor/pos", 10);

	odometrySub_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"/mission/odometry", 10, std::bind(&CameraControlNode::boatPoseCallback, this, std::placeholders::_1));

	imageFeedSub_ = this->create_subscription<sensor_msgs::msg::Image>(
		"/aquabot/sensors/cameras/main_camera_sensor/optical/image_raw", 10,
		std::bind(&CameraControlNode::scanQRCode, this, std::placeholders::_1));

	RCLCPP_INFO(this->get_logger(), "Camera Control Node has started");
}

rclcpp_action::GoalResponse CameraControlNode::handle_goal(
	const rclcpp_action::GoalUUID &uuid,
	std::shared_ptr<const CameraControl::Goal> goal)
{
	RCLCPP_INFO(this->get_logger(), "Received goal request with target x: %f, y: %f", goal->target.x, goal->target.y);
	(void)uuid;

	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CameraControlNode::handle_cancel(
	const std::shared_ptr<GoalHandleCameraControl> goal_handle)
{
	RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
	(void)goal_handle;

	return rclcpp_action::CancelResponse::ACCEPT;
}

void CameraControlNode::handle_accepted(const std::shared_ptr<GoalHandleCameraControl> goal_handle)
{
	std::thread{std::bind(&CameraControlNode::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void CameraControlNode::execute(const std::shared_ptr<GoalHandleCameraControl> goal_handle)
{
	RCLCPP_INFO(this->get_logger(), "Executing goal");

	const auto goal = goal_handle->get_goal();
	auto feedback = std::make_shared<CameraControl::Feedback>();
	auto result = std::make_shared<CameraControl::Result>();

	// Initialize inspection
	{
		std::lock_guard<std::mutex> lock(mutex_);
		currentTarget_ = goal->target;
		inspecting_ = true;
		targetProcessed_ = false;
		previous_theta_ = 0.0;
	}

	feedback->feedback = "Starting inspection";
	goal_handle->publish_feedback(feedback);

	while (rclcpp::ok())
	{
		{
			std::lock_guard<std::mutex> lock(mutex_);
			if (targetProcessed_)
				break;
		}

		if (goal_handle->is_canceling())
		{
			result->id = 0; // Set appropriate values
			result->state.data = false;
			result->qrcodedata.data = "";
			result->orientation.data = 0.0;
			goal_handle->canceled(result);
			RCLCPP_INFO(this->get_logger(), "Goal canceled");
			{
				std::lock_guard<std::mutex> lock(mutex_);
				inspecting_ = false;
				targetProcessed_ = false;
			}
			controlCamera(0.0);
			return;
		}

		// Provide feedbak
		feedback->feedback = "Scanning for QR code...";
		goal_handle->publish_feedback(feedback);

		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	{
		std::lock_guard<std::mutex> lock(mutex_);
		result->id = id_.data;
		result->state = state_;
		result->qrcodedata = QRCodeData_;
		result->orientation = orientation_;

		inspecting_ = false;
		targetProcessed_ = false;
		previous_theta_ = 0.0;
	}
	controlCamera(0.0);

	goal_handle->succeed(result);
	RCLCPP_INFO(this->get_logger(), "Goal succeeded");
}

void CameraControlNode::boatPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (!inspecting_)
		return;

	boat_position_ = msg->pose.pose.position;
	auto orientation = msg->pose.pose.orientation;

	tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
	tf2::Matrix3x3(q).getRPY(roll_, pitch_, yaw_); // We mainly use yaw here

	targetPoseCallback();
}

void CameraControlNode::targetPoseCallback(void)
{
	double dx = currentTarget_.x - boat_position_.x;
	double dy = currentTarget_.y - boat_position_.y;

	double theta = atan2(dy, dx);

	double theta_relative = theta - yaw_;

	if (theta_relative - this->previous_theta_ > M_PI)
		theta_relative -= 2 * M_PI;
	else if (this->previous_theta_ - theta_relative > M_PI)
		theta_relative += 2 * M_PI;

	previous_theta_ = theta_relative;

	controlCamera(theta_relative);
}

void CameraControlNode::controlCamera(double angle_in_radians)
{
	auto camera_msg = std_msgs::msg::Float64();
	camera_msg.data = angle_in_radians;
	camera_pub_->publish(camera_msg);
}

void CameraControlNode::scanQRCode(const sensor_msgs::msg::Image::SharedPtr msg)
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (targetProcessed_ || !inspecting_)
		return;

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
	}
	catch (cv_bridge::Exception &e)
	{
		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
		return;
	}

	cv::Mat img = cv_ptr->image;

	if (img.empty())
	{
		RCLCPP_ERROR(this->get_logger(), "Empty image received");
		return;
	}

	int croppedWidth = img.cols / 3;
	int croppedHeight = img.rows / 3;

	int x = (img.cols - croppedWidth) / 2;
	int y = (img.rows - croppedHeight) / 2;

	cv::Rect roi(x, y, croppedWidth, croppedHeight);

	cv::Mat croppedImage = img(roi);

	cv::Mat gray;
	cv::cvtColor(croppedImage, gray, cv::COLOR_BGR2GRAY);

	zbar::ImageScanner scanner;
	scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

	zbar::Image zbarImage(gray.cols, gray.rows, "Y800", gray.data, gray.cols * gray.rows);

	int n = scanner.scan(zbarImage);

	if (n > 0)
	{
		auto symbol = zbarImage.symbol_begin();
		std::string decodedText = symbol->get_data();
		RCLCPP_INFO(this->get_logger(), "QR code text: %s", decodedText.c_str());

		QRCodeData_.data = decodedText;
		orientation_.data = 0.0; // Temporary
		id_.data = std::stoi(QRCodeData_.data.substr(QRCodeData_.data.find_first_of(DIGITS)));
		state_.data = (QRCodeData_.data.find("KO") == std::string::npos);

		targetProcessed_ = true;
	}
	else
	{
		RCLCPP_WARN(this->get_logger(), "No QR code detected in the image.");
	}
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<CameraControlNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
