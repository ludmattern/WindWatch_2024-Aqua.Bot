#include "sensors/camera_processing_node.hpp"

#include <iostream>

CameraProcessingNode::CameraProcessingNode() : Node("camera_processing_node")
{
	// Subscriber pour la position et orientation du bateau via nav_msgs::msg::Odometry
	boat_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"/boat/odometry", 10, std::bind(&CameraProcessingNode::boatPoseCallback, this, std::placeholders::_1));
	// Subscriber pour la position du point à suivre (cela reste un PoseStamped)
	// point_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
	// "/target/pose", 10, std::bind(&CameraControllerNode::targetPoseCallback, this, std::placeholders::_1));
	
	camera_pub_ = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/main_camera_sensor/pos", 10);

	while (true)
	{
		targetPoseCallback();
		sleep(1);
	}
	RCLCPP_INFO(this->get_logger(), "Camera Processing Node has started");
}

void CameraProcessingNode::boatPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Récupérer la position du bateau
	boat_position_ = msg->pose.pose.position;

	// Récupérer l'orientation du bateau sous forme de quaternion
	auto orientation = msg->pose.pose.orientation;

	// Convertir le quaternion en angles de roulis, tangage, lacet (RPY)
	tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
	tf2::Matrix3x3(q).getRPY(roll_, pitch_, yaw_);  // Ici, on récupère surtout le yaw (lacet)
}

void CameraProcessingNode::targetPoseCallback(void)
{
	// Calculer le vecteur directionnel vers le point à fixer
	double dx = 120 - boat_position_.x;
	double dy = -50 - boat_position_.y;

	// Calculer l'angle entre l'axe X global et le point cible
	double theta = atan2(dy, dx);

	// Calculer l'angle relatif à l'avant du bateau
	double theta_relative = theta - yaw_;

	// Normaliser l'angle entre -pi et pi
	if (theta_relative > M_PI) {
		theta_relative -= 2 * M_PI;
	} else if (theta_relative < -M_PI) {
		theta_relative += 2 * M_PI;
	}

	RCLCPP_INFO(this->get_logger(), "Angle relatif de la caméra : %f rad", theta_relative);

	// Ici, orienter la caméra en fonction de theta_relative
	controlCamera(theta_relative);
}

void CameraProcessingNode::controlCamera(double angle_in_radians)
{
	auto camera_msg = std_msgs::msg::Float64();

	camera_msg.data = angle_in_radians;
	camera_pub_->publish(camera_msg);
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<CameraProcessingNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
