#include "navigation/propulsion_control_node.hpp"
#include <algorithm>
#include <cmath>

PropulsionControlNode::PropulsionControlNode()
: Node("propulsion_control_node")
{
	// Déclaration et récupération des paramètres
	this->declare_parameter<double>("distance_between_thrusters", 1.0); 
	this->declare_parameter<double>("steering_gain", 1.0);             
	this->declare_parameter<double>("max_thrust", 5000.0);             
	this->declare_parameter<double>("max_steering_angle", 0.78);       	
	this->declare_parameter<double>("scale_factor", 810.0);            
	this->declare_parameter<double>("min_linear_speed", 0.0);          
	this->declare_parameter<double>("rotation_gain", 1.0);          

	this->get_parameter("distance_between_thrusters", distance_between_thrusters_);
	this->get_parameter("steering_gain", steering_gain_);
	this->get_parameter("max_thrust", max_thrust_);
	this->get_parameter("max_steering_angle", max_steering_angle_);
	this->get_parameter("scale_factor", scale_factor_);
	this->get_parameter("min_linear_speed", min_linear_speed_);
	this->get_parameter("rotation_gain", rotation_gain_);

	// Souscription au topic /propulsion/command
	cmd_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
		"/propulsion/command", 10,
		std::bind(&PropulsionControlNode::cmdCallback, this, std::placeholders::_1));

	// Publications des commandes de poussée et de braquage
	left_thruster_pos_pub_ = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/left/pos", 10);
	right_thruster_pos_pub_ = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/right/pos", 10);
	left_thruster_thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/left/thrust", 10);
	right_thruster_thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/right/thrust", 10);

	RCLCPP_INFO(this->get_logger(), "Propulsion Control Node has started");
}

void PropulsionControlNode::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
	double linear_velocity = msg->linear.x;
	double angular_velocity = msg->angular.z;

	const double epsilon = 0.1;
	double thrust_left, thrust_right;
	double steering_angle_left, steering_angle_right;

	if (std::abs(angular_velocity) > 0.0 && std::abs(linear_velocity) < epsilon)
		computeRotationOnSpot(angular_velocity, thrust_left, thrust_right, steering_angle_left, steering_angle_right);
	else
		computeMovingRotation(linear_velocity, angular_velocity, thrust_left, thrust_right, steering_angle_left, steering_angle_right);

	publishCommands(thrust_left, thrust_right, steering_angle_left, steering_angle_right);
}

void PropulsionControlNode::computeRotationOnSpot(double angular_velocity, double& thrust_left, double& thrust_right, 
												double& steering_angle_left, double& steering_angle_right)
{
	double rotation_thrust = std::abs(angular_velocity) * rotation_gain_;
	thrust_left = (angular_velocity > 0) ? rotation_thrust : -rotation_thrust;
	thrust_right = -thrust_left;

	thrust_left = std::clamp(thrust_left, -max_thrust_, max_thrust_);
	thrust_right = std::clamp(thrust_right, -max_thrust_, max_thrust_);

	steering_angle_left = max_steering_angle_ * (angular_velocity >= 0 ? 1 : -1);
	steering_angle_right = -steering_angle_left;
}

void PropulsionControlNode::computeMovingRotation(double linear_velocity, double angular_velocity, 
												double& thrust_left, double& thrust_right,
												double& steering_angle_left, double& steering_angle_right)
{
	double rotation_compensation = (angular_velocity > 0.0) ? 1.0 : 1.0;
	double base_thrust = linear_velocity * scale_factor_;

	if (angular_velocity > 0.0)
	{
		thrust_left = (base_thrust - (distance_between_thrusters_ / 2.0) * angular_velocity * scale_factor_) * rotation_compensation;
		thrust_right = base_thrust + (distance_between_thrusters_ / 2.0) * angular_velocity * scale_factor_;
	}
	else if (angular_velocity < 0.0)
	{
		thrust_left = base_thrust + (distance_between_thrusters_ / 2.0) * angular_velocity * scale_factor_;
		thrust_right = (base_thrust - (distance_between_thrusters_ / 2.0) * angular_velocity * scale_factor_) * rotation_compensation;
	}
	else
		thrust_left = thrust_right = base_thrust;

	thrust_left = std::clamp(thrust_left, -max_thrust_, max_thrust_);
	thrust_right = std::clamp(thrust_right, -max_thrust_, max_thrust_);

	steering_angle_left = steering_gain_ * angular_velocity;
	steering_angle_right = -steering_gain_ * angular_velocity;
	steering_angle_left = std::clamp(steering_angle_left, -max_steering_angle_, max_steering_angle_);
	steering_angle_right = std::clamp(steering_angle_right, -max_steering_angle_, max_steering_angle_);
}

void PropulsionControlNode::publishCommands(double thrust_left, double thrust_right, 
											double steering_angle_left, double steering_angle_right)
{
	std_msgs::msg::Float64 left_pos_msg, right_pos_msg, left_thrust_msg, right_thrust_msg;
	
	left_pos_msg.data = steering_angle_left;
	right_pos_msg.data = steering_angle_right;
	left_thrust_msg.data = thrust_left;
	right_thrust_msg.data = thrust_right;

	left_thruster_pos_pub_->publish(left_pos_msg);
	right_thruster_pos_pub_->publish(right_pos_msg);
	left_thruster_thrust_pub_->publish(left_thrust_msg);
	right_thruster_thrust_pub_->publish(right_thrust_msg);

	RCLCPP_INFO(this->get_logger(), "Published Thruster Commands: Left Pos: %.3f rad, Right Pos: %.3f rad, "
									"Left Thrust: %.3f, Right Thrust: %.3f",
				steering_angle_left, steering_angle_right, thrust_left, thrust_right);
}



int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<PropulsionControlNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
