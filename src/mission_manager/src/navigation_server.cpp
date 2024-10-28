// src/navigation_server.cpp

#include "mission_manager/navigation_server.hpp"
#include <algorithm>
#include <limits>
#include <iostream>
#include <cmath>

using namespace std::chrono_literals;

NavigationServer::NavigationServer()
: Node("navigation_server"),
odom_received_(false),
initial_distance_to_goal_(0.0),
estimated_disturbance_angular_(0.0),
last_waypoint_index_(std::numeric_limits<size_t>::max()),
starting_point_{0.0, 0.0, false},
current_linear_speed_(0.0)
{
	action_server_ = rclcpp_action::create_server<Navigation>(
		this,
		"navigation",
		std::bind(&NavigationServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&NavigationServer::handle_cancel, this, std::placeholders::_1),
		std::bind(&NavigationServer::handle_accepted, this, std::placeholders::_1)
	);

	cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
		"/propulsion/command", 10);

	odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"/mission/odometry", 10,
		std::bind(&NavigationServer::odomCallback, this, std::placeholders::_1)
	);

	this->declare_parameter<double>("Kp_linear", 0.5);
	this->declare_parameter<double>("Ki_linear", 0.0);
	this->declare_parameter<double>("Kd_linear", 0.0);
	this->declare_parameter<double>("Kp_angular", 0.4);
	this->declare_parameter<double>("Ki_angular", 0.0);
	this->declare_parameter<double>("Kd_angular", 0.0);
	this->declare_parameter<double>("Kd_disturbance", 0.0);
	this->declare_parameter<double>("position_tolerance", 2.0);
	this->declare_parameter<double>("control_loop_rate", 20.0);
	this->declare_parameter<double>("min_linear_speed", 0.0);
	this->declare_parameter<double>("max_linear_speed", 6.0);
	this->declare_parameter<double>("max_angular_speed", 0.5);
	this->declare_parameter<double>("max_acceleration", 1.0); // m/s²
	this->declare_parameter<double>("max_deceleration", 1.0); // m/s²

	this->get_parameter("Kp_linear", Kp_linear_);
	this->get_parameter("Ki_linear", Ki_linear_);
	this->get_parameter("Kd_linear", Kd_linear_);
	this->get_parameter("Kp_angular", Kp_angular_);
	this->get_parameter("Ki_angular", Ki_angular_);
	this->get_parameter("Kd_angular", Kd_angular_);
	this->get_parameter("Kd_disturbance", Kd_disturbance_);
	this->get_parameter("position_tolerance", position_tolerance_);
	this->get_parameter("control_loop_rate", control_loop_rate_);
	this->get_parameter("min_linear_speed", min_linear_speed_);
	this->get_parameter("max_linear_speed", max_linear_speed_);
	this->get_parameter("max_angular_speed", max_angular_speed_);
	this->get_parameter("max_acceleration", max_acceleration_);
	this->get_parameter("max_deceleration", max_deceleration_);

	linear_pid_.set_parameters(Kp_linear_, Ki_linear_, Kd_linear_);
	angular_pid_.set_parameters(Kp_angular_, Ki_angular_, Kd_angular_);

	RCLCPP_INFO(this->get_logger(), "Navigation Server has been started.");
}

double calculateDistance(const Point& p1, const Point& p2) {
	double dx = p2.x - p1.x;
	double dy = p2.y - p1.y;
	return std::sqrt(dx * dx + dy * dy);
}

// Fonction pour déterminer de quel côté du segment DA se trouve B (produit vectoriel)
double crossProduct(const Point& D, const Point& A, const Point& B) {
	double DAx = A.x - D.x;
	double DAy = A.y - D.y;
	double DBx = B.x - D.x;
	double DBy = B.y - D.y;
	return DAx * DBy - DAy * DBx;
}

Point calculateCorrectedEndpoint(const Point& projection, const Point& A, const Point& D, const Point& B) {
	// Calculer la distance entre B et la projection R
	double distanceBR = calculateDistance(B, projection);

	// Calculer le vecteur orthogonal à DA
	double DAx = A.x - D.x;
	double DAy = A.y - D.y;

	// Normaliser le vecteur orthogonal
	double lengthDA = std::sqrt(DAx * DAx + DAy * DAy);
	double orthogonalDx = -DAy / lengthDA;
	double orthogonalDy = DAx / lengthDA;

	// Déterminer de quel côté se trouve le point B par rapport à DA
	double cross = crossProduct(D, A, B);

	// Toujours déplacer dans la direction opposée au point B pour obtenir la symétrie
	if (cross > 0) {
		orthogonalDx = -orthogonalDx;
		orthogonalDy = -orthogonalDy;
	}

	// Calculer le point corrigé en se déplaçant depuis A dans la direction orthogonale
	Point correctedEndpoint;
	correctedEndpoint.x = A.x + orthogonalDx * distanceBR;
	correctedEndpoint.y = A.y + orthogonalDy * distanceBR;

	return correctedEndpoint;
}


// Fonction pour calculer la projection perpendiculaire de B sur l'itinéraire DA
Point calculatePerpendicularProjection(const Point& A, const Point& D, const Point& B) {
	double DAx = A.x - D.x;
	double DAy = A.y - D.y;
	double DBx = B.x - D.x;
	double DBy = B.y - D.y;

	double dotProduct = DAx * DBx + DAy * DBy;
	double lengthSquared = DAx * DAx + DAy * DAy;

	double t = 0.0;
	if (lengthSquared != 0) {
		t = dotProduct / lengthSquared;
	}

	// Calculer la projection sur la droite DA (pas de limitation à l'intervalle [0, 1])
	Point projection;
	projection.x = D.x + t * DAx;
	projection.y = D.y + t * DAy;

	return projection;
}

// Fonction pour calculer l'écart de trajectoire entre la projection et le point externe
double calculateTrajectoryDeviation(const Point& projection, const Point& B) {
	return calculateDistance(projection, B);
}


void NavigationServer::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	current_odometry_ = *msg;
	odom_received_ = true;
}

rclcpp_action::GoalResponse NavigationServer::handle_goal(
	const rclcpp_action::GoalUUID & uuid,
	std::shared_ptr<const Navigation::Goal> goal)
{
	RCLCPP_INFO(this->get_logger(), "Received Navigation goal request.");

	if (goal->path.poses.empty())
	{
		RCLCPP_WARN(this->get_logger(), "Received empty path.");
		return rclcpp_action::GoalResponse::REJECT;
	}

	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavigationServer::handle_cancel(
	const std::shared_ptr<GoalHandleNavigation> goal_handle)
{
	RCLCPP_INFO(this->get_logger(), "Received request to cancel Navigation goal.");
	return rclcpp_action::CancelResponse::ACCEPT;
}

void NavigationServer::handle_accepted(const std::shared_ptr<GoalHandleNavigation> goal_handle)
{
	std::thread(
		[this, goal_handle]() {
			execute(goal_handle);
		}
	).detach();
}

void NavigationServer::execute(const std::shared_ptr<GoalHandleNavigation> goal_handle)
{
	RCLCPP_INFO(this->get_logger(), "Executing Navigation goal...");

	auto goal = goal_handle->get_goal();
	path_ = goal->path.poses;
	current_waypoint_index_ = 0;
	starting_point_ = {0.0, 0.0, false};
	goal_cancelled_ = false;

	auto feedback = std::make_shared<Navigation::Feedback>();
	auto result = std::make_shared<Navigation::Result>();

	rclcpp::Rate rate(control_loop_rate_);
	while (rclcpp::ok())
	{
		if (goal_handle->is_canceling())
		{
			goal_cancelled_ = true;
			result->success = false;
			goal_handle->canceled(result);
			RCLCPP_INFO(this->get_logger(), "Navigation goal canceled.");
			return;
		}

		if (!odom_received_)
		{
			RCLCPP_WARN(this->get_logger(), "Waiting for odometry...");
			rate.sleep();
			continue;
		}

		controlLoop(goal_handle);

		if (current_waypoint_index_ >= path_.size())
		{
			auto cmd_msg = geometry_msgs::msg::Twist();
			cmd_publisher_->publish(cmd_msg);

			result->success = true;
			goal_handle->succeed(result);
			RCLCPP_INFO(this->get_logger(), "Navigation goal succeeded.");
			return;
		}

		feedback->progress = static_cast<float>(current_waypoint_index_) / path_.size() * 100.0f;
		goal_handle->publish_feedback(feedback);

		rate.sleep();
	}
}

void NavigationServer::controlLoop(const std::shared_ptr<GoalHandleNavigation> goal_handle)
{
	if (current_waypoint_index_ >= path_.size())
		return;

	auto current_waypoint = path_[current_waypoint_index_].pose;

	double x_current = current_odometry_.pose.pose.position.x;
	double y_current = current_odometry_.pose.pose.position.y;

	tf2::Quaternion q_current(
		current_odometry_.pose.pose.orientation.x,
		current_odometry_.pose.pose.orientation.y,
		current_odometry_.pose.pose.orientation.z,
		current_odometry_.pose.pose.orientation.w
	);
	double roll_current, pitch_current, yaw_current;
	tf2::Matrix3x3(q_current).getRPY(roll_current, pitch_current, yaw_current);

	Point A = {current_waypoint.position.x, current_waypoint.position.y};

	if (current_waypoint_index_ != 0)
		starting_point_ = {current_waypoint.position.x, current_waypoint.position.y, true};
	else if (current_waypoint_index_ == 0 && starting_point_.initialized == false)
		starting_point_ = {current_odometry_.pose.pose.position.x, current_odometry_.pose.pose.position.y, true};

	Point D = {starting_point_.x, starting_point_.y};
	Point B = {x_current, y_current};

	Point projection = calculatePerpendicularProjection(A, D, B);
	double deviation = calculateTrajectoryDeviation(projection, B);
	Point correctedEndpoint = calculateCorrectedEndpoint(projection, A, D, B);

	//log des valeurs waypoint index, A D B et correctedEndpoint
	RCLCPP_INFO(this->get_logger(), "Waypoint Index: %zu", current_waypoint_index_);
	RCLCPP_INFO(this->get_logger(), "A: %.2f, %.2f", A.x, A.y);
	RCLCPP_INFO(this->get_logger(), "D: %.2f, %.2f", D.x, D.y);
	RCLCPP_INFO(this->get_logger(), "B: %.2f, %.2f", B.x, B.y);
	RCLCPP_INFO(this->get_logger(), "Corrected Endpoint: %.2f, %.2f", correctedEndpoint.x, correctedEndpoint.y);

	double x_goal = correctedEndpoint.x;
	double y_goal = correctedEndpoint.y;

	double error_x = x_goal - x_current;
	double error_y = y_goal - y_current;
	double distance_to_goal = std::sqrt(error_x * error_x + error_y * error_y);

	if (distance_to_goal < position_tolerance_)
	{
		RCLCPP_INFO(this->get_logger(), "Waypoint %zu reached.", current_waypoint_index_ + 1);
		current_waypoint_index_++;
		initial_distance_to_goal_ = 0.0;
		return;
	}

	if (last_waypoint_index_ != current_waypoint_index_)
	{
		initial_distance_to_goal_ = distance_to_goal;
		last_waypoint_index_ = current_waypoint_index_;
		
		linear_pid_.reset();
		angular_pid_.reset();
	}

	double distance_traveled = initial_distance_to_goal_ - distance_to_goal;
	// distance_traveled = std::max(0.0, distance_traveled);

	double theta_goal = std::atan2(error_y, error_x);

	double error_theta = theta_goal - yaw_current;
	error_theta = std::atan2(std::sin(error_theta), std::cos(error_theta));

	double error_linear = distance_to_goal;

	double dt = 1.0 / control_loop_rate_;

	double linear_speed_pid = linear_pid_.compute(0.0, -error_linear, dt);
	double angular_speed_pid = angular_pid_.compute(0.0, -error_theta, dt);

	double accel_distance = 100.0; // mètres
	double decel_distance = 100.0; // mètres
	double target_speed = max_linear_speed_;

	if (distance_traveled <= accel_distance)
	{
		double scaling_factor = std::abs(distance_traveled) / accel_distance;
		scaling_factor = std::clamp(scaling_factor, 0.0, 1.0);
		target_speed = scaling_factor * max_linear_speed_;
	}

	if (distance_to_goal <= decel_distance)
	{
		double decel_factor = distance_to_goal / decel_distance;
		decel_factor = std::clamp(decel_factor, 0.0, 1.0);
		target_speed = std::min(target_speed, decel_factor * max_linear_speed_);
	}

	target_speed = std::clamp(target_speed, min_linear_speed_, max_linear_speed_);

	double delta_speed = target_speed - current_linear_speed_;

	if (delta_speed > 0)
		delta_speed = std::min(delta_speed, max_acceleration_ * dt);
	else
		delta_speed = std::max(delta_speed, -max_deceleration_ * dt);

	current_linear_speed_ += delta_speed;

	double linear_speed = current_linear_speed_;

	double angular_error_threshold = 0.5; // radians
	if (std::abs(error_theta) > angular_error_threshold)
	{
		double scaling_factor = angular_error_threshold / std::abs(error_theta);
		scaling_factor = std::clamp(scaling_factor, 0.0, 1.0);
		linear_speed *= scaling_factor;
	}

	linear_speed = std::clamp(linear_speed, min_linear_speed_, max_linear_speed_);
	double angular_speed = std::clamp(angular_speed_pid, -max_angular_speed_, max_angular_speed_);

	auto cmd_msg = geometry_msgs::msg::Twist();
	cmd_msg.linear.x = linear_speed;
	cmd_msg.angular.z = angular_speed;

	cmd_publisher_->publish(cmd_msg);

	RCLCPP_INFO(this->get_logger(), "distance_to_goal: %.2f", distance_to_goal);
	RCLCPP_INFO(this->get_logger(), "Control Outputs: linear_speed=%.2f m/s, angular_speed=%.2f rad/s",
				linear_speed, angular_speed);
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<NavigationServer>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
