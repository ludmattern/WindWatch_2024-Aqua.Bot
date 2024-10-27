// src/navigation_server.cpp

#include "mission_manager/navigation_server.hpp"
#include <algorithm>
#include <limits>
#include <cmath>

using namespace std::chrono_literals;

NavigationServer::NavigationServer()
: Node("navigation_server"),
odom_received_(false),
initial_distance_to_goal_(0.0),
estimated_disturbance_angular_(0.0),
last_waypoint_index_(std::numeric_limits<size_t>::max()),
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
	this->declare_parameter<double>("position_tolerance", 20.0);
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
    {
        return;
    }

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

    double x_goal = current_waypoint.position.x;
    double y_goal = current_waypoint.position.y;

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
    distance_traveled = std::max(0.0, distance_traveled);

    double theta_goal = std::atan2(error_y, error_x);

    double error_theta = theta_goal - yaw_current;
    error_theta = std::atan2(std::sin(error_theta), std::cos(error_theta));

    double error_linear = distance_to_goal;

    double dt = 1.0 / control_loop_rate_;

    double linear_speed = linear_pid_.compute(0.0, -error_linear, dt);
    double angular_speed = angular_pid_.compute(0.0, -error_theta, dt);

    double accel_distance = 100.0; // mètres
    double decel_distance = 100.0; // mètres
    double target_speed = max_linear_speed_;

    if (distance_to_goal <= decel_distance)
    {
        target_speed = (distance_to_goal / decel_distance) * max_linear_speed_;
    }
    else if (distance_to_goal <= (accel_distance + decel_distance))
    {
        target_speed = ((distance_to_goal - decel_distance) / accel_distance) * max_linear_speed_;
    }

    // Limiter la vitesse cible pour ne pas dépasser la vitesse maximale
    target_speed = std::clamp(target_speed, 0.0, max_linear_speed_);

    // Calcul du changement de vitesse autorisé
    double max_delta_speed = 0.0;
    if (target_speed > current_linear_speed_)
    {
        // Accélération
        max_delta_speed = max_acceleration_ * dt;
    }
    else
    {
        // Décélération
        max_delta_speed = max_deceleration_ * dt;
    }

    double delta_speed = target_speed - current_linear_speed_;
    delta_speed = std::clamp(delta_speed, -max_deceleration_ * dt, max_acceleration_ * dt);

    current_linear_speed_ += delta_speed;

    // Appliquer la vitesse actuelle contrôlée
    linear_speed = current_linear_speed_;

    // Ajuster la vitesse linéaire en fonction de l'erreur angulaire
    double angular_error_threshold = 0.5; // radians
    if (std::abs(error_theta) > angular_error_threshold)
    {
        double scaling_factor = angular_error_threshold / std::abs(error_theta);
        linear_speed *= scaling_factor;
    }

    // Limitation des vitesses
    linear_speed = std::clamp(linear_speed, min_linear_speed_, max_linear_speed_);
    angular_speed = std::clamp(angular_speed, -max_angular_speed_, max_angular_speed_);

    // Création du message de commande
    auto cmd_msg = geometry_msgs::msg::Twist();
    cmd_msg.linear.x = linear_speed;
    cmd_msg.angular.z = angular_speed;

    // Publication de la commande
    cmd_publisher_->publish(cmd_msg);

    // Logs pour le débogage
    RCLCPP_INFO(this->get_logger(), "distance_to_goal: %.2f", distance_to_goal);
    RCLCPP_INFO(this->get_logger(), "initial_distance_to_goal_: %.2f", initial_distance_to_goal_);
    RCLCPP_INFO(this->get_logger(), "distance_traveled: %.2f", distance_traveled);
    RCLCPP_INFO(this->get_logger(), "linear_speed before clamp: %.2f", linear_speed);
    RCLCPP_INFO(this->get_logger(), "Angular Speed Commanded: %.2f rad/s", angular_speed);
    RCLCPP_INFO(this->get_logger(), "Current Position: x=%.2f, y=%.2f, yaw=%.2f rad", x_current, y_current, yaw_current);
    RCLCPP_INFO(this->get_logger(), "Goal Position: x=%.2f, y=%.2f", x_goal, y_goal);
    RCLCPP_INFO(this->get_logger(), "Errors: error_x=%.2f, error_y=%.2f, distance=%.2f, error_theta=%.2f rad",
                error_x, error_y, distance_to_goal, error_theta);
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
