// src/navigation_server.cpp

#include "mission_manager/navigation_server.hpp"
#include <algorithm>
#include <limits>
#include <iostream>
#include <cmath>

using namespace std::chrono_literals;

NavigationServer::NavigationServer()
: Node("navigation_server"),
odom_received_(false)
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

	headingController_ = PIDController(0.7, 0.01, 0.0, 0.78, -0.78, 0.5, 0.017);
	speedController_ = PIDController(0.1, 0.005, 0.02, 6.17, 0.0, 0.5, 0.1);

	RCLCPP_INFO(this->get_logger(), "Navigation Server has been started.");
}

void NavigationServer::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	current_odometry_ = *msg;
	odom_received_ = true;
}

rclcpp_action::GoalResponse NavigationServer::handle_goal(const rclcpp_action::GoalUUID & uuid,	std::shared_ptr<const Navigation::Goal> goal)
{
	RCLCPP_INFO(this->get_logger(), "Received Navigation goal request.");

	if (goal->path.poses.empty())
	{
		RCLCPP_WARN(this->get_logger(), "Received empty path.");
		return rclcpp_action::GoalResponse::REJECT;
	}

	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavigationServer::handle_cancel(const std::shared_ptr<GoalHandleNavigation> goal_handle)
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
	goal_cancelled_ = false;

	auto feedback = std::make_shared<Navigation::Feedback>();
	auto result = std::make_shared<Navigation::Result>();

	rclcpp::Rate rate(10);
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

		if (target_index_ >= path_.size())
		{
			auto cmd_msg = geometry_msgs::msg::Twist();
			cmd_publisher_->publish(cmd_msg);
			target_index_ = 0;
			result->success = true;
			goal_handle->succeed(result);
			RCLCPP_INFO(this->get_logger(), "Navigation goal succeeded.");
			return;
		}

		feedback->progress = static_cast<float>(target_index_) / path_.size() * 100.0f;
		goal_handle->publish_feedback(feedback);

		rate.sleep();
	}
}

NavigationServer::OdometryData NavigationServer::getOdometryData(const geometry_msgs::msg::PoseStamped & target)
{
	std::lock_guard<std::mutex> lock(odom_mutex_);
	nav_msgs::msg::Odometry odom = current_odometry_;

	OdometryData data;
	data.pos_x = odom.pose.pose.position.x;
	data.pos_y = odom.pose.pose.position.y;

	tf2::Quaternion q(
    odom.pose.pose.orientation.x,
    odom.pose.pose.orientation.y,
    odom.pose.pose.orientation.z,
    odom.pose.pose.orientation.w);

	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;

	m.getRPY(roll, pitch, yaw);
	data.yaw = yaw;

	data.linear_velocity = std::sqrt(
		std::pow(odom.twist.twist.linear.x, 2) +
		std::pow(odom.twist.twist.linear.y, 2));

	data.distance_to_target = std::sqrt(
		std::pow(target.pose.position.x - data.pos_x, 2) +
		std::pow(target.pose.position.y - data.pos_y, 2));

	return data;
}

double NavigationServer::getTgtAngleError(const OdometryData & odometryData, const geometry_msgs::msg::PoseStamped & target)
{
	double targetAngleError = std::atan2(
		target.pose.position.y - odometryData.pos_y,
		target.pose.position.x - odometryData.pos_x);

	double angleError = targetAngleError - odometryData.yaw;
    while (angleError > M_PI) angleError -= 2 * M_PI;
    while (angleError < -M_PI) angleError += 2 * M_PI;

	return angleError;
}

void NavigationServer::controlLoop(const std::shared_ptr<GoalHandleNavigation> goal_handle)
{
	if (target_index_ >= path_.size())
		return;

	const geometry_msgs::msg::PoseStamped & target = path_[target_index_];

	OdometryData odometryData = getOdometryData(target);

	if (odometryData.distance_to_target < 6.0)
	{
		target_index_++;
		headingController_.reset();
		speedController_.reset();
		speedController_.setMaxOutput(3.0); 
		return;
	}

	double targetAngleError = getTgtAngleError(odometryData, target);


	if (odometryData.distance_to_target < 80.0)
	{
		speedController_.setMultipliers(0.4, 0.02, 0.03);
		headingController_.setMultipliers(1.5, 0.025, 0.04);
		// speedController_.setMaxOutput(2); // fonctionne
	}
	else
	{
		speedController_.setMaxOutput(6.17);
		headingController_.setMultipliers(1.0, 0.02, 0.05);
		speedController_.setMultipliers(0.1, 0.005, 0.02);
	}
	
	double headingOutput = headingController_.calculate(targetAngleError, odometryData.distance_to_target);
	double speedOutput = speedController_.calculate(odometryData.distance_to_target);

	geometry_msgs::msg::Twist cmd_msg;
	cmd_msg.linear.x = speedOutput;
	cmd_msg.angular.z = headingOutput;
	cmd_publisher_->publish(cmd_msg);

	RCLCPP_INFO(this->get_logger(), "Control Outputs: Cap: %f, Speed: %f", headingOutput, speedOutput);
	RCLCPP_INFO(this->get_logger(), "Distance from target: %f", odometryData.distance_to_target);

}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<NavigationServer>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
