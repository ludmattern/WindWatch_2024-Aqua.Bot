// src/navigation_server.cpp

#include "mission_manager/navigation_server.hpp"
#include "mission_manager/controlUtils.hpp"
#include <algorithm>
#include <limits>
#include <iostream>
#include <cmath>

using namespace std::chrono_literals;

NavigationServer::NavigationServer(): Node("navigation_server"), odom_received_(false)
{
	action_server_ = rclcpp_action::create_server<Navigation>(this, "navigation",
		std::bind(&NavigationServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&NavigationServer::handle_cancel, this, std::placeholders::_1),
		std::bind(&NavigationServer::handle_accepted, this, std::placeholders::_1));

	cmdPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/propulsion/command", 10);

	odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/mission/odometry", 10,
		std::bind(&NavigationServer::odomCallback, this, std::placeholders::_1));

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
	RCLCPP_INFO(this->get_logger(), "Received request to cancel Navigation");
	return rclcpp_action::CancelResponse::ACCEPT;
}

void NavigationServer::handle_accepted(const std::shared_ptr<GoalHandleNavigation> goal_handle)
{
	std::thread([this, goal_handle](){ execute(goal_handle); }).detach();
}

void NavigationServer::execute(const std::shared_ptr<GoalHandleNavigation> goal_handle)
{
	RCLCPP_INFO(this->get_logger(), "Executing Navigation goal.");


	auto goal = goal_handle->get_goal();
	path_ = goal->path.poses;
	goalCancelled_ = false;

	auto feedback = std::make_shared<Navigation::Feedback>();
	auto result = std::make_shared<Navigation::Result>();

	rclcpp::Rate rate(10);
	while (rclcpp::ok())
	{
		if (goal_handle->is_canceling())
		{
			goalCancelled_ = true;
			result->success = false;
			goal_handle->canceled(result);
			RCLCPP_INFO(this->get_logger(), "Navigation canceled");
			return;
		}

		if (!odom_received_)
		{
			RCLCPP_WARN(this->get_logger(), "Waiting for odometry...");
			rate.sleep();
			continue;
		}

		controlLoop(goal_handle);

		if (targetIndex_ >= path_.size())
		{
			auto cmdMsg = geometry_msgs::msg::Twist();
			cmdPublisher_->publish(cmdMsg);
			targetIndex_ = 0;
			result->success = true;
			goal_handle->succeed(result);
			RCLCPP_INFO(this->get_logger(), "Navigation succeeded");
			return;
		}

		feedback->progress = static_cast<float>(targetIndex_) / path_.size() * 100.0f;
		goal_handle->publish_feedback(feedback);

		rate.sleep();
	}
}

bool NavigationServer::isPointTGT(const geometry_msgs::msg::PoseStamped & target)
{
	return targetIndex_ == path_.size() - 1;
}

bool NavigationServer::isGoalReached(void)
{
	return targetIndex_ >= path_.size();
}

void NavigationServer::adjustPIDSettings(double distanceToTarget, double requestedPrecision)
{
	if (distanceToTarget < 150.0 && requestedPrecision == 100.0)
	{
		speedController_.setMultipliers(0.4, 0.02, 0.03);
		headingController_.setMultipliers(1.5, 0.025, 0.04);
	}
	else
	{
		speedController_.setMaxOutput(6.17);
		headingController_.setMultipliers(1.5, 0.02, 0.05);
		speedController_.setMultipliers(0.1, 0.005, 0.02);
	}
}

void NavigationServer::controlLoop(const std::shared_ptr<GoalHandleNavigation> goal_handle)
{
	if (goalCancelled_ || isGoalReached()) return;

	const geometry_msgs::msg::PoseStamped & target = path_[targetIndex_];

	std::lock_guard<std::mutex> lock(odom_mutex_);
	controlUtils::OdometryData odometryData = controlUtils::getOdometryData(target, current_odometry_);

	double requestedPrecision_ = isPointTGT(target) ? 100.0 : 60.0;

	if (odometryData.distanceToTarget <= requestedPrecision_)
	{
		targetIndex_++;
		headingController_.reset();
		speedController_.reset();
		speedController_.setMaxOutput(3.0); 
		return;
	}

	double targetAngleError = controlUtils::getTgtAngleError(odometryData, target);

	adjustPIDSettings(odometryData.distanceToTarget, requestedPrecision_);
	
	double headingOutput = headingController_.calculate(targetAngleError, odometryData.distanceToTarget);
	double speedOutput = speedController_.calculate(odometryData.distanceToTarget);

	controlUtils::sendThrustersCommands(speedOutput, headingOutput, cmdPublisher_);
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<NavigationServer>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
