// src/stabilization_server.cpp

#include "mission_manager/stabilization_server.hpp"
#include "mission_manager/controlUtils.hpp"
#include <algorithm>
#include <limits>
#include <iostream>
#include <cmath>

using namespace std::chrono_literals;

StabilizationServer::StabilizationServer(): Node("stabilization_server"), odom_received_(false)
{
	action_server_ = rclcpp_action::create_server<Stabilization>(this, "stabilization",
		std::bind(&StabilizationServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&StabilizationServer::handle_cancel, this, std::placeholders::_1),
		std::bind(&StabilizationServer::handle_accepted, this, std::placeholders::_1));

	cmdPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/propulsion/command", 10);

	odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/mission/odometry", 10,
		std::bind(&StabilizationServer::odomCallback, this, std::placeholders::_1));

	headingController_ = PIDController(0.7, 0.01, 0.0, 0.78, -0.78, 0.5, 0.017);
	speedController_ = PIDController(0.1, 0.005, 0.02, 6.17, 0.0, 0.5, 0.1);

	RCLCPP_INFO(this->get_logger(), "Stabilization Server has been started.");
}

void StabilizationServer::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	current_odometry_ = *msg;
	odom_received_ = true;
}

rclcpp_action::GoalResponse StabilizationServer::handle_goal(const rclcpp_action::GoalUUID & uuid,	std::shared_ptr<const Stabilization::Goal> goal)
{
	RCLCPP_INFO(this->get_logger(), "Received Stabilization goal request.");

	if (goal->path.poses.empty())
	{
		RCLCPP_WARN(this->get_logger(), "Received empty path.");
		return rclcpp_action::GoalResponse::REJECT;
	}

	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse StabilizationServer::handle_cancel(const std::shared_ptr<GoalHandleStabilization> goal_handle)
{
	RCLCPP_INFO(this->get_logger(), "Received request to cancel Stabilization");
	return rclcpp_action::CancelResponse::ACCEPT;
}

void StabilizationServer::handle_accepted(const std::shared_ptr<GoalHandleStabilization> goal_handle)
{
	std::thread([this, goal_handle](){ execute(goal_handle); }).detach();
}

void StabilizationServer::execute(const std::shared_ptr<GoalHandleStabilization> goal_handle)
{
	RCLCPP_INFO(this->get_logger(), "Executing Stabilization goal.");


	auto goal = goal_handle->get_goal();
	path_ = goal->path.poses;
	goalCancelled_ = false;

	auto feedback = std::make_shared<Stabilization::Feedback>();
	auto result = std::make_shared<Stabilization::Result>();

	rclcpp::Rate rate(10);
	while (rclcpp::ok())
	{
		if (goal_handle->is_canceling())
		{
			goalCancelled_ = true;
			result->success = false;
			goal_handle->canceled(result);
			RCLCPP_INFO(this->get_logger(), "Stabilization canceled");
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
			RCLCPP_INFO(this->get_logger(), "Stabilization succeeded");
			return;
		}

		feedback->progress = static_cast<float>(targetIndex_) / path_.size() * 100.0f;
		goal_handle->publish_feedback(feedback);

		rate.sleep();
	}
}

void StabilizationServer::controlLoop(const std::shared_ptr<GoalHandleStabilization> goal_handle)
{
	if (goalCancelled_) return;

	const geometry_msgs::msg::PoseStamped & target = path_[targetIndex_];

	std::lock_guard<std::mutex> lock(odom_mutex_);
	controlUtils::OdometryData odometryData = controlUtils::getOdometryData(target, current_odometry_);

	double targetAngleError = controlUtils::getTgtAngleError(odometryData, target);

	double headingOutput = headingController_.calculate(targetAngleError, odometryData.distanceToTarget);
	double speedOutput = speedController_.calculate(odometryData.distanceToTarget);

	controlUtils::sendThrustersCommands(speedOutput, headingOutput, cmdPublisher_);
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<StabilizationServer>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
