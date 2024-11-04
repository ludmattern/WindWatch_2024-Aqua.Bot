// src/inspection_server.cpp

#include "mission_manager/inspection_server.hpp"
#include "mission_manager/controlUtils.hpp"
#include <algorithm>
#include <limits>
#include <iostream>
#include <cmath>

using namespace std::chrono_literals;

InspectionServer::InspectionServer(): Node("inspection_server"), odom_received_(false), dataReceived_(false)
{
	action_server_ = rclcpp_action::create_server<Inspection>(
		this,
		"inspection",
		std::bind(&InspectionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&InspectionServer::handle_cancel, this, std::placeholders::_1),
		std::bind(&InspectionServer::handle_accepted, this, std::placeholders::_1)
	);

	CameraControlServClient_ = this->create_client<sensors::srv::CameraControlServ>("mission/camera_control");

	cmdPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
		"/propulsion/command", 10);

	odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"/mission/odometry", 10,
		std::bind(&InspectionServer::odomCallback, this, std::placeholders::_1)
	);

	headingController_ = PIDController(0.7, 0.01, 0.0, 0.78, -0.78, 0.5, 0.017);
	speedController_ = PIDController(0.1, 0.005, 0.02, 6.17, 0.0, 0.5, 0.1);

	RCLCPP_INFO(this->get_logger(), "Inspection Server has been started.");
}

void InspectionServer::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	current_odometry_ = *msg;
	odom_received_ = true;
}

rclcpp_action::GoalResponse InspectionServer::handle_goal(const rclcpp_action::GoalUUID & uuid,	std::shared_ptr<const Inspection::Goal> goal)
{
	RCLCPP_INFO(this->get_logger(), "Received Inspection goal request.");

	if (goal->path.poses.empty())
	{
		RCLCPP_WARN(this->get_logger(), "Received empty path.");
		return rclcpp_action::GoalResponse::REJECT;
	}

	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse InspectionServer::handle_cancel(const std::shared_ptr<GoalHandleInspection> goal_handle)
{
	RCLCPP_INFO(this->get_logger(), "Received request to cancel Inspection goal.");
	return rclcpp_action::CancelResponse::ACCEPT;
}

void InspectionServer::handle_accepted(const std::shared_ptr<GoalHandleInspection> goal_handle)
{
	std::thread(
		[this, goal_handle]() {
			execute(goal_handle);
		}
	).detach();
}

void InspectionServer::execute(const std::shared_ptr<GoalHandleInspection> goal_handle)
{
	RCLCPP_INFO(this->get_logger(), "Executing Inspection goal...");

	auto goal = goal_handle->get_goal();
	path_ = goal->path.poses;
	goalCancelled_ = false;

	auto feedback = std::make_shared<Inspection::Feedback>();
	auto result = std::make_shared<Inspection::Result>();

	rclcpp::Rate rate(10);
	while (rclcpp::ok())
	{
		if (goal_handle->is_canceling())
		{
			goalCancelled_ = true;
			result->success = false;
			goal_handle->canceled(result);
			RCLCPP_INFO(this->get_logger(), "Inspection goal canceled.");
			return;
		}

		if (!odom_received_)
		{
			RCLCPP_WARN(this->get_logger(), "Waiting for odometry...");
			rate.sleep();
			continue;
		}

		controlLoop(goal_handle);

		if (dataReceived_)
		{
			auto cmdMsg = geometry_msgs::msg::Twist();
			cmdPublisher_->publish(cmdMsg);
			targetIndex_ = 0;
			result->success = true;
			goal_handle->succeed(result);
			RCLCPP_INFO(this->get_logger(), "Inspection goal succeeded.");
			dataReceived_ = false;
			return;
		}

		feedback->progress = static_cast<float>(targetIndex_) / path_.size() * 100.0f;
		goal_handle->publish_feedback(feedback);

		rate.sleep();
	}
}

bool InspectionServer::isGoalReached(void)
{
	return false;
}

void InspectionServer::controlLoop(const std::shared_ptr<GoalHandleInspection> goal_handle)
{
	if (goalCancelled_ || isGoalReached())
	{
		RCLCPP_INFO(this->get_logger(), "Goal cancelled or reached.");
		return;
	}

	const geometry_msgs::msg::PoseStamped &target = path_.back();

	std::lock_guard<std::mutex> lock(odom_mutex_);
	controlUtils::OdometryData odometryData = controlUtils::getOdometryData(target, current_odometry_);

	const double orbitRadius = 8.0;
	const double orbitSpeed = 1.0;

	if (!entryPointInitialized_)
		initializeEntryPoint(odometryData, target, orbitRadius);

	switch (state_)
	{
		case InspectionState::APPROACH:
			processApproachState(odometryData, target, orbitRadius, orbitSpeed);
			break;

		case InspectionState::ORBIT:
			processOrbitState(odometryData, target, orbitRadius, orbitSpeed);
			break;

		default:
			RCLCPP_WARN(this->get_logger(), "Unknown state.");
			break;
	}
}

void InspectionServer::initializeEntryPoint(const controlUtils::OdometryData &odometryData, const geometry_msgs::msg::PoseStamped &target, double orbitRadius)
{
	geometry_msgs::msg::Pose entryPoint = controlUtils::ClosestPointOnOrbit(odometryData, target, orbitRadius);
	entryPoint_.pose = entryPoint;
	entryPointInitialized_ = true;
}

void InspectionServer::serviceResponseCallback(rclcpp::Client<sensors::srv::CameraControlServ>::SharedFuture future)
{
	dataReceived_ = true;
}

void InspectionServer::processApproachState(const controlUtils::OdometryData &odometryData, const geometry_msgs::msg::PoseStamped &target, double orbitRadius, double orbitSpeed)
{
	double angleToEntryPoint = controlUtils::getTgtAngleError(odometryData, entryPoint_);
	double distanceToEntryPoint = controlUtils::calculateDistance(odometryData, entryPoint_);

	double headingOutput = headingController_.calculate(angleToEntryPoint, orbitRadius);
	double speedOutput = std::min(orbitSpeed, distanceToEntryPoint * 0.5);

	if (distanceToEntryPoint <= orbitRadius + 6.0)
	{
		state_ = InspectionState::ORBIT;

		while (!this->CameraControlServClient_->wait_for_service(std::chrono::seconds(1)))
			RCLCPP_ERROR(this->get_logger(), "Service 'mission/mission_goal' not available");

		RCLCPP_INFO(this->get_logger(), "Service is available. Sending request...");

		auto request = std::make_shared<sensors::srv::CameraControlServ::Request>();

		//posestamp;
		request->target.x = target.pose.position.x;
		request->target.y = target.pose.position.y;

		auto future = this->CameraControlServClient_->async_send_request(
			request,
			std::bind(&InspectionServer::serviceResponseCallback, this, std::placeholders::_1)
		);

		headingController_.reset();
		speedController_.reset();

		headingController_.setMultipliers(2.0, 0.0, 0.0);
	}

	controlUtils::sendThrustersCommands(speedOutput, headingOutput, cmdPublisher_);
}

void InspectionServer::processOrbitState(const controlUtils::OdometryData &odometryData, const geometry_msgs::msg::PoseStamped &target, double orbitRadius, double orbitSpeed)
{
	double angleToTarget = controlUtils::getTgtAngleError(odometryData, target);
	double distanceError = odometryData.distanceToTarget - orbitRadius;
	double orbitAngleError = controlUtils::OrbitHeadingAdjustment(odometryData, angleToTarget, distanceError, orbitRadius);
	double headingOutput = headingController_.calculate(orbitAngleError, orbitRadius);
	double speedOutput = std::min(orbitSpeed, distanceError * 0.5);

	controlUtils::sendThrustersCommands(speedOutput, headingOutput, cmdPublisher_);
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<InspectionServer>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

