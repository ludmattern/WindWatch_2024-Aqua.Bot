#ifndef MISSION_MANAGER__INSPECTION_SERVER_HPP_
#define MISSION_MANAGER__INSPECTION_SERVER_HPP_

#include <memory>
#include <vector>
#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "mission_manager/controlUtils.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "mission_manager/action/inspection.hpp"
#include "mission_manager/PIDController.hpp"

#include "sensors/action/camera_control.hpp"

enum class InspectionState
{
	APPROACH,
	ORBIT
};

class InspectionServer : public rclcpp::Node
{
public:
	using Inspection = mission_manager::action::Inspection;
	using GoalHandleInspection = rclcpp_action::ServerGoalHandle<Inspection>;

	using CameraControl = sensors::action::CameraControl;
	using GoalHandleCameraControl = rclcpp_action::ClientGoalHandle<CameraControl>;

	InspectionServer();

private:
	rclcpp_action::Server<Inspection>::SharedPtr action_server_;

	rclcpp_action::Client<CameraControl>::SharedPtr camera_control_client_;

	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdPublisher_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

	nav_msgs::msg::Odometry current_odometry_;
	std::mutex odom_mutex_;
	bool odom_received_;

	std::vector<geometry_msgs::msg::PoseStamped> path_;
	size_t targetIndex_;
	bool dataReceived_;

	bool goalCancelled_;

	rclcpp_action::GoalResponse handle_goal(
		const rclcpp_action::GoalUUID &uuid,
		std::shared_ptr<const Inspection::Goal> goal);

	rclcpp_action::CancelResponse handle_cancel(
		const std::shared_ptr<GoalHandleInspection> goal_handle);

	void handle_accepted(
		const std::shared_ptr<GoalHandleInspection> goal_handle);

	void execute(
		const std::shared_ptr<GoalHandleInspection> goal_handle);

	void controlLoop(
		const std::shared_ptr<GoalHandleInspection> goal_handle);

	void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
	bool isGoalReached(void);

	PIDController headingController_;
	PIDController speedController_;

	geometry_msgs::msg::PoseStamped entryPoint_;
	bool entryPointInitialized_ = false;
	InspectionState state_ = InspectionState::APPROACH;

	void processOrbitState(
		const controlUtils::OdometryData &odometryData,
		const geometry_msgs::msg::PoseStamped &target,
		double orbitRadius,
		double orbitSpeed);

	void processApproachState(
		const controlUtils::OdometryData &odometryData,
		const geometry_msgs::msg::PoseStamped &target,
		double orbitRadius,
		double orbitSpeed);

	void initializeEntryPoint(
		const controlUtils::OdometryData &odometryData,
		const geometry_msgs::msg::PoseStamped &target,
		double orbitRadius);

	void camera_control_goal_response_callback(
		std::shared_ptr<GoalHandleCameraControl> goal_handle);

	void camera_control_feedback_callback(
		GoalHandleCameraControl::SharedPtr,
		const std::shared_ptr<const CameraControl::Feedback> feedback);

	void camera_control_result_callback(
		const GoalHandleCameraControl::WrappedResult &result);
};

#endif  // MISSION_MANAGER__INSPECTION_SERVER_HPP_
