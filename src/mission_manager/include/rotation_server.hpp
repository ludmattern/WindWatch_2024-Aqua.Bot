// src/rotation_server.hpp

#ifndef MISSION_MANAGER__ROTATION_SERVER_HPP_
#define MISSION_MANAGER__ROTATION_SERVER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "mission_manager/action/rotation.hpp"

class RotationServer : public rclcpp::Node
{
	public:
	using Rotation = mission_manager::action::Rotation;
	using GoalHandleRotation = rclcpp_action::ServerGoalHandle<Rotation>;

	RotationServer();

	private:
	rclcpp_action::Server<Rotation>::SharedPtr action_server_;

	rclcpp_action::GoalResponse handle_goal(
		const rclcpp_action::GoalUUID & uuid,
		std::shared_ptr<const Rotation::Goal> goal);

	rclcpp_action::CancelResponse handle_cancel(
		const std::shared_ptr<GoalHandleRotation> goal_handle);

	void handle_accepted(const std::shared_ptr<GoalHandleRotation> goal_handle);
	void execute(const std::shared_ptr<GoalHandleRotation> goal_handle);
};

#endif  // MISSION_MANAGER__ROTATION_SERVER_HPP_
