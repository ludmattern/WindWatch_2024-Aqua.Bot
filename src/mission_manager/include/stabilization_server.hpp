// src/stabilization_server.hpp

#ifndef MISSION_MANAGER__STABILIZATION_SERVER_HPP_
#define MISSION_MANAGER__STABILIZATION_SERVER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "mission_manager/action/stabilization.hpp"

class StabilizationServer : public rclcpp::Node
{
	public:
	using Stabilization = mission_manager::action::Stabilization;
	using GoalHandleStabilization = rclcpp_action::ServerGoalHandle<Stabilization>;

	StabilizationServer();

	private:
	rclcpp_action::Server<Stabilization>::SharedPtr action_server_;

	rclcpp_action::GoalResponse handle_goal(
		const rclcpp_action::GoalUUID & uuid,
		std::shared_ptr<const Stabilization::Goal> goal);

	rclcpp_action::CancelResponse handle_cancel(
		const std::shared_ptr<GoalHandleStabilization> goal_handle);

	void handle_accepted(const std::shared_ptr<GoalHandleStabilization> goal_handle);
	void execute(const std::shared_ptr<GoalHandleStabilization> goal_handle);
};

#endif  // MISSION_MANAGER__STABILIZATION_SERVER_HPP_
