// src/inspection_server.hpp

#ifndef MISSION_MANAGER__INSPECTION_SERVER_HPP_
#define MISSION_MANAGER__INSPECTION_SERVER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "mission_manager/action/inspection.hpp"

class InspectionServer : public rclcpp::Node
{
	public:
	using Inspection = mission_manager::action::Inspection;
	using GoalHandleInspection = rclcpp_action::ServerGoalHandle<Inspection>;

	InspectionServer();

	private:
	rclcpp_action::Server<Inspection>::SharedPtr action_server_;

	rclcpp_action::GoalResponse handle_goal(
		const rclcpp_action::GoalUUID & uuid,
		std::shared_ptr<const Inspection::Goal> goal);

	rclcpp_action::CancelResponse handle_cancel(
		const std::shared_ptr<GoalHandleInspection> goal_handle);

	void handle_accepted(const std::shared_ptr<GoalHandleInspection> goal_handle);
	void execute(const std::shared_ptr<GoalHandleInspection> goal_handle);
};

#endif  // MISSION_MANAGER__INSPECTION_SERVER_HPP_
