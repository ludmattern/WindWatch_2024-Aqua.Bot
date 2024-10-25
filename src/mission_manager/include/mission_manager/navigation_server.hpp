// src/navigation_server.hpp

#ifndef MISSION_MANAGER__NAVIGATION_SERVER_HPP_
#define MISSION_MANAGER__NAVIGATION_SERVER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "mission_manager/action/navigation.hpp"

class NavigationServer : public rclcpp::Node
{
	public:
	using Navigation = mission_manager::action::Navigation;
	using GoalHandleNavigation = rclcpp_action::ServerGoalHandle<Navigation>;

	NavigationServer();

	private:
	rclcpp_action::Server<Navigation>::SharedPtr action_server_;

	rclcpp_action::GoalResponse handle_goal(
		const rclcpp_action::GoalUUID & uuid,
		std::shared_ptr<const Navigation::Goal> goal);

	rclcpp_action::CancelResponse handle_cancel(
		const std::shared_ptr<GoalHandleNavigation> goal_handle);

	void handle_accepted(const std::shared_ptr<GoalHandleNavigation> goal_handle);
	void execute(const std::shared_ptr<GoalHandleNavigation> goal_handle);
};

#endif  // MISSION_MANAGER__NAVIGATION_SERVER_HPP_
