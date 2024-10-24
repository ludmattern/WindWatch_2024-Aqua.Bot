// src/action_manager.hpp

#ifndef ACTION_MANAGER_DEMO__ACTION_MANAGER_HPP_
#define ACTION_MANAGER_DEMO__ACTION_MANAGER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_manager_demo/action/navigation.hpp"
#include "action_manager_demo/action/inspection.hpp"
#include "action_manager_demo/action/stabilization.hpp"
#include "action_manager_demo/action/rotation.hpp"

class ActionManager : public rclcpp::Node
{
public:
	using Navigation = action_manager_demo::action::Navigation;
	using Inspection = action_manager_demo::action::Inspection;
	using Stabilization = action_manager_demo::action::Stabilization;
	using Rotation = action_manager_demo::action::Rotation;
	using GoalHandleNavigation = rclcpp_action::ClientGoalHandle<Navigation>;
	using GoalHandleInspection = rclcpp_action::ClientGoalHandle<Inspection>;
	using GoalHandleStabilization = rclcpp_action::ClientGoalHandle<Stabilization>;
	using GoalHandleRotation = rclcpp_action::ClientGoalHandle<Rotation>;

	ActionManager();

private:
	rclcpp_action::Client<Navigation>::SharedPtr navigation_client_;
	rclcpp_action::Client<Inspection>::SharedPtr inspection_client_;
	rclcpp_action::Client<Stabilization>::SharedPtr stabilization_client_;
	rclcpp_action::Client<Rotation>::SharedPtr rotation_client_;

	void send_navigation_goal(int target_number);
	void send_inspection_goal(int target_number);
	void send_stabilization_goal(int target_number);
	void send_rotation_goal(int target_number);

	void handle_navigation_result(const GoalHandleNavigation::WrappedResult & result);
	void handle_inspection_result(const GoalHandleInspection::WrappedResult & result);
	void handle_stabilization_result(const GoalHandleStabilization::WrappedResult & result);
	void handle_rotation_result(const GoalHandleRotation::WrappedResult & result);

	int sequence1_count_;          // Nombre de répétitions de la Séquence 1
	const int sequence1_max_;      // Maximum de répétitions pour la Séquence 1

	enum class Sequence
	{
		SEQUENCE1,
		SEQUENCE2,
		DONE
	} current_sequence_;            // Séquence actuelle

// Callbacks for feedback
void handle_navigation_feedback(
	GoalHandleNavigation::SharedPtr,
	const std::shared_ptr<const Navigation::Feedback> feedback);

void handle_inspection_feedback(
	GoalHandleInspection::SharedPtr,
	const std::shared_ptr<const Inspection::Feedback> feedback);

void handle_stabilization_feedback(
	GoalHandleStabilization::SharedPtr,
	const std::shared_ptr<const Stabilization::Feedback> feedback);

void handle_rotation_feedback(
	GoalHandleRotation::SharedPtr,
	const std::shared_ptr<const Rotation::Feedback> feedback);
};

#endif  // ACTION_MANAGER_DEMO__ACTION_MANAGER_HPP_
