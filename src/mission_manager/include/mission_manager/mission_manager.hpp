#ifndef MISSION_MANAGER_DEMO__MISSION_MANAGER_HPP_
#define MISSION_MANAGER_DEMO__MISSION_MANAGER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "mission_manager/action/inspection.hpp"
#include "mission_manager/action/navigation.hpp"
#include "mission_manager/action/rotation.hpp"
#include "mission_manager/action/stabilization.hpp"


class MissionManager : public rclcpp::Node
{
	public:
	// Types d'action
	using Navigation = mission_manager::action::Navigation;
	using Inspection = mission_manager::action::Inspection;
	using Stabilization = mission_manager::action::Stabilization;
	using Rotation = mission_manager::action::Rotation;

	// Définition des types
	template<typename ActionT>
	using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;

	template<typename ActionT>
	using ActionClient = rclcpp_action::Client<ActionT>;

	MissionManager();

	private:
	// Énumération des séquences
	enum class Sequence
	{
		SEQUENCE1,
		SEQUENCE2,
		DONE
	};

	// Membres privés
	ActionClient<Navigation>::SharedPtr navigation_client_;
	ActionClient<Inspection>::SharedPtr inspection_client_;
	ActionClient<Stabilization>::SharedPtr stabilization_client_;
	ActionClient<Rotation>::SharedPtr rotation_client_;

	int sequence1_iteration_count_;        ///< Nombre d'itérations de la Séquence 1
	const int sequence1_max_iterations_;   ///< Nombre maximal d'itérations pour la Séquence 1
	Sequence current_sequence_;            ///< Séquence actuelle
	nav_msgs::msg::Path current_path_;	   ///< current path

	// Création des clients d'actions
	template<typename ActionT>
	typename ActionClient<ActionT>::SharedPtr create_action_client(const std::string &action_name)
	{
		auto client = rclcpp_action::create_client<ActionT>(this, action_name);
		return client;
	}

	// Attente des serveurs d'actions
	template<typename ActionT>
	bool wait_for_action_server(typename ActionClient<ActionT>::SharedPtr client, const std::string &action_name, const std::chrono::seconds timeout)
	{
		if (!client->wait_for_action_server(timeout))
		{
			RCLCPP_ERROR(this->get_logger(), "%s action server not available after waiting for %ld seconds", action_name.c_str(), timeout.count());
			return false;
		}
		return true;
	}

	// Envoi de goals
	template<typename ActionT>
	void send_goal(int target_number, typename ActionClient<ActionT>::SharedPtr client,	const std::string &action_name,
					std::function<void(const typename GoalHandle<ActionT>::WrappedResult &)> result_callback)
	{
		auto goal_msg = typename ActionT::Goal();
		goal_msg.target_number = target_number;

		RCLCPP_INFO(this->get_logger(), "Sending %s goal to count to %d", action_name.c_str(), target_number);

		auto send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
		send_goal_options.feedback_callback =
		[this, action_name](typename GoalHandle<ActionT>::SharedPtr, const std::shared_ptr<const typename ActionT::Feedback> feedback)
		{
			RCLCPP_INFO(this->get_logger(), "%s Feedback: Current count = %d",
						action_name.c_str(), feedback->current_number);
		};
		send_goal_options.result_callback = result_callback;

		client->async_send_goal(goal_msg, send_goal_options);
	}

	void send_navigation_goal(const nav_msgs::msg::Path & path);
	void send_inspection_goal(const nav_msgs::msg::Path & path);
	void send_stabilization_goal(int target_number);
	void send_rotation_goal(int target_number);

	// Gestion des résultats
	template<typename ActionT>
	void handle_result(const typename GoalHandle<ActionT>::WrappedResult & result, const std::string &action_name)
	{
		switch (result.code)
		{
			case rclcpp_action::ResultCode::SUCCEEDED:
				RCLCPP_INFO(this->get_logger(), "%s succeeded with final count = %d", action_name.c_str(), true);
				break;
			case rclcpp_action::ResultCode::ABORTED:
				RCLCPP_ERROR(this->get_logger(), "%s was aborted", action_name.c_str());
				break;
			case rclcpp_action::ResultCode::CANCELED:
				RCLCPP_WARN(this->get_logger(), "%s was canceled", action_name.c_str());
				break;
			default:
				RCLCPP_ERROR(this->get_logger(), "Unknown result code for %s", action_name.c_str());
				break;
		}
	}

	void handle_navigation_result(const GoalHandle<Navigation>::WrappedResult & result);
	void handle_inspection_result(const GoalHandle<Inspection>::WrappedResult & result);
	void handle_stabilization_result(const GoalHandle<Stabilization>::WrappedResult & result);
	void handle_rotation_result(const GoalHandle<Rotation>::WrappedResult & result);
};

#endif  // MISSION_MANAGER_DEMO__MISSION_MANAGER_HPP_
