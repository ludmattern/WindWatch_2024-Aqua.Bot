// src/mission_manager.cpp

#include "mission_manager/mission_manager.hpp"

using namespace std::chrono_literals;

MissionManager::MissionManager(): Node("mission_manager"),  sequence1_iteration_count_(0),  sequence1_max_iterations_(3),  current_sequence_(Sequence::SEQUENCE1)
{
	current_path_ = nav_msgs::msg::Path();

	// Initialisation des clients d'actions
	navigation_client_ = create_action_client<Navigation>("navigation");
	inspection_client_ = create_action_client<Inspection>("inspection");
	stabilization_client_ = create_action_client<Stabilization>("stabilization");
	rotation_client_ = create_action_client<Rotation>("rotation");

	RCLCPP_INFO(this->get_logger(), "Action Manager has been started.");

	// Attendre que tous les serveurs d'actions soient disponibles
	if (!wait_for_action_server<Navigation>(navigation_client_, "navigation", 5s) ||
		!wait_for_action_server<Inspection>(inspection_client_, "inspection", 5s) ||
		!wait_for_action_server<Stabilization>(stabilization_client_, "stabilization", 5s) ||
		!wait_for_action_server<Rotation>(rotation_client_, "rotation", 5s))
	{
		RCLCPP_ERROR(this->get_logger(), "One or more action servers are not available.");
		rclcpp::shutdown();
		return;
	}

	RCLCPP_INFO(this->get_logger(), "All action servers are available.");

	// Attendre que tout le monde soit prêt
		//boucle branchee sur un topic

	// Demander le nombre d'objectifs à atteindre
		//service pour demander le nombre d'objectifs et le prochain itinéraire
			//attendre la réponse et changer sequence1_max_iterations_
			//attendre la reponse et stocker le prochain itinéraire (path_)

	// Commencer la Séquence 1 en envoyant Navigation avec le prochain objectif (current_path_)
	send_navigation_goal(current_path_);
}

void MissionManager::send_navigation_goal(const nav_msgs::msg::Path & path)
{
	auto goal_msg = Navigation::Goal();
	goal_msg.path = path;

	// Envoyer le goal comme précédemment, en utilisant le client d'action
	auto send_goal_options = rclcpp_action::Client<Navigation>::SendGoalOptions();
	send_goal_options.result_callback = std::bind(&MissionManager::handle_navigation_result, this, std::placeholders::_1);

	navigation_client_->async_send_goal(goal_msg, send_goal_options);
}

void MissionManager::send_inspection_goal(int target_number)
{
	send_goal<Inspection>(target_number, inspection_client_, "inspection",
		std::bind(&MissionManager::handle_inspection_result, this, std::placeholders::_1));
}

void MissionManager::send_stabilization_goal(int target_number)
{
	send_goal<Stabilization>(target_number, stabilization_client_, "stabilization",
		std::bind(&MissionManager::handle_stabilization_result, this, std::placeholders::_1));
}

void MissionManager::send_rotation_goal(int target_number)
{
	send_goal<Rotation>(target_number, rotation_client_, "rotation",
		std::bind(&MissionManager::handle_rotation_result, this, std::placeholders::_1));
}

void MissionManager::handle_navigation_result(const GoalHandle<Navigation>::WrappedResult & result)
{
	handle_result<Navigation>(result, "Navigation");

	if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
	{
		if (current_sequence_ == Sequence::SEQUENCE1)
			send_inspection_goal(3);
		else if (current_sequence_ == Sequence::SEQUENCE2)
			send_stabilization_goal(3);
	}
}

void MissionManager::handle_inspection_result(const GoalHandle<Inspection>::WrappedResult & result)
{
	handle_result<Inspection>(result, "Inspection");

	if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
		return;
	if (current_sequence_ == Sequence::SEQUENCE1)
	{
		sequence1_iteration_count_++;
		if (sequence1_iteration_count_ < sequence1_max_iterations_)
			send_navigation_goal(current_path_);
		else
		{
			// Transition vers la Séquence 2
			current_sequence_ = Sequence::SEQUENCE2;
			RCLCPP_INFO(this->get_logger(), "Transitioning to SEQUENCE2");
			// Commencer la Séquence 2 en envoyant Navigation
			send_navigation_goal(current_path_);
		}
	}
}

void MissionManager::handle_stabilization_result(const GoalHandle<Stabilization>::WrappedResult & result)
{
	handle_result<Stabilization>(result, "Stabilization");

	if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
		send_rotation_goal(10);
}

void MissionManager::handle_rotation_result(const GoalHandle<Rotation>::WrappedResult & result)
{
	handle_result<Rotation>(result, "Rotation");

	if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
	{
		// Toutes les séquences sont terminées
		current_sequence_ = Sequence::DONE;
		RCLCPP_INFO(this->get_logger(), "All action sequences have been executed successfully.");
	}
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<MissionManager>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
