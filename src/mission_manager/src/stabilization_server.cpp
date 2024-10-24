// src/stabilization_server.cpp

#include "stabilization_server.hpp"

#include <memory>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

StabilizationServer::StabilizationServer() : Node("stabilization_server")
{
	action_server_ = rclcpp_action::create_server<Stabilization>(
		this,
		"stabilization",
		[this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Stabilization::Goal> goal) {
		return handle_goal(uuid, goal);
		},
		[this](const std::shared_ptr<GoalHandleStabilization> goal_handle) {
		return handle_cancel(goal_handle);
		},
		[this](const std::shared_ptr<GoalHandleStabilization> goal_handle) {
		handle_accepted(goal_handle);
		});

	RCLCPP_INFO(this->get_logger(), "Stabilization Server has been started.");
}

rclcpp_action::GoalResponse StabilizationServer::handle_goal(
const rclcpp_action::GoalUUID & uuid,
std::shared_ptr<const Stabilization::Goal> goal)
{
	RCLCPP_INFO(this->get_logger(), "Received Stabilization goal request to count to %d", goal->target_number);
	// Accepter tous les objectifs
	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse StabilizationServer::handle_cancel(
const std::shared_ptr<GoalHandleStabilization> goal_handle)
{
	RCLCPP_INFO(this->get_logger(), "Received request to cancel Stabilization goal.");
	// Accepter toutes les demandes d'annulation
	return rclcpp_action::CancelResponse::ACCEPT;
}

void StabilizationServer::handle_accepted(const std::shared_ptr<GoalHandleStabilization> goal_handle)
{
	// Créer un nouveau thread pour exécuter l'objectif
	std::thread(
		[this, goal_handle]() {
		execute(goal_handle);
		}
	).detach();
}

void StabilizationServer::execute(const std::shared_ptr<GoalHandleStabilization> goal_handle)
{
	RCLCPP_INFO(this->get_logger(), "Executing Stabilization goal...");
	const auto goal = goal_handle->get_goal();
	auto feedback = std::make_shared<Stabilization::Feedback>();
	auto result = std::make_shared<Stabilization::Result>();

	rclcpp::Rate loop_rate(1); // Fréquence de 1 Hz

	try
	{
		for(int i = 1; i <= goal->target_number; ++i)
		{
		// Vérifier si l'objectif a été annulé
		if (goal_handle->is_canceling())
		{
			result->final_count = i - 1;
			goal_handle->canceled(result);
			RCLCPP_INFO(this->get_logger(), "Stabilization goal canceled.");
			return;
		}

		// Simuler un travail en cours
		loop_rate.sleep();

		// Publier le feedback
		feedback->current_number = i;
		goal_handle->publish_feedback(feedback);
		RCLCPP_INFO(this->get_logger(), "Stabilization Feedback: %d/%d", i, goal->target_number);
		}

		// Indiquer que l'objectif est réussi
		result->final_count = goal->target_number;
		goal_handle->succeed(result);
		RCLCPP_INFO(this->get_logger(), "Stabilization goal succeeded with final count: %d", result->final_count);
	}
	catch (const std::exception &e)
	{
		RCLCPP_ERROR(this->get_logger(), "Exception in execute: %s", e.what());
		goal_handle->abort(result);
	}
	}

	int main(int argc, char **argv)
	{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<StabilizationServer>());
	rclcpp::shutdown();
	return 0;
}
