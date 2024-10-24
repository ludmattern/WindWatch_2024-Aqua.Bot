// src/stabilization_server.cpp

#include <memory>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_manager_demo/action/stabilization.hpp"

using namespace std::chrono_literals;

class StabilizationServer : public rclcpp::Node
{
public:
  using Stabilization = action_manager_demo::action::Stabilization;
  using GoalHandleStabilization = rclcpp_action::ServerGoalHandle<Stabilization>;

  StabilizationServer()
  : Node("stabilization_server")
  {
    action_server_ = rclcpp_action::create_server<Stabilization>(
      this,
      "stabilization",
      std::bind(&StabilizationServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&StabilizationServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&StabilizationServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Stabilization Server has been started.");
  }

private:
  rclcpp_action::Server<Stabilization>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Stabilization::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received Stabilization goal request to count to %d", goal->target_number);
    // Accepter tous les objectifs
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleStabilization> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel Stabilization goal.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleStabilization> goal_handle)
  {
    // Exécuter la tâche dans un thread séparé
    std::thread{std::bind(&StabilizationServer::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleStabilization> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing Stabilization goal...");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Stabilization::Feedback>();
    auto result = std::make_shared<Stabilization::Result>();

    for(int i = 1; i <= goal->target_number; ++i)
    {
      if (goal_handle->is_canceling())
      {
        RCLCPP_INFO(this->get_logger(), "Stabilization goal canceled.");
        goal_handle->canceled(result);
        result->final_count = i - 1;
        return;
      }

      // Simuler un travail en cours
      std::this_thread::sleep_for(1s);

      // Publier le feedback
      feedback->current_number = i;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Stabilization Feedback: %d/%d", i, goal->target_number);
    }

    // Finaliser l'action
    result->final_count = goal->target_number;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Stabilization goal succeeded with final count: %d", result->final_count);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StabilizationServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
