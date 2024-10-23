// src/action2_server.cpp

#include <memory>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_manager_demo/action/action2.hpp"

using namespace std::chrono_literals;

class Action2Server : public rclcpp::Node
{
public:
  using Action2 = action_manager_demo::action::Action2;
  using GoalHandleAction2 = rclcpp_action::ServerGoalHandle<Action2>;

  Action2Server()
  : Node("action2_server")
  {
    action_server_ = rclcpp_action::create_server<Action2>(
      this,
      "action2",
      std::bind(&Action2Server::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&Action2Server::handle_cancel, this, std::placeholders::_1),
      std::bind(&Action2Server::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Action2 Server has been started.");
  }

private:
  rclcpp_action::Server<Action2>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Action2::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received Action2 goal request to count to %d", goal->target_number);
    // Accepter tous les objectifs
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleAction2> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel Action2 goal.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleAction2> goal_handle)
  {
    // Exécuter la tâche dans un thread séparé
    std::thread{std::bind(&Action2Server::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleAction2> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing Action2 goal...");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Action2::Feedback>();
    auto result = std::make_shared<Action2::Result>();

    for(int i = 1; i <= goal->target_number; ++i)
    {
      if (goal_handle->is_canceling())
      {
        RCLCPP_INFO(this->get_logger(), "Action2 goal canceled.");
        goal_handle->canceled(result);
        result->final_count = i - 1;
        return;
      }

      // Simuler un travail en cours
      std::this_thread::sleep_for(1s);

      // Publier le feedback
      feedback->current_number = i;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Action2 Feedback: %d/%d", i, goal->target_number);
    }

    // Finaliser l'action
    result->final_count = goal->target_number;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Action2 goal succeeded with final count: %d", result->final_count);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Action2Server>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
