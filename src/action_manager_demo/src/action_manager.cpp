// src/action_manager.cpp

#include "action_manager.hpp"

using namespace std::chrono_literals;

ActionManager::ActionManager()
: Node("action_manager")
{
  // Créer les clients d'actions
  action1_client_ = rclcpp_action::create_client<Action1>(this, "action1");
  action2_client_ = rclcpp_action::create_client<Action2>(this, "action2");

  RCLCPP_INFO(this->get_logger(), "Action Manager has been started.");

  // Attendre que les serveurs d'actions soient disponibles
  if (!action1_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(), "Action1 server not available after waiting");
    rclcpp::shutdown();
    return;
  }

  if (!action2_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(), "Action2 server not available after waiting");
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Both Action1 and Action2 servers are available.");

  // Lancer Action1 puis Action2
  send_action1_goal(5);  // Par exemple, compter jusqu'à 5
}

void ActionManager::send_action1_goal(int target_number)
{
  auto goal_msg = Action1::Goal();
  goal_msg.target_number = target_number;

  RCLCPP_INFO(this->get_logger(), "Sending Action1 goal to count to %d", target_number);

  auto send_goal_options = rclcpp_action::Client<Action1>::SendGoalOptions();
  send_goal_options.feedback_callback =
    std::bind(&ActionManager::handle_action1_feedback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&ActionManager::handle_action1_result, this, std::placeholders::_1);

  action1_client_->async_send_goal(goal_msg, send_goal_options);
}

void ActionManager::handle_action1_feedback(
  GoalHandleAction1::SharedPtr,
  const std::shared_ptr<const Action1::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Action1 Feedback: Current count = %d", feedback->current_number);
}

void ActionManager::handle_action1_result(const GoalHandleAction1::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Action1 succeeded with final count = %d", result.result->final_count);
      // Après Action1, lancer Action2
      send_action2_goal(3);  // Par exemple, compter jusqu'à 3
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Action1 was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "Action1 was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code for Action1");
      return;
  }
}

void ActionManager::send_action2_goal(int target_number)
{
  auto goal_msg = Action2::Goal();
  goal_msg.target_number = target_number;

  RCLCPP_INFO(this->get_logger(), "Sending Action2 goal to count to %d", target_number);

  auto send_goal_options = rclcpp_action::Client<Action2>::SendGoalOptions();
  send_goal_options.feedback_callback =
    std::bind(&ActionManager::handle_action2_feedback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&ActionManager::handle_action2_result, this, std::placeholders::_1);

  action2_client_->async_send_goal(goal_msg, send_goal_options);
}

void ActionManager::handle_action2_feedback(
  GoalHandleAction2::SharedPtr,
  const std::shared_ptr<const Action2::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Action2 Feedback: Current count = %d", feedback->current_number);
}

void ActionManager::handle_action2_result(const GoalHandleAction2::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Action2 succeeded with final count = %d", result.result->final_count);
      RCLCPP_INFO(this->get_logger(), "All actions have been executed successfully.");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Action2 was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "Action2 was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code for Action2");
      return;
  }
}
