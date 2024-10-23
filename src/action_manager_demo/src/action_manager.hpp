// src/action_manager.hpp

#ifndef ACTION_MANAGER_DEMO__ACTION_MANAGER_HPP_
#define ACTION_MANAGER_DEMO__ACTION_MANAGER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_manager_demo/action/action1.hpp"
#include "action_manager_demo/action/action2.hpp"

class ActionManager : public rclcpp::Node
{
public:
  using Action1 = action_manager_demo::action::Action1;
  using Action2 = action_manager_demo::action::Action2;
  using GoalHandleAction1 = rclcpp_action::ClientGoalHandle<Action1>;
  using GoalHandleAction2 = rclcpp_action::ClientGoalHandle<Action2>;

  ActionManager();

private:
  rclcpp_action::Client<Action1>::SharedPtr action1_client_;
  rclcpp_action::Client<Action2>::SharedPtr action2_client_;

  void send_action1_goal(int target_number);
  void send_action2_goal(int target_number);

  void handle_action1_result(const GoalHandleAction1::WrappedResult & result);
  void handle_action2_result(const GoalHandleAction2::WrappedResult & result);

  // Callbacks for feedback
  void handle_action1_feedback(
    GoalHandleAction1::SharedPtr,
    const std::shared_ptr<const Action1::Feedback> feedback);

  void handle_action2_feedback(
    GoalHandleAction2::SharedPtr,
    const std::shared_ptr<const Action2::Feedback> feedback);
};

#endif  // ACTION_MANAGER_DEMO__ACTION_MANAGER_HPP_
