// src/action_manager.cpp

#include "action_manager.hpp"

using namespace std::chrono_literals;

ActionManager::ActionManager()
: Node("action_manager"),
  sequence1_count_(0),
  sequence1_max_(3),
  current_sequence_(Sequence::SEQUENCE1)
{
  // Créer les clients d'actions
  navigation_client_ = rclcpp_action::create_client<Navigation>(this, "navigation");
  inspection_client_ = rclcpp_action::create_client<Inspection>(this, "inspection");
  stabilization_client_ = rclcpp_action::create_client<Stabilization>(this, "stabilization");
  rotation_client_ = rclcpp_action::create_client<Rotation>(this, "rotation");

  RCLCPP_INFO(this->get_logger(), "Action Manager has been started.");

  // Attendre que les serveurs d'actions soient disponibles
  if (!navigation_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(), "Navigation server not available after waiting");
    rclcpp::shutdown();
    return;
  }

  if (!inspection_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(), "Inspection server not available after waiting");
    rclcpp::shutdown();
    return;
  }

  if (!stabilization_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(), "Stabilization server not available after waiting");
    rclcpp::shutdown();
    return;
  }

  if (!rotation_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(), "Rotation server not available after waiting");
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Navigation, Inspection, Stabilization and Rotation servers are available.");

	//savoir combien de fois la séquence 1 doit etre exécutée
  	send_navigation_goal(1);
}

void ActionManager::send_navigation_goal(int target_number)
{
  auto goal_msg = Navigation::Goal();
  goal_msg.target_number = target_number;

  RCLCPP_INFO(this->get_logger(), "Sending Navigation goal to count to %d", target_number);

  auto send_goal_options = rclcpp_action::Client<Navigation>::SendGoalOptions();
  send_goal_options.feedback_callback =
    [this](rclcpp_action::ClientGoalHandle<Navigation>::SharedPtr,
           const std::shared_ptr<const Navigation::Feedback> feedback) {
      RCLCPP_INFO(this->get_logger(), "Navigation Feedback: Current count = %d", feedback->current_number);
    };
  send_goal_options.result_callback =
    std::bind(&ActionManager::handle_navigation_result, this, std::placeholders::_1);

  navigation_client_->async_send_goal(goal_msg, send_goal_options);
}

void ActionManager::send_inspection_goal(int target_number)
{
  auto goal_msg = Inspection::Goal();
  goal_msg.target_number = target_number;

  RCLCPP_INFO(this->get_logger(), "Sending Inspection goal to count to %d", target_number);

  auto send_goal_options = rclcpp_action::Client<Inspection>::SendGoalOptions();
  send_goal_options.feedback_callback =
    [this](rclcpp_action::ClientGoalHandle<Inspection>::SharedPtr,
           const std::shared_ptr<const Inspection::Feedback> feedback) {
      RCLCPP_INFO(this->get_logger(), "Inspection Feedback: Current count = %d", feedback->current_number);
    };
  send_goal_options.result_callback =
    std::bind(&ActionManager::handle_inspection_result, this, std::placeholders::_1);

  inspection_client_->async_send_goal(goal_msg, send_goal_options);
}

void ActionManager::send_stabilization_goal(int target_number)
{
  auto goal_msg = Stabilization::Goal();
  goal_msg.target_number = target_number;

  RCLCPP_INFO(this->get_logger(), "Sending Stabilization goal to count to %d", target_number);

  auto send_goal_options = rclcpp_action::Client<Stabilization>::SendGoalOptions();
  send_goal_options.feedback_callback =
    [this](rclcpp_action::ClientGoalHandle<Stabilization>::SharedPtr,
           const std::shared_ptr<const Stabilization::Feedback> feedback) {
      RCLCPP_INFO(this->get_logger(), "Stabilization Feedback: Current count = %d", feedback->current_number);
    };
  send_goal_options.result_callback =
    std::bind(&ActionManager::handle_stabilization_result, this, std::placeholders::_1);

  stabilization_client_->async_send_goal(goal_msg, send_goal_options);
}

void ActionManager::send_rotation_goal(int target_number)
{
  auto goal_msg = Rotation::Goal();
  goal_msg.target_number = target_number;

  RCLCPP_INFO(this->get_logger(), "Sending Rotation goal to count to %d", target_number);

  auto send_goal_options = rclcpp_action::Client<Rotation>::SendGoalOptions();
  send_goal_options.feedback_callback =
    [this](rclcpp_action::ClientGoalHandle<Rotation>::SharedPtr,
           const std::shared_ptr<const Rotation::Feedback> feedback) {
      RCLCPP_INFO(this->get_logger(), "Rotation Feedback: Current count = %d", feedback->current_number);
    };
  send_goal_options.result_callback =
    std::bind(&ActionManager::handle_rotation_result, this, std::placeholders::_1);

  rotation_client_->async_send_goal(goal_msg, send_goal_options);
}

void ActionManager::handle_navigation_result(const rclcpp_action::ClientGoalHandle<Navigation>::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Navigation succeeded with final count = %d", result.result->final_count);
      if (current_sequence_ == Sequence::SEQUENCE1) {
        send_inspection_goal(3);
      }
      else if (current_sequence_ == Sequence::SEQUENCE2) {
        send_stabilization_goal(3);
      }
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Navigation was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "Navigation was canceled");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code for Navigation");
      break;
  }
}

void ActionManager::handle_inspection_result(const rclcpp_action::ClientGoalHandle<Inspection>::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Inspection succeeded with final count = %d", result.result->final_count);
      if (current_sequence_ == Sequence::SEQUENCE1) {
        sequence1_count_++;
        if (sequence1_count_ < sequence1_max_) {
          send_navigation_goal(5);
        }
        else {
          // Passer à la Séquence 2
          current_sequence_ = Sequence::SEQUENCE2;
          RCLCPP_INFO(this->get_logger(), "Transitioning to SEQUENCE2");
          // Commencer la Séquence 2 en envoyant Navigation
          send_navigation_goal(5);
        }
      }
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Inspection was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "Inspection was canceled");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code for Inspection");
      break;
  }
}

void ActionManager::handle_stabilization_result(const rclcpp_action::ClientGoalHandle<Stabilization>::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Stabilization succeeded with final count = %d", result.result->final_count);
      // Envoyer Rotation
      send_rotation_goal(10);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Stabilization was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "Stabilization was canceled");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code for Stabilization");
      break;
  }
}

void ActionManager::handle_rotation_result(const rclcpp_action::ClientGoalHandle<Rotation>::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Rotation succeeded with final count = %d", result.result->final_count);
      // Toutes les séquences sont terminées
      current_sequence_ = Sequence::DONE;
      RCLCPP_INFO(this->get_logger(), "All action sequences have been executed successfully.");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Rotation was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "Rotation was canceled");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code for Rotation");
      break;
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ActionManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
