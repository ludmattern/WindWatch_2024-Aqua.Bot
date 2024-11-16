// src/mission_manager.cpp

#include "mission_manager/mission_manager.hpp"

using namespace std::chrono_literals;

MissionManager::MissionManager(): Node("mission_manager"),  sequence1_iteration_count_(0),  sequence1_max_iterations_(0), timer_(nullptr),  current_sequence_(Sequence::SEQUENCE1)
{
	current_path_ = nav_msgs::msg::Path();

	navigation_client_ = create_action_client<Navigation>("navigation");
	inspection_client_ = create_action_client<Inspection>("inspection");
	stabilization_client_ = create_action_client<Stabilization>("stabilization");
	rotation_client_ = create_action_client<Rotation>("rotation");
	targetManagerClient_ = this->create_client<mission_manager::srv::TargetManagerServ>("mission/mission_goal");

	RCLCPP_INFO(this->get_logger(), "Action Manager has been started.");

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
	timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MissionManager::launch, this));
}

void MissionManager::launch()
{
	timer_->cancel();

	while (!this->targetManagerClient_->wait_for_service(std::chrono::seconds(1)))
		RCLCPP_ERROR(this->get_logger(), "Service 'mission/mission_goal' not available");

	RCLCPP_INFO(this->get_logger(), "Service is available. Sending request...");


	auto request = std::make_shared<mission_manager::srv::TargetManagerServ::Request>();
	auto future = this->targetManagerClient_->async_send_request(
		request,
		std::bind(&MissionManager::service_response_callback, this, std::placeholders::_1)
	);
}

void MissionManager::service_response_callback(rclcpp::Client<mission_manager::srv::TargetManagerServ>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), "Received Path from TargetManagerService");

	auto response = future.get();
	// si le path est vide ou si le nombre de cibles est nul
	if (response->path.poses.empty() || response->targetcount.data == 0)
	{
		RCLCPP_WARN(this->get_logger(), "Received empty path from TargetManagerService. Retrying...");

		timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MissionManager::launch, this));
		return ;
	}

	RCLCPP_INFO(this->get_logger(), "Processing received path");
	// changer max iteration
	sequence1_max_iterations_ = response->targetcount.data;
	if (!current_path_.poses.empty())
		current_path_.poses.clear();
	current_path_ = response->path;
	send_navigation_goal(current_path_);
}

void MissionManager::send_navigation_goal(const nav_msgs::msg::Path & path)
{
	auto goal_msg = Navigation::Goal();
	goal_msg.path = path;

	auto send_goal_options = rclcpp_action::Client<Navigation>::SendGoalOptions();
	send_goal_options.result_callback = std::bind(&MissionManager::handle_navigation_result, this, std::placeholders::_1);

	navigation_client_->async_send_goal(goal_msg, send_goal_options);
}

void MissionManager::send_inspection_goal(const nav_msgs::msg::Path & path)
{
	RCLCPP_INFO(this->get_logger(), "Path inspection x: %f y: %f", path.poses[0].pose.position.x, path.poses[0].pose.position.y);
	auto goal_msg = Inspection::Goal();
	goal_msg.path = path;

	auto send_goal_options = rclcpp_action::Client<Inspection>::SendGoalOptions();
	send_goal_options.result_callback = std::bind(&MissionManager::handle_inspection_result, this, std::placeholders::_1);

	inspection_client_->async_send_goal(goal_msg, send_goal_options);
}

void MissionManager::send_stabilization_goal(const nav_msgs::msg::Path & path)
{
	auto goal_msg = Stabilization::Goal();
	goal_msg.path = path;

	auto send_goal_options = rclcpp_action::Client<Stabilization>::SendGoalOptions();
	send_goal_options.result_callback = std::bind(&MissionManager::handle_stabilization_result, this, std::placeholders::_1);

	stabilization_client_->async_send_goal(goal_msg, send_goal_options);
}

void MissionManager::send_rotation_goal(int target_number)
{
	send_goal<Rotation>(target_number, rotation_client_, "rotation",
		std::bind(&MissionManager::handle_rotation_result, this, std::placeholders::_1));
}

void MissionManager::handle_navigation_result(const GoalHandle<Navigation>::WrappedResult & result)
{
	handle_result<Navigation>(result, "Navigation");

	if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
		return;
	if (current_sequence_ == Sequence::SEQUENCE1)
	{
		RCLCPP_INFO(this->get_logger(), "Transitioning to next point INSPECTION in SEQUENCE1");
		send_inspection_goal(current_path_);
	}
	else if (current_sequence_ == Sequence::SEQUENCE2)
	{
		RCLCPP_INFO(this->get_logger(), "Transitioning to stabilization in SEQUENCE2");
		send_stabilization_goal(current_path_);
	}
}

void MissionManager::handle_inspection_result(const GoalHandle<Inspection>::WrappedResult & result)
{
	handle_result<Inspection>(result, "Inspection");

	if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
		return;
	sequence1_iteration_count_++;
	if (sequence1_iteration_count_ < sequence1_max_iterations_)
	{
		RCLCPP_INFO(this->get_logger(), "Transitioning to next point NAVIGATION in SEQUENCE1");
		auto request = std::make_shared<mission_manager::srv::TargetManagerServ::Request>();
		auto future = this->targetManagerClient_->async_send_request(
			request,
			std::bind(&MissionManager::service_response_callback, this, std::placeholders::_1)
		);
	}
	else
	{
		current_sequence_ = Sequence::SEQUENCE2;
		RCLCPP_INFO(this->get_logger(), "Transitioning to SEQUENCE2 first point NAVIGATION");
		auto request = std::make_shared<mission_manager::srv::TargetManagerServ::Request>();
		auto future = this->targetManagerClient_->async_send_request(
			request,
			std::bind(&MissionManager::service_response_callback, this, std::placeholders::_1)
		);
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
