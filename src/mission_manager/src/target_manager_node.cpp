#include "mission_manager/target_manager_node.hpp"
#include "std_msgs/msg/int32.hpp"

TargetManagerNode::TargetManagerNode() : Node("target_manager_node"), timer_(nullptr)
{
	RCLCPP_INFO(this->get_logger(), "Target Manager Node has started");
	callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

	TargetManagerService_ = this->create_service<mission_manager::srv::TargetManagerServ>(
		"/mission/mission_goal", std::bind(&TargetManagerNode::ServerCallback, this, std::placeholders::_1, std::placeholders::_2), 
		rmw_qos_profile_services_default, callback_group_);

	// Create the client for the service
	// TmaPos_Client_ = this->create_client<sensors::srv::TargetPositions>("mission/target_positions");

	// Define a Timer to launch the connection after initialization
	// timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&TargetManagerNode::launch, this));
}

// void TargetManagerNode::launch()
// {
// 	timer_->cancel();

// 	while (!this->TmaPos_Client_->wait_for_service(std::chrono::seconds(1)))
// 		RCLCPP_ERROR(this->get_logger(), "Service 'mission/target_positions' not available");

// 	RCLCPP_INFO(this->get_logger(), "Service is available. Sending request...");

// 	auto request = std::make_shared<sensors::srv::TargetPositions::Request>();
// 	auto future = this->TmaPos_Client_->async_send_request(
// 		request,
// 		std::bind(&TargetManagerNode::service_response_callback, this, std::placeholders::_1)
// 	);
// }

// void TargetManagerNode::service_response_callback(
// 	rclcpp::Client<sensors::srv::TargetPositions>::SharedFuture future)
// {
// 	RCLCPP_INFO(this->get_logger(), "Received response from service");

// 	auto response = future.get();

// 	if (response->poses.poses.empty())
// 	{
// 		RCLCPP_WARN(this->get_logger(), "Received empty poses from service. Retrying...");

// 		// Optionally, you can retry the request after some delay
// 		// For example, using a one-shot timer:
// 		auto retry_timer = this->create_wall_timer(
// 			std::chrono::seconds(5),
// 			[this]() {
// 				this->launch();
// 			}
// 		);
// 	}
// 	else
// 	{
// 		RCLCPP_INFO(this->get_logger(), "Processing received poses");
// 		this->TmaPosRegister(response->poses);
// 	}
// }

void TargetManagerNode::TmaPosRegister(geometry_msgs::msg::PoseArray msg)
{
	if (msg.poses.empty()) {
		RCLCPP_WARN(this->get_logger(), "Received message without poses.");
		return;
	}
	wind_data_.wind.poses.clear();
	wind_data_.wind.poses.reserve(msg.poses.size());

	// Reset and resize the status vector
	wind_data_.status.clear();
	wind_data_.status.resize(msg.poses.size(), false); // Initialize all statuses to false

	for (size_t i = 0; i < msg.poses.size(); ++i)
	{
		geometry_msgs::msg::Pose pose = msg.poses[i];
		wind_data_.wind.poses.push_back(pose);

		// Initialize the status
		wind_data_.status[i] = false;

		RCLCPP_INFO(this->get_logger(), "Wind %zu - x: %f, y: %f, z: %f",
					i, pose.position.x, pose.position.y, pose.position.z);
	}

	wind_data_.nb_wind = static_cast<int>(msg.poses.size());
	RCLCPP_INFO(this->get_logger(), "Total number of winds recorded: %d", wind_data_.nb_wind);
}

void TargetManagerNode::ServerCallback(const std::shared_ptr<mission_manager::srv::TargetManagerServ::Request> request,
	const std::shared_ptr<mission_manager::srv::TargetManagerServ::Response> response)
{
	nav_msgs::msg::Path temp = nav_msgs::msg::Path();
	std_msgs::msg::Int32 targetcount = std_msgs::msg::Int32();
	targetcount.data = 3;

	temp.header.frame_id = "map";
	temp.poses.resize(2);

	temp.poses[0].pose.position.x = 127.0;
	temp.poses[0].pose.position.y = 33.0;
	temp.poses[0].pose.orientation.w = 1.0;

	temp.poses[1].pose.position.x = 219.19;
	temp.poses[1].pose.position.y = 290.79;
	temp.poses[1].pose.orientation.w = 1.0;

	response->path = temp;
	response->targetcount = targetcount;

	RCLCPP_INFO(this->get_logger(), "Réponse envoyée.");
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<TargetManagerNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
