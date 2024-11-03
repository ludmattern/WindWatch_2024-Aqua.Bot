#ifndef TARGET_MANAGER_NODE_HPP
#define TARGET_MANAGER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <iostream>
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensors/srv/target_positions.hpp"
//#include "mission_manager/srv/target_manager_nodes.hpp"

class TargetManagerNode : public rclcpp::Node
{
public:
	TargetManagerNode();
	//Client for target position
	rclcpp::Client<sensors::srv::TargetPositions>::SharedPtr TmaPos_Client_;
	void TmaPosRegister(geometry_msgs::msg::PoseArray msg);
	void launch();


private:
	//Service
	//rclcpp::Service<mission_manager::srv::TargetManagerNodes>::SharedPtr _TmaPos_Service;
	// Structure pour stocker les données des vents
	
	/* void ServerCallback(const std::shared_ptr<mission_manager::srv::TargetManagerNodes::Request> request,
		const std::shared_ptr<mission_manager::srv::TargetManagerNodes::Response> response);
*/
	geometry_msgs::msg::PoseArray MakeRequest(sensors::srv::TargetPositions::Request::SharedPtr request);
	void service_response_callback(rclcpp::Client<sensors::srv::TargetPositions>::SharedFuture future);

	struct s_wind
	{
		geometry_msgs::msg::PoseArray wind;
		int nb_wind;
		std::vector<bool> status; // Utilisation de std::vector<bool> au lieu de bool*
	};

	s_wind wind_data_;
	rclcpp::TimerBase::SharedPtr timer_;
	//int _nb_wind_to_;
};

#endif // TARGET_MANAGER_NODE_HPP