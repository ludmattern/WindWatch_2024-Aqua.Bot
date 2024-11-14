#ifndef TARGET_MANAGER_NODE_HPP
#define TARGET_MANAGER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <iostream>
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "mission_manager/srv/target_manager_serv.hpp" // Inclusion du service
#include "navigation/srv/path.hpp" // Si nécessaire
#include "navigation/srv/path_last.hpp"
#include "nav_msgs/msg/odometry.hpp"



class TargetManagerNode : public rclcpp::Node
{
public:
    TargetManagerNode();
    void TmaPosRegister(geometry_msgs::msg::PoseArray msg);
    void launch();
    void launch_last();
    void WindInspection(const nav_msgs::msg::Odometry::SharedPtr msg);

private:
    rclcpp::Service<mission_manager::srv::TargetManagerServ>::SharedPtr TargetManagerService_;
    rclcpp::Client<navigation::srv::Path>::SharedPtr TargetPath_Client_;
    //rclcpp::Client<sensors::srv::Status>::SharedPtr TargetStatus_Client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr  wind_insp_subscription_;

    void ServerCallback(
        const std::shared_ptr<mission_manager::srv::TargetManagerServ::Request> request,
        const std::shared_ptr<mission_manager::srv::TargetManagerServ::Response> response);

     //void ServerCallback(const std::shared_ptr<sensors::srv::WindInsp::Request> request,
	   // const std::shared_ptr<sensors::srv::WindInsp::Response> response);

    void service_response_callback(
        rclcpp::Client<navigation::srv::Path>::SharedFuture future);
    void service_response_callback_last(
        rclcpp::Client<navigation::srv::PathLast>::SharedFuture future);
    void PathPlan(nav_msgs::msg::Path path);

    struct s_wind
    {
        geometry_msgs::msg::PoseArray wind;
        std::vector<double> pos_wind;
        int nb_wind;
        std::vector<bool> status;
        std::vector<std_msgs::msg::String> qr;
        //std::vector<std_msgs::msg::String> status_string;
    };
    struct s_path
    {
        nav_msgs::msg::Path temp_;
        std::vector<bool> status;
    };
    s_path path_data_;
    s_wind wind_data_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;


    nav_msgs::msg::Odometry ship;
    nav_msgs::msg::Path last_path;
    bool shipAdd = false;
    bool path_sent = false;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_Subscription_;
	rclcpp::Client<navigation::srv::PathLast>::SharedPtr LastPath_Client_;
	rclcpp::TimerBase::SharedPtr timer_inspec_;



    int wind_def;
};

#endif // TARGET_MANAGER_NODE_HPP