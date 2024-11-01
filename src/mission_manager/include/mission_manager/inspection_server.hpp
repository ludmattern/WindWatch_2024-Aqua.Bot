#ifndef MISSION_MANAGER__INSPECTION_SERVER_HPP_
#define MISSION_MANAGER__INSPECTION_SERVER_HPP_

#include <memory>
#include <vector>
#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "mission_manager/action/inspection.hpp"
#include "mission_manager/PIDController.hpp"

class InspectionServer : public rclcpp::Node
{
public:
    using Inspection = mission_manager::action::Inspection;
    using GoalHandleInspection = rclcpp_action::ServerGoalHandle<Inspection>;

    InspectionServer();

private:
    // Action server
    rclcpp_action::Server<Inspection>::SharedPtr action_server_;

    // Publishers and subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdPublisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

    // Odometry data
    nav_msgs::msg::Odometry current_odometry_;
    std::mutex odom_mutex_;
    bool odom_received_;

	struct OdometryData
	{
		double pos_x;
		double pos_y;
		double pos_z;
		double yaw;
		double linear_velocity;
		double distanceToTarget;
	};

    // Path and waypoints
    std::vector<geometry_msgs::msg::PoseStamped> path_;
    size_t targetIndex_;

    // Control flags
    bool goalCancelled_;

    // Action server callbacks
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Inspection::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleInspection> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleInspection> goal_handle);


    // Execution and control loop
    void execute(const std::shared_ptr<GoalHandleInspection> goal_handle);
    void controlLoop(const std::shared_ptr<GoalHandleInspection> goal_handle);

    // Odometry callback
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
	bool isGoalReached(void);

	// PID controllers
	PIDController headingController_;
	PIDController speedController_;

};

#endif  // MISSION_MANAGER__INSPECTION_SERVER_HPP_
