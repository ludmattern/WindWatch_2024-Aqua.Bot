#ifndef MISSION_MANAGER__NAVIGATION_SERVER_HPP_
#define MISSION_MANAGER__NAVIGATION_SERVER_HPP_

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
#include "mission_manager/action/navigation.hpp"
#include "mission_manager/PIDController.hpp"

class NavigationServer : public rclcpp::Node
{
public:
    using Navigation = mission_manager::action::Navigation;
    using GoalHandleNavigation = rclcpp_action::ServerGoalHandle<Navigation>;

    NavigationServer();

private:
    // Action server
    rclcpp_action::Server<Navigation>::SharedPtr action_server_;

    // Publishers and subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
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
		double distance_to_target;
	};

    // Path and waypoints
    std::vector<geometry_msgs::msg::PoseStamped> path_;
    size_t target_index_;

    // Control flags
    bool goal_cancelled_;

    // Action server callbacks
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Navigation::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNavigation> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleNavigation> goal_handle);


    // Execution and control loop
    void execute(const std::shared_ptr<GoalHandleNavigation> goal_handle);
    void controlLoop(const std::shared_ptr<GoalHandleNavigation> goal_handle);

    // Odometry callback
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
	OdometryData getOdometryData(const geometry_msgs::msg::PoseStamped & target);
	double getTgtAngleError(const OdometryData & odometryData, const geometry_msgs::msg::PoseStamped & target);
	double calculatePIDHeadingOutput(double angleError, double distanceError);

	// PID controllers
	PIDController headingController_;
	PIDController speedController_;

};

#endif  // MISSION_MANAGER__NAVIGATION_SERVER_HPP_
