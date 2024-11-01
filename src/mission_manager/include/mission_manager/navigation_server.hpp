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
#include "mission_manager/pid_controller.hpp"
#include "mission_manager/Point.hpp"

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

    // Path and waypoints
    std::vector<geometry_msgs::msg::PoseStamped> path_;
    size_t current_waypoint_index_;

    // Acceleration parameters
    double max_acceleration_;
    double max_deceleration_;

    // Control parameters
    double position_tolerance_;
    double control_loop_rate_;
    double initial_distance_to_goal_;
    double estimated_disturbance_angular_;
    size_t last_waypoint_index_;

    // PID parameters
    double Kp_linear_;
    double Ki_linear_;
    double Kd_linear_;
    double Kp_angular_;
    double Ki_angular_;
    double Kd_angular_;
    double Kd_disturbance_;

    // Speed limits
    double max_linear_speed_;
    double min_linear_speed_;
    double max_angular_speed_;
    double current_linear_speed_;

    // PID Controllers
    PIDController linear_pid_;
    PIDController angular_pid_;

    // Control flags
    bool goal_cancelled_;

	Point starting_point_;
	
    // Action server callbacks
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const Navigation::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleNavigation> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleNavigation> goal_handle);

    // Execution and control loop
    void execute(const std::shared_ptr<GoalHandleNavigation> goal_handle);
    void controlLoop(const std::shared_ptr<GoalHandleNavigation> goal_handle);

    // Odometry callback
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
};

#endif  // MISSION_MANAGER__NAVIGATION_SERVER_HPP_
