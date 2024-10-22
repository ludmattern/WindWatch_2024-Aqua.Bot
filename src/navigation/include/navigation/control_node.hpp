// control_node.hpp

#ifndef CONTROL_NODE_HPP
#define CONTROL_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <vector>
#include <memory>

class ControlNode : public rclcpp::Node
{
public:
    ControlNode();

private:
    // Callbacks
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void controlLoop();
    void checkCommandTimeout();

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;

    // Parameters
    double Kp_linear_;
    double Ki_linear_;
    double Kd_linear_;
    double Kp_angular_;
    double Ki_angular_;
    double Kd_angular_;
    double position_tolerance_; // meters
    double control_loop_rate_;  // Hz
    double timeout_duration_;   // seconds

    // State
    geometry_msgs::msg::Pose current_pose_;
    std::vector<geometry_msgs::msg::PoseStamped> path_;
    size_t current_waypoint_index_;
    bool goal_received_;

    // PID Controller variables
    double prev_error_linear_;
    double prev_error_angular_;
    double integral_error_linear_;
    double integral_error_angular_;

    // Timer for control loop
    rclcpp::TimerBase::SharedPtr timer_;

    // Timer for command timeout
    rclcpp::TimerBase::SharedPtr timeout_timer_;
    bool command_received_;
};

#endif // CONTROL_NODE_HPP
