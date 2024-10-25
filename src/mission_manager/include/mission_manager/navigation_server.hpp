// include/navigation_server.hpp

#ifndef MISSION_MANAGER__NAVIGATION_SERVER_HPP_
#define MISSION_MANAGER__NAVIGATION_SERVER_HPP_

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "mission_manager/action/navigation.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class NavigationServer : public rclcpp::Node
{
public:
  using Navigation = mission_manager::action::Navigation;
  using GoalHandleNavigation = rclcpp_action::ServerGoalHandle<Navigation>;

  NavigationServer();

private:
  // Action Server
  rclcpp_action::Server<Navigation>::SharedPtr action_server_;

  // Publishers and Subscribers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

  // Current Odometry
  nav_msgs::msg::Odometry current_odometry_;
  bool odom_received_;

  // Path and Control Variables
  std::vector<geometry_msgs::msg::PoseStamped> path_;
  size_t current_waypoint_index_;
  bool goal_cancelled_;

  // PID Parameters
  double Kp_linear_, Ki_linear_, Kd_linear_;
  double Kp_angular_, Ki_angular_, Kd_angular_;
  double position_tolerance_;
  double control_loop_rate_;

  // PID Errors
  double prev_error_linear_, prev_error_angular_;
  double integral_error_linear_, integral_error_angular_;

  // Callbacks and Methods
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Navigation::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigation> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleNavigation> goal_handle);

  void execute(const std::shared_ptr<GoalHandleNavigation> goal_handle);

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  void controlLoop(const std::shared_ptr<GoalHandleNavigation> goal_handle);
};

#endif  // MISSION_MANAGER__NAVIGATION_SERVER_HPP_
