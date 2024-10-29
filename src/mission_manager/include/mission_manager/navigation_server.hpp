// include/mission_manager/navigation_server.hpp

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
#include "mission_manager/geometry_utils.hpp"
#include "mission_manager/trajectory_manager.hpp"

namespace mission_manager {

/**
 * @brief Enum représentant l'état de la navigation.
 */
enum class NavigationState {
    IDLE,
    EXECUTING,
    CANCELLED,
    SUCCEEDED,
    FAILED
};

/**
 * @brief Classe représentant le serveur de navigation.
 * 
 * Cette classe gère les objectifs de navigation, contrôle la vitesse linéaire et angulaire 
 * en utilisant des PID, et publie les commandes de mouvement.
 */
class NavigationServer : public rclcpp::Node
{
public:
    using Navigation = mission_manager::action::Navigation;
    using GoalHandleNavigation = rclcpp_action::ServerGoalHandle<Navigation>;

    /**
     * @brief Constructeur de la classe NavigationServer.
     */
    NavigationServer();

private:
    // Action Server
    rclcpp_action::Server<Navigation>::SharedPtr action_server_;

    // Publishers and Subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

    // Odometry Data
    nav_msgs::msg::Odometry current_odometry_;
    std::mutex odom_mutex_;
    bool odom_received_;

    // Path and Waypoints
    std::vector<geometry_msgs::msg::PoseStamped> path_;
    size_t current_waypoint_index_;

    // Control Parameters
    double position_tolerance_;
    double control_loop_rate_;
    double initial_distance_to_goal_;
    double estimated_disturbance_angular_;
    size_t last_waypoint_index_;

    // PID Parameters
    double Kp_linear_;
    double Ki_linear_;
    double Kd_linear_;
    double Kp_angular_;
    double Ki_angular_;
    double Kd_angular_;
    double Kd_disturbance_;

    // Speed Limits
    double max_linear_speed_;
    double min_linear_speed_;
    double max_angular_speed_;
    double current_linear_speed_;
    double max_acceleration_;
    double max_deceleration_;

    // PID Controllers
    PIDController linear_pid_;
    PIDController angular_pid_;

    // Control Flags
    bool goal_cancelled_;

    // Starting Point
    Point starting_point_;

    // Current State
    NavigationState current_state_;

    // Action Server Callbacks
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const Navigation::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleNavigation> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleNavigation> goal_handle);

    // Execution and Control Loop
    void execute(const std::shared_ptr<GoalHandleNavigation> goal_handle);
    void controlLoop(const std::shared_ptr<GoalHandleNavigation> goal_handle, TrajectoryManager &traj_manager);

    // Odometry Callback
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Utility Functions
    bool isGoalCancelled(const std::shared_ptr<GoalHandleNavigation> goal_handle, 
                         std::shared_ptr<Navigation::Result> &result);
    bool isOdometryReceived();
    bool isGoalReached(std::shared_ptr<Navigation::Result> &result, const std::shared_ptr<GoalHandleNavigation> goal_handle);
    void stopRobot();
    void updateFeedback(std::shared_ptr<Navigation::Feedback> feedback);
    void initializeExecution(const std::shared_ptr<GoalHandleNavigation> goal_handle);
};

} // namespace mission_manager

#endif  // MISSION_MANAGER__NAVIGATION_SERVER_HPP_
