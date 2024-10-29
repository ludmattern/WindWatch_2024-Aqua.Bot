// src/navigation_server.cpp

#include "mission_manager/navigation_server.hpp"
#include "mission_manager/trajectory_manager.hpp"
#include "mission_manager/geometry_utils.hpp"
#include <thread>

namespace mission_manager {

NavigationServer::NavigationServer()
: Node("navigation_server"),
  odom_received_(false),
  initial_distance_to_goal_(0.0),
  estimated_disturbance_angular_(0.0),
  last_waypoint_index_(std::numeric_limits<size_t>::max()),
  starting_point_{0.0, 0.0, false},
  current_linear_speed_(0.0),
  Kp_linear_(0.5),
  Ki_linear_(0.0),
  Kd_linear_(0.0),
  Kp_angular_(0.4),
  Ki_angular_(0.0),
  Kd_angular_(0.0),
  Kd_disturbance_(0.0),
  position_tolerance_(5.0),
  control_loop_rate_(20.0),
  min_linear_speed_(0.0),
  max_linear_speed_(6.0),
  max_angular_speed_(0.5),
  max_acceleration_(1.0),
  max_deceleration_(1.0),
  linear_pid_(Kp_linear_, Ki_linear_, Kd_linear_),
  angular_pid_(Kp_angular_, Ki_angular_, Kd_angular_),
  current_state_(NavigationState::IDLE)
{
    // Initialize the action server
    action_server_ = rclcpp_action::create_server<Navigation>(
        this,
        "navigation",
        std::bind(&NavigationServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&NavigationServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&NavigationServer::handle_accepted, this, std::placeholders::_1)
    );

    // Publisher for propulsion commands
    cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/propulsion/command", 10);

    // Subscriber for odometry
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/mission/odometry", 10,
        std::bind(&NavigationServer::odomCallback, this, std::placeholders::_1)
    );

    // Declare parameters
    this->declare_parameter<double>("Kp_linear", Kp_linear_);
    this->declare_parameter<double>("Ki_linear", Ki_linear_);
    this->declare_parameter<double>("Kd_linear", Kd_linear_);
    this->declare_parameter<double>("Kp_angular", Kp_angular_);
    this->declare_parameter<double>("Ki_angular", Ki_angular_);
    this->declare_parameter<double>("Kd_angular", Kd_angular_);
    this->declare_parameter<double>("Kd_disturbance", Kd_disturbance_);
    this->declare_parameter<double>("position_tolerance", position_tolerance_);
    this->declare_parameter<double>("control_loop_rate", control_loop_rate_);
    this->declare_parameter<double>("min_linear_speed", min_linear_speed_);
    this->declare_parameter<double>("max_linear_speed", max_linear_speed_);
    this->declare_parameter<double>("max_angular_speed", max_angular_speed_);
    this->declare_parameter<double>("max_acceleration", max_acceleration_);
    this->declare_parameter<double>("max_deceleration", max_deceleration_);

    // Get parameters
    this->get_parameter("Kp_linear", Kp_linear_);
    this->get_parameter("Ki_linear", Ki_linear_);
    this->get_parameter("Kd_linear", Kd_linear_);
    this->get_parameter("Kp_angular", Kp_angular_);
    this->get_parameter("Ki_angular", Ki_angular_);
    this->get_parameter("Kd_angular", Kd_angular_);
    this->get_parameter("Kd_disturbance", Kd_disturbance_);
    this->get_parameter("position_tolerance", position_tolerance_);
    this->get_parameter("control_loop_rate", control_loop_rate_);
    this->get_parameter("min_linear_speed", min_linear_speed_);
    this->get_parameter("max_linear_speed", max_linear_speed_);
    this->get_parameter("max_angular_speed", max_angular_speed_);
    this->get_parameter("max_acceleration", max_acceleration_);
    this->get_parameter("max_deceleration", max_deceleration_);

    // Update PID controllers with retrieved parameters
    linear_pid_.set_parameters(Kp_linear_, Ki_linear_, Kd_linear_);
    angular_pid_.set_parameters(Kp_angular_, Ki_angular_, Kd_angular_);

    RCLCPP_INFO(this->get_logger(), "Navigation Server démarré.");
}

rclcpp_action::GoalResponse NavigationServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Navigation::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Requête d'objectif de navigation reçue.");

    if (goal->path.poses.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Chemin reçu vide.");
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavigationServer::handle_cancel(
    const std::shared_ptr<GoalHandleNavigation> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Requête d'annulation de l'objectif de navigation reçue.");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void NavigationServer::handle_accepted(const std::shared_ptr<GoalHandleNavigation> goal_handle)
{
    // Démarrer un nouveau thread pour exécuter l'objectif
    std::thread([this, goal_handle]() {
        execute(goal_handle);
    }).detach();
}

void NavigationServer::execute(const std::shared_ptr<GoalHandleNavigation> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Exécution de l'objectif de navigation...");

    initializeExecution(goal_handle);

    auto feedback = std::make_shared<Navigation::Feedback>();
    auto result = std::make_shared<Navigation::Result>();

    TrajectoryManager traj_manager;

    rclcpp::Rate rate(control_loop_rate_);
    current_state_ = NavigationState::EXECUTING;

    while (rclcpp::ok())
    {
        if (isGoalCancelled(goal_handle, result))
        {
            RCLCPP_INFO(this->get_logger(), "Objectif de navigation annulé.");
            current_state_ = NavigationState::CANCELLED;
            return;
        }

        if (!isOdometryReceived())
        {
            RCLCPP_WARN(this->get_logger(), "En attente des données d'odométrie...");
            rate.sleep();
            continue;
        }

        controlLoop(goal_handle, traj_manager);

        if (isGoalReached(result, goal_handle))
        {
            current_state_ = NavigationState::SUCCEEDED;
            return;
        }

        // Mettre à jour le feedback
        feedback->progress = static_cast<float>(current_waypoint_index_) / path_.size() * 100.0f;
        goal_handle->publish_feedback(feedback);

        rate.sleep();
    }
}

void NavigationServer::controlLoop(const std::shared_ptr<GoalHandleNavigation> goal_handle, TrajectoryManager &traj_manager)
{
    if (current_waypoint_index_ >= path_.size())
        return;

    auto current_waypoint = path_[current_waypoint_index_].pose;

    double x_current = 0.0;
    double y_current = 0.0;
    double yaw_current = 0.0;

    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        x_current = current_odometry_.pose.pose.position.x;
        y_current = current_odometry_.pose.pose.position.y;

        tf2::Quaternion q_current(
            current_odometry_.pose.pose.orientation.x,
            current_odometry_.pose.pose.orientation.y,
            current_odometry_.pose.pose.orientation.z,
            current_odometry_.pose.pose.orientation.w
        );
        double roll_current, pitch_current;
        tf2::Matrix3x3(q_current).getRPY(roll_current, pitch_current, yaw_current);
    }

    Point waypoint_position = {current_waypoint.position.x, current_waypoint.position.y};

    if (current_waypoint_index_ != 0)
    {
        const auto& previous_pose = path_[current_waypoint_index_ - 1].pose;
        starting_point_ = {previous_pose.position.x, previous_pose.position.y, true};
    }
    else if (current_waypoint_index_ == 0 && !starting_point_.initialized)
    {
        starting_point_ = {x_current, y_current, true};
    }

    Point start_position = {starting_point_.x, starting_point_.y};
    Point current_position = {x_current, y_current};

    Point projection = traj_manager.calculatePerpendicularProjection(waypoint_position, start_position, current_position);
    double deviation = traj_manager.calculateTrajectoryDeviation(projection, current_position);
    Point correctedEndpoint = traj_manager.calculateCorrectedEndpoint(projection, waypoint_position, start_position, current_position);

    RCLCPP_DEBUG(this->get_logger(), "Index du waypoint: %zu", current_waypoint_index_);
    RCLCPP_DEBUG(this->get_logger(), "Waypoint: (%.2f, %.2f)", waypoint_position.x, waypoint_position.y);
    RCLCPP_DEBUG(this->get_logger(), "Point de départ: (%.2f, %.2f)", start_position.x, start_position.y);
    RCLCPP_DEBUG(this->get_logger(), "Position actuelle: (%.2f, %.2f)", current_position.x, current_position.y);
    RCLCPP_DEBUG(this->get_logger(), "Endpoint corrigé: (%.2f, %.2f)", correctedEndpoint.x, correctedEndpoint.y);

    double x_goal = correctedEndpoint.x;
    double y_goal = correctedEndpoint.y;

    double error_x = x_goal - x_current;
    double error_y = y_goal - y_current;

    double real_error_x = current_waypoint.position.x - x_current;
    double real_error_y = current_waypoint.position.y - y_current;
    double distance_to_goal = std::sqrt(real_error_x * real_error_x + real_error_y * real_error_y);

    if (distance_to_goal < position_tolerance_)
    {
        RCLCPP_INFO(this->get_logger(), "Waypoint %zu atteint.", current_waypoint_index_ + 1);
        current_waypoint_index_++;
        initial_distance_to_goal_ = 0.0;
        return;
    }

    if (last_waypoint_index_ != current_waypoint_index_)
    {
        initial_distance_to_goal_ = distance_to_goal;
        last_waypoint_index_ = current_waypoint_index_;

        linear_pid_.reset();
        angular_pid_.reset();
    }

    double distance_traveled = initial_distance_to_goal_ - distance_to_goal;

    double theta_goal = std::atan2(error_y, error_x);
    double error_theta = theta_goal - yaw_current;
    error_theta = std::atan2(std::sin(error_theta), std::cos(error_theta));

    double error_linear = distance_to_goal;

    double dt = 1.0 / control_loop_rate_;

    double linear_speed_pid = linear_pid_.compute(0.0, -error_linear, dt);
    double angular_speed_pid = angular_pid_.compute(0.0, -error_theta, dt);

    double target_speed = max_linear_speed_;

    if (distance_traveled <= TrajectoryManager::ACCEL_DISTANCE_METERS)
    {
        double scaling_factor = std::abs(distance_traveled) / TrajectoryManager::ACCEL_DISTANCE_METERS;
        scaling_factor = std::clamp(scaling_factor, 0.0, 1.0);
        target_speed = scaling_factor * max_linear_speed_;
    }

    if (distance_to_goal <= TrajectoryManager::DECEL_DISTANCE_METERS)
    {
        double decel_factor = distance_to_goal / TrajectoryManager::DECEL_DISTANCE_METERS;
        decel_factor = std::clamp(decel_factor, 0.0, 1.0);
        target_speed = std::min(target_speed, decel_factor * max_linear_speed_);
    }

    target_speed = std::clamp(target_speed, min_linear_speed_, max_linear_speed_);

    double delta_speed = target_speed - current_linear_speed_;

    if (delta_speed > 0)
        delta_speed = std::min(delta_speed, max_acceleration_ * dt);
    else
        delta_speed = std::max(delta_speed, -max_deceleration_ * dt);

    current_linear_speed_ += delta_speed;

    double linear_speed = current_linear_speed_;

    if (std::abs(error_theta) > TrajectoryManager::ANGULAR_ERROR_THRESHOLD_RAD)
    {
        double scaling_factor = TrajectoryManager::ANGULAR_ERROR_THRESHOLD_RAD / std::abs(error_theta);
        scaling_factor = std::clamp(scaling_factor, 0.0, 1.0);
        linear_speed *= scaling_factor;
    }

    linear_speed = std::clamp(linear_speed, min_linear_speed_, max_linear_speed_);
    double angular_speed = std::clamp(angular_speed_pid, -max_angular_speed_, max_angular_speed_);

    auto cmd_msg = geometry_msgs::msg::Twist();
    cmd_msg.linear.x = linear_speed;
    cmd_msg.angular.z = angular_speed;

    cmd_publisher_->publish(cmd_msg);

    RCLCPP_DEBUG(this->get_logger(), "distance_to_goal: %.2f", distance_to_goal);
    RCLCPP_DEBUG(this->get_logger(), "Control Outputs: linear_speed=%.2f m/s, angular_speed=%.2f rad/s",
                linear_speed, angular_speed);
}

bool NavigationServer::isGoalCancelled(const std::shared_ptr<GoalHandleNavigation> goal_handle, 
                                       std::shared_ptr<Navigation::Result> &result)
{
    if (goal_handle->is_canceling())
    {
        goal_cancelled_ = true;
        result->success = false;
        goal_handle->canceled(result);
        return true;
    }
    return false;
}

bool NavigationServer::isOdometryReceived()
{
    std::lock_guard<std::mutex> lock(odom_mutex_);
    return odom_received_;
}

bool NavigationServer::isGoalReached(std::shared_ptr<Navigation::Result> &result, const std::shared_ptr<GoalHandleNavigation> goal_handle)
{
    if (current_waypoint_index_ >= path_.size())
    {
        stopRobot();
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Navigation goal succeeded.");
        return true;
    }
    return false;
}

void NavigationServer::stopRobot()
{
    auto cmd_msg = geometry_msgs::msg::Twist();
    cmd_publisher_->publish(cmd_msg);
}

void NavigationServer::updateFeedback(std::shared_ptr<Navigation::Feedback> feedback)
{
    feedback->progress = static_cast<float>(current_waypoint_index_) / path_.size() * 100.0f;
}

void NavigationServer::initializeExecution(const std::shared_ptr<GoalHandleNavigation> goal_handle)
{
    auto goal = goal_handle->get_goal();
    path_ = goal->path.poses;
    current_waypoint_index_ = 0;
    starting_point_ = {0.0, 0.0, false};
    goal_cancelled_ = false;
}

void NavigationServer::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(odom_mutex_);
    current_odometry_ = *msg;
    odom_received_ = true;
    RCLCPP_DEBUG(this->get_logger(), "Odometry mise à jour: x=%.2f, y=%.2f", msg->pose.pose.position.x, msg->pose.pose.position.y);
}

} // namespace mission_manager

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mission_manager::NavigationServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
