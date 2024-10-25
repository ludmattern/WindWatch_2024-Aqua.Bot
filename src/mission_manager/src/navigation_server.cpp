// src/navigation_server.cpp

#include "mission_manager/navigation_server.hpp"

using namespace std::chrono_literals;

NavigationServer::NavigationServer() : Node("navigation_server"), odom_received_(false)
{
  // Initialisation du serveur d'action
  action_server_ = rclcpp_action::create_server<Navigation>(
    this,
    "navigation",
    std::bind(&NavigationServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&NavigationServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&NavigationServer::handle_accepted, this, std::placeholders::_1)
  );

  // Initialisation du publisher pour les commandes
  cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/propulsion/command", 10);

  // Souscription à l'odométrie
  odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/mission/odometry", 10,
    std::bind(&NavigationServer::odomCallback, this, std::placeholders::_1)
  );

  // Déclaration des paramètres PID et autres
  this->declare_parameter<double>("Kp_linear", 1.0);
  this->declare_parameter<double>("Ki_linear", 0.0);
  this->declare_parameter<double>("Kd_linear", 0.0);
  this->declare_parameter<double>("Kp_angular", 2.0);
  this->declare_parameter<double>("Ki_angular", 0.0);
  this->declare_parameter<double>("Kd_angular", 0.0);
  this->declare_parameter<double>("position_tolerance", 0.5); // mètres
  this->declare_parameter<double>("control_loop_rate", 10.0); // Hz

  // Récupération des paramètres
  this->get_parameter("Kp_linear", Kp_linear_);
  this->get_parameter("Ki_linear", Ki_linear_);
  this->get_parameter("Kd_linear", Kd_linear_);
  this->get_parameter("Kp_angular", Kp_angular_);
  this->get_parameter("Ki_angular", Ki_angular_);
  this->get_parameter("Kd_angular", Kd_angular_);
  this->get_parameter("position_tolerance", position_tolerance_);
  this->get_parameter("control_loop_rate", control_loop_rate_);

  // Initialisation des erreurs PID
  prev_error_linear_ = 0.0;
  prev_error_angular_ = 0.0;
  integral_error_linear_ = 0.0;
  integral_error_angular_ = 0.0;

  RCLCPP_INFO(this->get_logger(), "Navigation Server has been started.");
}

void NavigationServer::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_odometry_ = *msg;
  odom_received_ = true;
}

rclcpp_action::GoalResponse NavigationServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const Navigation::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received Navigation goal request.");

  // Vérifier que le chemin n'est pas vide
  if (goal->path.poses.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Received empty path.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavigationServer::handle_cancel(
  const std::shared_ptr<GoalHandleNavigation> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel Navigation goal.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void NavigationServer::handle_accepted(const std::shared_ptr<GoalHandleNavigation> goal_handle)
{
  // Créer un nouveau thread pour exécuter l'objectif
  std::thread(
    [this, goal_handle]() {
      execute(goal_handle);
    }
  ).detach();
}

void NavigationServer::execute(const std::shared_ptr<GoalHandleNavigation> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing Navigation goal...");

  // Obtenir l'itinéraire depuis le goal
  auto goal = goal_handle->get_goal();
  path_ = goal->path.poses;
  current_waypoint_index_ = 0;
  goal_cancelled_ = false;

  // Initialisation du feedback et du résultat
  auto feedback = std::make_shared<Navigation::Feedback>();
  auto result = std::make_shared<Navigation::Result>();

  // Démarrer la boucle de contrôle
  rclcpp::Rate rate(control_loop_rate_);
  while (rclcpp::ok())
  {
    if (goal_handle->is_canceling())
    {
      goal_cancelled_ = true;
      result->success = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Navigation goal canceled.");
      return;
    }

    if (!odom_received_)
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for odometry...");
      rate.sleep();
      continue;
    }

    controlLoop(goal_handle);

    if (current_waypoint_index_ >= path_.size())
    {
      // Arrêter le bateau en envoyant une commande nulle
      auto cmd_msg = geometry_msgs::msg::Twist();
      cmd_publisher_->publish(cmd_msg);

      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Navigation goal succeeded.");
      return;
    }

    // Mettre à jour le feedback
    feedback->progress = static_cast<float>(current_waypoint_index_) / path_.size() * 100.0f;
    goal_handle->publish_feedback(feedback);

    rate.sleep();
  }
}

void NavigationServer::controlLoop(const std::shared_ptr<GoalHandleNavigation> goal_handle)
{
  // Vérifier si le waypoint courant est valide
  if (current_waypoint_index_ >= path_.size())
  {
    return;
  }

  // Waypoint courant
  auto current_waypoint = path_[current_waypoint_index_].pose;

  // Extraction de la position actuelle
  double x_current = current_odometry_.pose.pose.position.x;
  double y_current = current_odometry_.pose.pose.position.y;

  // Extraction de l'orientation actuelle (yaw)
  tf2::Quaternion q_current(
    current_odometry_.pose.pose.orientation.x,
    current_odometry_.pose.pose.orientation.y,
    current_odometry_.pose.pose.orientation.z,
    current_odometry_.pose.pose.orientation.w
  );
  double roll_current, pitch_current, yaw_current;
  tf2::Matrix3x3(q_current).getRPY(roll_current, pitch_current, yaw_current);

  // Extraction du point de destination
  double x_goal = current_waypoint.position.x;
  double y_goal = current_waypoint.position.y;

  // Calcul de l'erreur de position
  double error_x = x_goal - x_current;
  double error_y = y_goal - y_current;
  double distance_to_goal = std::sqrt(error_x * error_x + error_y * error_y);

  // Vérification si le waypoint est atteint
  if (distance_to_goal < position_tolerance_)
  {
    RCLCPP_INFO(this->get_logger(), "Waypoint %zu reached.", current_waypoint_index_ + 1);
    current_waypoint_index_++;
    return;
  }

  // Calcul de l'angle vers le point de destination
  double theta_goal = std::atan2(error_y, error_x);

  // Calcul de l'erreur d'orientation
  double error_theta = theta_goal - yaw_current;
  // Normalisation entre -pi et pi
  error_theta = std::atan2(std::sin(error_theta), std::cos(error_theta));

  // Calcul des erreurs PID
  double error_linear = distance_to_goal;
  double error_angular = error_theta;

  // Intégrales
  integral_error_linear_ += error_linear * (1.0 / control_loop_rate_);
  integral_error_angular_ += error_angular * (1.0 / control_loop_rate_);

  // Dérivées
  double derivative_error_linear = (error_linear - prev_error_linear_) * control_loop_rate_;
  double derivative_error_angular = (error_angular - prev_error_angular_) * control_loop_rate_;

  // Calcul PID
  double linear_speed = Kp_linear_ * error_linear
                       + Ki_linear_ * integral_error_linear_
                       + Kd_linear_ * derivative_error_linear;

  double angular_speed = Kp_angular_ * error_angular
                        + Ki_angular_ * integral_error_angular_
                        + Kd_angular_ * derivative_error_angular;

  // Mise à jour des erreurs précédentes
  prev_error_linear_ = error_linear;
  prev_error_angular_ = error_angular;

  // Limitation des vitesses
  double max_linear_speed = 6.0;      // m/s (12 nœuds)
  double max_angular_speed = 0.785398; // rad/s (45 degrés/s)

  linear_speed = std::clamp(linear_speed, 0.0, max_linear_speed); // Pas de marche arrière
  angular_speed = std::clamp(angular_speed, -max_angular_speed, max_angular_speed);

  // Création du message de commande
  auto cmd_msg = geometry_msgs::msg::Twist();
  cmd_msg.linear.x = linear_speed;
  cmd_msg.angular.z = angular_speed;

  // Publication de la commande
  cmd_publisher_->publish(cmd_msg);

  RCLCPP_INFO(this->get_logger(), "Published Command -> Linear: %.2f m/s, Angular: %.2f rad/s",
              linear_speed, angular_speed);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigationServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
