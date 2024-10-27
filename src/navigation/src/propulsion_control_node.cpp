// propulsion_control_node.cpp

#include "navigation/propulsion_control_node.hpp"
#include <algorithm>
#include <cmath>

PropulsionControlNode::PropulsionControlNode()
: Node("propulsion_control_node")
{
    // Déclaration et récupération des paramètres
    this->declare_parameter<double>("distance_between_thrusters", 1.0); // Distance L en mètres
    this->declare_parameter<double>("steering_gain", 1.0);             // Gain pour le calcul des angles de braquage
    this->declare_parameter<double>("max_thrust", 5000.0);             // Vitesse maximale de l'hélice
    this->declare_parameter<double>("max_steering_angle", 0.785398);   // Angle maximal en radians (45 degrés)
    this->declare_parameter<double>("scale_factor", 810.0);            // Facteur d'échelle pour la vitesse linéaire
    this->declare_parameter<double>("min_linear_speed", 0.0);          // Vitesse linéaire minimale (m/s)
    this->declare_parameter<double>("rotation_gain", 1000.0);          // Gain pour les rotations sur place

    this->get_parameter("distance_between_thrusters", distance_between_thrusters_);
    this->get_parameter("steering_gain", steering_gain_);
    this->get_parameter("max_thrust", max_thrust_);
    this->get_parameter("max_steering_angle", max_steering_angle_);
    this->get_parameter("scale_factor", scale_factor_);
    this->get_parameter("min_linear_speed", min_linear_speed_);
    this->get_parameter("rotation_gain", rotation_gain_);

    // Souscription au topic /propulsion/command
    cmd_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/propulsion/command", 10,
        std::bind(&PropulsionControlNode::cmdCallback, this, std::placeholders::_1));

    // Publications des commandes de poussée et de braquage
    left_thruster_pos_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/aquabot/thrusters/left/pos", 10);
    right_thruster_pos_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/aquabot/thrusters/right/pos", 10);
    left_thruster_thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/aquabot/thrusters/left/thrust", 10);
    right_thruster_thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/aquabot/thrusters/right/thrust", 10);

    RCLCPP_INFO(this->get_logger(), "Propulsion Control Node has started");
}

void PropulsionControlNode::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // Extraction des commandes de vitesse linéaire et angulaire
    double linear_velocity = msg->linear.x;    // Vitesse linéaire longitudinale (m/s)
    double angular_velocity = msg->angular.z;  // Vitesse angulaire autour de l'axe z (rad/s)

    // Définir un epsilon pour détecter la vitesse linéaire proche de zéro
    const double epsilon = 0.1; // Ajustez cette valeur si nécessaire

    double thrust_left, thrust_right;
    double steering_angle_left, steering_angle_right;

    if (std::abs(angular_velocity) > 0.0 && std::abs(linear_velocity) < epsilon)
    {
        // Rotation sur place avec une petite vitesse linéaire minimale pour stabiliser
        double rotation_thrust = angular_velocity * rotation_gain_; // `rotation_gain_` à définir expérimentalement

        // Limiter la poussée
        rotation_thrust = std::clamp(rotation_thrust, -max_thrust_, max_thrust_);

        // Angles de braquage à +/- max_steering_angle_
        steering_angle_left = steering_gain_ * angular_velocity;
        steering_angle_right = -steering_gain_ * angular_velocity;

        // Limitation des angles de braquage
        steering_angle_left = std::clamp(steering_angle_left, -max_steering_angle_, max_steering_angle_);
        steering_angle_right = std::clamp(steering_angle_right, -max_steering_angle_, max_steering_angle_);

        // Poussées opposées pour créer un couple de rotation
        thrust_left = rotation_thrust;
        thrust_right = rotation_thrust;
    }
    else
    {
        // Calculs normaux
        // Calcul des poussées pour chaque propulseur avec échelle
        thrust_left = (linear_velocity * scale_factor_) - (distance_between_thrusters_ / 2.0) * angular_velocity * scale_factor_;
        thrust_right = (linear_velocity * scale_factor_) + (distance_between_thrusters_ / 2.0) * angular_velocity * scale_factor_;

        // Limitation des poussées
        thrust_left = std::clamp(thrust_left, -max_thrust_, max_thrust_);
        thrust_right = std::clamp(thrust_right, -max_thrust_, max_thrust_);

        // Calcul des angles de braquage
        steering_angle_left = steering_gain_ * angular_velocity;
        steering_angle_right = -steering_gain_ * angular_velocity;

        // Limitation des angles de braquage
        steering_angle_left = std::clamp(steering_angle_left, -max_steering_angle_, max_steering_angle_);
        steering_angle_right = std::clamp(steering_angle_right, -max_steering_angle_, max_steering_angle_);

        // Assurer une vitesse linéaire minimale si nécessaire
        if (std::abs(linear_velocity) > epsilon)
        {
            double min_thrust = min_linear_speed_ * scale_factor_;
            thrust_left = std::max(std::abs(thrust_left), min_thrust) * (thrust_left >= 0 ? 1.0 : -1.0);
            thrust_right = std::max(std::abs(thrust_right), min_thrust) * (thrust_right >= 0 ? 1.0 : -1.0);
        }
    }

    // Préparation des messages pour les thrusters
    std_msgs::msg::Float64 left_pos_msg;
    left_pos_msg.data = steering_angle_left;

    std_msgs::msg::Float64 right_pos_msg;
    right_pos_msg.data = steering_angle_right;

    std_msgs::msg::Float64 left_thrust_msg;
    left_thrust_msg.data = thrust_left;

    std_msgs::msg::Float64 right_thrust_msg;
    right_thrust_msg.data = thrust_right;

    // Publication des commandes
    left_thruster_pos_pub_->publish(left_pos_msg);
    right_thruster_pos_pub_->publish(right_pos_msg);
    left_thruster_thrust_pub_->publish(left_thrust_msg);
    right_thruster_thrust_pub_->publish(right_thrust_msg);

    // Log des commandes publiées pour le débogage
    RCLCPP_DEBUG(this->get_logger(), "Published Thruster Commands: "
                "Left Pos: %.3f rad, Right Pos: %.3f rad, "
                "Left Thrust: %.3f, Right Thrust: %.3f",
                steering_angle_left, steering_angle_right,
                thrust_left, thrust_right);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PropulsionControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
