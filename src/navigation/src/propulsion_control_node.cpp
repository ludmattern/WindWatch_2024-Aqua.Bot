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
    this->declare_parameter<double>("max_steering_angle", 0.78);   // Angle maximal en radians (45 degrés)
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
    double linear_velocity = msg->linear.x;    // Vitesse linéaire
    double angular_velocity = msg->angular.z;  // Vitesse angulaire

    const double epsilon = 0.1;

    double thrust_left, thrust_right;
    double steering_angle_left, steering_angle_right;

    // Variables persistantes pour le lissage
    static double prev_steering_angle_left = 0.0;
    static double prev_steering_angle_right = 0.0;
    static double prev_thrust_left = 0.0;
    static double prev_thrust_right = 0.0;

    // Temps écoulé depuis la dernière mise à jour (à calculer ou à définir)
    double delta_time = 0.1; // Par exemple, 100 ms ; ajustez selon votre contexte

    // Facteurs ajustables
    double steering_gain_factor = 0.5;           // Réduction du gain de braquage
    double steering_angle_limit_factor = 0.7;    // Limitation de l'angle de braquage
    double smoothing_factor = 0.1;               // Facteur de lissage
    double max_steering_rate = 1.0;              // Vitesse maximale de changement d'angle (rad/s)
    double max_thrust_rate = 1000;                // Vitesse maximale de changement de poussée
    double steering_gain_adjusted = steering_gain_ * steering_gain_factor;

    if (std::abs(angular_velocity) > 0.0 && std::abs(linear_velocity) < epsilon)
    {
        // Rotation sur place
        double rotation_thrust = std::abs(angular_velocity) * rotation_gain_;

        // Appliquer une poussée différentielle en fonction du signe d’angular_velocity
        if (angular_velocity > 0) {
            thrust_left = rotation_thrust;
            thrust_right = -rotation_thrust;
        } else {
            thrust_left = -rotation_thrust;
            thrust_right = rotation_thrust;
        }

        // Limiter la poussée
        thrust_left = std::clamp(thrust_left, -max_thrust_, max_thrust_);
        thrust_right = std::clamp(thrust_right, -max_thrust_, max_thrust_);

        // Angles de braquage à +/- max_steering_angle_
        steering_angle_left = max_steering_angle_ * (angular_velocity >= 0 ? 1 : -1);
        steering_angle_right = max_steering_angle_ * (angular_velocity >= 0 ? 1 : -1);
    }
    else
    {
        // Calcul des angles de braquage avec signe opposé
        steering_angle_left = steering_gain_adjusted * angular_velocity;
        steering_angle_right = -steering_gain_adjusted * angular_velocity;

        // Limiter les angles de braquage
        double max_safe_steering_angle = max_steering_angle_ * steering_angle_limit_factor;
        steering_angle_left = std::clamp(steering_angle_left, -max_safe_steering_angle, max_safe_steering_angle);
        steering_angle_right = std::clamp(steering_angle_right, -max_safe_steering_angle, max_safe_steering_angle);

        // Lissage des angles de braquage
        steering_angle_left = prev_steering_angle_left + smoothing_factor * (steering_angle_left - prev_steering_angle_left);
        steering_angle_right = prev_steering_angle_right + smoothing_factor * (steering_angle_right - prev_steering_angle_right);

        // Limitation du taux de changement des angles de braquage
        double max_steering_change = max_steering_rate * delta_time;
        double steering_change_left = steering_angle_left - prev_steering_angle_left;
        steering_change_left = std::clamp(steering_change_left, -max_steering_change, max_steering_change);
        steering_angle_left = prev_steering_angle_left + steering_change_left;

        double steering_change_right = steering_angle_right - prev_steering_angle_right;
        steering_change_right = std::clamp(steering_change_right, -max_steering_change, max_steering_change);
        steering_angle_right = prev_steering_angle_right + steering_change_right;

        // Mise à jour des angles de braquage précédents
        prev_steering_angle_left = steering_angle_left;
        prev_steering_angle_right = steering_angle_right;

        // Calcul de la poussée
        double base_thrust = linear_velocity * scale_factor_;
        thrust_left = base_thrust;
        thrust_right = base_thrust;

        // Calcul d'un facteur pour réduire la composante angulaire en fonction de l'effet de braquage
        double steering_effect = std::max(std::abs(steering_angle_left), std::abs(steering_angle_right)) / max_steering_angle_;
        double angular_component = ((distance_between_thrusters_ / 2.0) * angular_velocity * scale_factor_) * (1.0 - steering_effect);

        thrust_left -= angular_component;
        thrust_right += angular_component;

        // Limitation du taux de changement de la poussée
        double max_thrust_change = max_thrust_rate * delta_time;
        double thrust_change_left = thrust_left - prev_thrust_left;
        thrust_change_left = std::clamp(thrust_change_left, -max_thrust_change, max_thrust_change);
        thrust_left = prev_thrust_left + thrust_change_left;

        double thrust_change_right = thrust_right - prev_thrust_right;
        thrust_change_right = std::clamp(thrust_change_right, -max_thrust_change, max_thrust_change);
        thrust_right = prev_thrust_right + thrust_change_right;

        // Mise à jour des poussées précédentes
        prev_thrust_left = thrust_left;
        prev_thrust_right = thrust_right;

        // Limitation des poussées
        thrust_left = std::clamp(thrust_left, -max_thrust_, max_thrust_);
        thrust_right = std::clamp(thrust_right, -max_thrust_, max_thrust_);
    }

    // Préparation et publication des commandes pour les propulseurs
    std_msgs::msg::Float64 left_pos_msg;
    left_pos_msg.data = steering_angle_left;
    left_thruster_pos_pub_->publish(left_pos_msg);

    std_msgs::msg::Float64 right_pos_msg;
    right_pos_msg.data = steering_angle_right;
    right_thruster_pos_pub_->publish(right_pos_msg);

    std_msgs::msg::Float64 left_thrust_msg;
    left_thrust_msg.data = thrust_left;
    left_thruster_thrust_pub_->publish(left_thrust_msg);

    std_msgs::msg::Float64 right_thrust_msg;
    right_thrust_msg.data = thrust_right;
    right_thruster_thrust_pub_->publish(right_thrust_msg);

    RCLCPP_INFO(this->get_logger(), "Published Thruster Commands: "
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
