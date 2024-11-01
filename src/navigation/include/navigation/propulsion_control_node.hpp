#ifndef PROPULSION_CONTROL_NODE_HPP
#define PROPULSION_CONTROL_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class PropulsionControlNode : public rclcpp::Node
{
public:
    PropulsionControlNode();

private:
    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // Publishers for thruster commands
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_thruster_pos_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_thruster_pos_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_thruster_thrust_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_thruster_thrust_pub_;

    // Subscription to propulsion command
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscription_;

    // Parameters
    double distance_between_thrusters_; // Distance between thrusters (meters)
    double steering_gain_;             // Gain for steering angle calculation
    double max_thrust_;                // Maximum thrust (Newtons)
    double max_steering_angle_;        // Maximum steering angle (radians)

    // Scaling factor for linear velocity to thruster speed
    double scale_factor_;
    double min_linear_speed_;
    double rotation_gain_;

	void computeRotationOnSpot(double angular_velocity, double& thrust_left, double& thrust_right, double& steering_angle_left, double& steering_angle_right);
	void computeMovingRotation(double linear_velocity, double angular_velocity, double& thrust_left, double& thrust_right, double& steering_angle_left, double& steering_angle_right);
	void publishCommands(double thrust_left, double thrust_right, double steering_angle_left, double steering_angle_right);
};

#endif // PROPULSION_CONTROL_NODE_HPP
