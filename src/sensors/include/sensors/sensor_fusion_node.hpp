#ifndef SENSOR_FUSION_NODE_HPP
#define SENSOR_FUSION_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <Eigen/Dense>

class SensorFusionNode : public rclcpp::Node
{
public:
    SensorFusionNode();

private:
    // Callbacks
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void publishOdometry();
    void predict(double dt);
    void update();

    void latLonToENU(double latitude, double longitude, double altitude, double &x, double &y, double &z);

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;

    // Publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;

    // Timer for odometry publication
    rclcpp::TimerBase::SharedPtr odometry_timer_;

    // State vector and covariance matrix
    Eigen::VectorXd state_;       // [x, y, z, vx, vy, vz, roll, pitch, yaw]
    Eigen::MatrixXd covariance_;

    // Process noise and measurement noise covariance matrices
    Eigen::MatrixXd process_noise_;
    Eigen::MatrixXd measurement_noise_;

    // Reference coordinates for ENU conversion
    double reference_latitude_;
    double reference_longitude_;
    double reference_altitude_;

    // Latest sensor data
    sensor_msgs::msg::NavSatFix::SharedPtr last_gps_data_;
    sensor_msgs::msg::Imu::SharedPtr last_imu_data_;

    // Timestamps for time management
    rclcpp::Time last_gps_time_;
    rclcpp::Time last_imu_time_;

    // Flags to check if initial data has been received
    bool initial_gps_received_;
    bool initial_imu_received_;
};

#endif // SENSOR_FUSION_NODE_HPP
