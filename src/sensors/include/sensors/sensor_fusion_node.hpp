#ifndef SENSOR_FUSION_NODE_HPP
#define SENSOR_FUSION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class SensorFusionNode : public rclcpp::Node
{
public:
    SensorFusionNode();

private:
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void publishOdometry();
    void predict();
    void update();
    void latLonToENU(double latitude, double longitude, double altitude, double& x, double& y, double& z);

    // Member variables
    double reference_latitude_;
    double reference_longitude_;
    double reference_altitude_;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;

    Eigen::VectorXd state_; // State vector [x, y, z, roll, pitch, yaw]
    Eigen::MatrixXd covariance_; // Covariance matrix

    sensor_msgs::msg::NavSatFix::SharedPtr last_gps_data_; // Latest GPS data
    sensor_msgs::msg::Imu::SharedPtr last_imu_data_; // Latest IMU data
};

#endif // SENSOR_FUSION_NODE_HPP
