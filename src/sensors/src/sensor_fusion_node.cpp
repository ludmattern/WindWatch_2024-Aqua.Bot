#include "sensors/sensor_fusion_node.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

SensorFusionNode::SensorFusionNode() : Node("sensor_fusion_node"),
                                       initial_gps_received_(false),
                                       initial_imu_received_(false)
{
    // Reference point for ENU conversion (latitude, longitude, altitude)
    reference_latitude_ = 48.046300;
    reference_longitude_ = -4.976320;
    reference_altitude_ = 0.0;

    // Initialize subscriptions
    gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/aquabot/sensors/gps/gps/fix", 10,
        std::bind(&SensorFusionNode::gpsCallback, this, std::placeholders::_1));

    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/aquabot/sensors/imu/imu/data", 10,
        std::bind(&SensorFusionNode::imuCallback, this, std::placeholders::_1));

    // Initialize publisher
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/mission/odometry", 10);

    // Initialize state and covariance
    state_ = Eigen::VectorXd::Zero(9); // [x, y, z, vx, vy, vz, roll, pitch, yaw]
    covariance_ = Eigen::MatrixXd::Identity(9, 9);

    // Initialize noise covariance matrices
    process_noise_ = Eigen::MatrixXd::Zero(9, 9);
    measurement_noise_ = Eigen::MatrixXd::Zero(6, 6); // Measurements: [x, y, z, roll, pitch, yaw]

    // Set process noise variances (squared standard deviations)
    process_noise_(0, 0) = 0.1;    // Position X noise variance
    process_noise_(1, 1) = 0.1;    // Position Y noise variance
    process_noise_(2, 2) = 0.1;    // Position Z noise variance
    process_noise_(3, 3) = 0.1;    // Velocity X noise variance
    process_noise_(4, 4) = 0.1;    // Velocity Y noise variance
    process_noise_(5, 5) = 0.1;    // Velocity Z noise variance
    process_noise_(6, 6) = pow(0.01 * M_PI / 180.0, 2); // Roll noise variance
    process_noise_(7, 7) = pow(0.01 * M_PI / 180.0, 2); // Pitch noise variance
    process_noise_(8, 8) = pow(0.01 * M_PI / 180.0, 2); // Yaw noise variance

    // Set measurement noise variances (squared standard deviations)
    measurement_noise_(0, 0) = pow(0.85, 2);                     // GPS horizontal position noise variance
    measurement_noise_(1, 1) = pow(0.85, 2);                     // GPS horizontal position noise variance
    measurement_noise_(2, 2) = pow(2.0, 2);                      // GPS vertical position noise variance
    measurement_noise_(3, 3) = pow(0.08 * M_PI / 180.0, 2);      // IMU roll noise variance
    measurement_noise_(4, 4) = pow(0.08 * M_PI / 180.0, 2);      // IMU pitch noise variance
    measurement_noise_(5, 5) = pow(0.8 * M_PI / 180.0, 2);       // IMU yaw noise variance

    // Timer to publish odometry at 10 Hz
    odometry_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),  // 10 Hz
        std::bind(&SensorFusionNode::publishOdometry, this));

    RCLCPP_INFO(this->get_logger(), "Sensor Fusion Node has started");
}

void SensorFusionNode::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "GPS callback triggered.");
    last_gps_data_ = msg;

    if (!initial_gps_received_)
    {
        last_gps_time_ = rclcpp::Time(msg->header.stamp);
        initial_gps_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Initial GPS data received.");
        return;
    }

    double dt = (rclcpp::Time(msg->header.stamp) - last_gps_time_).seconds();
    last_gps_time_ = rclcpp::Time(msg->header.stamp);

    if (dt <= 0.0)
    {
        RCLCPP_WARN(this->get_logger(), "Non-positive time difference in GPS callback.");
        return;
    }

    // Predict the state to the current time
    predict(dt);

    // Convert GPS data to ENU coordinates
    double gps_x, gps_y, gps_z;
    latLonToENU(msg->latitude, msg->longitude, msg->altitude, gps_x, gps_y, gps_z);

    // Prepare measurement vector z (positions)
    Eigen::VectorXd z(6);
    z << gps_x, gps_y, gps_z, state_(6), state_(7), state_(8); // Utilisation de l'orientation de l'état

    // Measurement matrix H
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, 9);
    H.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3); // Position
    H.block<3, 3>(3, 6) = Eigen::MatrixXd::Identity(3, 3); // Orientation

    // Innovation
    Eigen::VectorXd y = z - H * state_;

    // Innovation covariance
    Eigen::MatrixXd S = H * covariance_ * H.transpose() + measurement_noise_;

    // Kalman gain
    Eigen::MatrixXd K = covariance_ * H.transpose() * S.inverse();

    // Update state and covariance
    state_ = state_ + K * y;
    covariance_ = (Eigen::MatrixXd::Identity(9, 9) - K * H) * covariance_;

    RCLCPP_DEBUG(this->get_logger(), "GPS update step executed.");
}

void SensorFusionNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    last_imu_data_ = msg;

    if (!initial_imu_received_)
    {
        last_imu_time_ = rclcpp::Time(msg->header.stamp);
        initial_imu_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Initial IMU data received.");
        return;
    }

    double dt = (rclcpp::Time(msg->header.stamp) - last_imu_time_).seconds();
    last_imu_time_ = rclcpp::Time(msg->header.stamp);

    if (dt <= 0.0)
    {
        RCLCPP_WARN(this->get_logger(), "Non-positive time difference in IMU callback.");
        return;
    }

    // Predict the state to the current time
    predict(dt);

    // Extract angular velocities and linear accelerations
    double wx = msg->angular_velocity.x;
    double wy = msg->angular_velocity.y;
    double wz = msg->angular_velocity.z;

    double ax = msg->linear_acceleration.x;
    double ay = msg->linear_acceleration.y;
    double az = msg->linear_acceleration.z;

    // Update orientation state using angular velocities
    state_(6) += wx * dt;
    state_(7) += wy * dt;
    state_(8) += wz * dt;

    // Wrap angles between -pi and pi
    state_(6) = std::atan2(std::sin(state_(6)), std::cos(state_(6)));
    state_(7) = std::atan2(std::sin(state_(7)), std::cos(state_(7)));
    state_(8) = std::atan2(std::sin(state_(8)), std::cos(state_(8)));

    // Update velocities using accelerations (assuming flat earth and neglecting gravity)
    // For a more accurate model, transform accelerations to the world frame and subtract gravity

    state_(3) += ax * dt;
    state_(4) += ay * dt;
    state_(5) += az * dt;

    // No measurement update here, as IMU data is used in the prediction step
    RCLCPP_DEBUG(this->get_logger(), "IMU prediction step executed.");
}

void SensorFusionNode::predict(double dt)
{
    // State transition model
    // Position update: x = x + vx * dt
    state_(0) += state_(3) * dt;
    state_(1) += state_(4) * dt;
    state_(2) += state_(5) * dt;

    // The velocities and orientations are updated in the IMU callback

    // State transition matrix F
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(9, 9);
    F(0, 3) = dt;
    F(1, 4) = dt;
    F(2, 5) = dt;
    // Velocities updated with accelerations in IMU callback
    // Orientations updated with angular velocities in IMU callback

    // Process noise Q scaled by dt
    Eigen::MatrixXd Q = process_noise_ * dt;

    covariance_ = F * covariance_ * F.transpose() + Q;

    RCLCPP_DEBUG(this->get_logger(), "Predict step executed with dt = %f", dt);
}

void SensorFusionNode::publishOdometry()
{
    if (initial_gps_received_ && initial_imu_received_)
    {
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = this->get_clock()->now();
        odom_msg.header.frame_id = "odom";

        // Fill in the position and orientation data from the state vector
        odom_msg.pose.pose.position.x = state_(0);
        odom_msg.pose.pose.position.y = state_(1);
        odom_msg.pose.pose.position.z = state_(2);

        // Convert roll, pitch, yaw to quaternion for orientation
        tf2::Quaternion q;
        q.setRPY(state_(6), state_(7), state_(8));
        odom_msg.pose.pose.orientation = tf2::toMsg(q);

        // Set the velocity
        odom_msg.twist.twist.linear.x = state_(3);
        odom_msg.twist.twist.linear.y = state_(4);
        odom_msg.twist.twist.linear.z = state_(5);

        // Fill in covariance matrix
        for (int i = 0; i < 6; ++i)
        {
            odom_msg.pose.covariance[i * 6 + i] = covariance_(i, i);
        }

        // Publish the odometry message
        odometry_publisher_->publish(odom_msg);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for initial sensor data to publish odometry.");
    }
}

void SensorFusionNode::latLonToENU(double latitude, double longitude, double altitude, double& x, double& y, double& z)
{
    constexpr double a = 6378137.0;                // Semi-major axis of WGS84 ellipsoid in meters
    constexpr double f = 1 / 298.257223563;        // Flattening of WGS84 ellipsoid
    constexpr double e_sq = f * (2 - f);           // Square of eccentricity

    // Convert latitude/longitude to radians
    double lat_rad = latitude * M_PI / 180.0;
    double lon_rad = longitude * M_PI / 180.0;

    double ref_lat_rad = reference_latitude_ * M_PI / 180.0;
    double ref_lon_rad = reference_longitude_ * M_PI / 180.0;

    // Calculate prime vertical radius of curvature
    double N = a / sqrt(1 - e_sq * sin(lat_rad) * sin(lat_rad));

    // Convert current GPS to ECEF
    double ecef_x = (N + altitude) * cos(lat_rad) * cos(lon_rad);
    double ecef_y = (N + altitude) * cos(lat_rad) * sin(lon_rad);
    double ecef_z = (N * (1 - e_sq) + altitude) * sin(lat_rad);

    // Convert reference point to ECEF
    N = a / sqrt(1 - e_sq * sin(ref_lat_rad) * sin(ref_lat_rad));
    double ref_ecef_x = (N + reference_altitude_) * cos(ref_lat_rad) * cos(ref_lon_rad);
    double ref_ecef_y = (N + reference_altitude_) * cos(ref_lat_rad) * sin(ref_lon_rad);
    double ref_ecef_z = (N * (1 - e_sq) + reference_altitude_) * sin(ref_lat_rad);

    // Calculate delta ECEF
    double dx = ecef_x - ref_ecef_x;
    double dy = ecef_y - ref_ecef_y;
    double dz = ecef_z - ref_ecef_z;

    // ECEF to ENU conversion
    double sin_lat = sin(ref_lat_rad);
    double cos_lat = cos(ref_lat_rad);
    double sin_lon = sin(ref_lon_rad);
    double cos_lon = cos(ref_lon_rad);

    x = -sin_lon * dx + cos_lon * dy;
    y = -cos_lon * sin_lat * dx - sin_lat * sin_lon * dy + cos_lat * dz;
    z = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz;

    RCLCPP_DEBUG(this->get_logger(), "Converted lat/lon to ENU: (%f, %f, %f) -> (%f, %f, %f)", latitude, longitude, altitude, x, y, z);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
