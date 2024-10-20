// sensor_fusion_node.cpp

#include "sensors/sensor_fusion_node.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

SensorFusionNode::SensorFusionNode() : Node("sensor_fusion_node")
{
    // Reference point for ENU conversion (latitude, longitude, altitude)
    reference_latitude_ = 48.046300;
    reference_longitude_ = -4.976320;
    reference_altitude_ = 0.0;

    // Initialize subscriptions
    gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/aqua_bot/sensors/gps/fix", 10, 
        std::bind(&SensorFusionNode::gpsCallback, this, std::placeholders::_1));

    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/aqua_bot/sensors/imu/data", 10, 
        std::bind(&SensorFusionNode::imuCallback, this, std::placeholders::_1));

    // Initialize publisher
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/mission/odometry", 10);

    // Initialize state and covariance
    state_ = Eigen::VectorXd::Zero(6); // [x, y, z, roll, pitch, yaw]
    covariance_ = Eigen::MatrixXd::Identity(6, 6);

    RCLCPP_INFO(this->get_logger(), "Sensor Fusion Node has started");
}

void SensorFusionNode::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "GPS callback triggered.");
    last_gps_data_ = msg;

    double x, y, z;
    latLonToENU(msg->latitude, msg->longitude, msg->altitude, x, y, z);
    state_(0) = x; 
    state_(1) = y; 
    state_(2) = z;

    RCLCPP_INFO(this->get_logger(), "GPS data processed. ENU: x=%f, y=%f, z=%f", x, y, z);

    update();
    publishOdometry();
}


void SensorFusionNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    last_imu_data_ = msg;

    // Extract orientation (roll, pitch, yaw) from IMU
    tf2::Quaternion quat;
    tf2::fromMsg(msg->orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // Update orientation state
    state_(3) = roll;
    state_(4) = pitch;
    state_(5) = yaw;

    // Predict next state based on IMU data (typically angular velocities)
    predict();

    publishOdometry();
}

void SensorFusionNode::publishOdometry()
{
    if (last_gps_data_ && last_imu_data_)
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
        q.setRPY(state_(3), state_(4), state_(5));
        odom_msg.pose.pose.orientation = tf2::toMsg(q);

        // Fill in covariance matrix (for demonstration purposes, keep it constant)
        for (int i = 0; i < 6; ++i)
        {
            odom_msg.pose.covariance[i * 6 + i] = covariance_(i, i);
        }

        // Publish the odometry message
        odometry_publisher_->publish(odom_msg);
    }
}

void SensorFusionNode::predict()
{
    // TODO: Implement the prediction step for the EKF
    RCLCPP_DEBUG(this->get_logger(), "Predict step of EKF not implemented yet.");
}

void SensorFusionNode::update()
{
    // TODO: Implement the update step for the EKF
    RCLCPP_DEBUG(this->get_logger(), "Update step of EKF not implemented yet.");
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
    double N = a / sqrt(1 - e_sq * sin(ref_lat_rad) * sin(ref_lat_rad));

    // Convert reference point to ECEF
    double ref_x = (N + reference_altitude_) * cos(ref_lat_rad) * cos(ref_lon_rad);
    double ref_y = (N + reference_altitude_) * cos(ref_lat_rad) * sin(ref_lon_rad);
    double ref_z = (N * (1 - e_sq) + reference_altitude_) * sin(ref_lat_rad);

    // Convert current GPS to ECEF
    N = a / sqrt(1 - e_sq * sin(lat_rad) * sin(lat_rad));
    double ecef_x = (N + altitude) * cos(lat_rad) * cos(lon_rad);
    double ecef_y = (N + altitude) * cos(lat_rad) * sin(lon_rad);
    double ecef_z = (N * (1 - e_sq) + altitude) * sin(lat_rad);

    // Calculate ENU coordinates
    double dx = ecef_x - ref_x;
    double dy = ecef_y - ref_y;
    double dz = ecef_z - ref_z;

    x = -sin(ref_lon_rad) * dx + cos(ref_lon_rad) * dy;
    y = -sin(ref_lat_rad) * cos(ref_lon_rad) * dx - sin(ref_lat_rad) * sin(ref_lon_rad) * dy + cos(ref_lat_rad) * dz;
    z = cos(ref_lat_rad) * cos(ref_lon_rad) * dx + cos(ref_lat_rad) * sin(ref_lon_rad) * dy + sin(ref_lat_rad) * dz;

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
