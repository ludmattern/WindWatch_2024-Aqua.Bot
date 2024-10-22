// sensor_fusion_node.cpp

#include "sensors/sensor_fusion_node.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

SensorFusionNode::SensorFusionNode() : Node("sensor_fusion_node"),
                                       initial_gps_received_(false),
                                       initial_imu_received_(false)
{
    // Point de référence pour la conversion ENU (latitude, longitude, altitude)
    reference_latitude_ = 48.046300;
    reference_longitude_ = -4.976320;
    reference_altitude_ = 1.20; // Altitude de référence en mètres
    geoid_height_ = 51.7976;     // Hauteur du géoïde à la position de référence

    // Initialisation des abonnements
    gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/aquabot/sensors/gps/gps/fix", 10,
        std::bind(&SensorFusionNode::gpsCallback, this, std::placeholders::_1));

    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/aquabot/sensors/imu/imu/data", 10,
        std::bind(&SensorFusionNode::imuCallback, this, std::placeholders::_1));

    // Initialisation du publisher
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/mission/odometry", 10);

    // Initialisation de l'état et de la covariance
    state_ = Eigen::VectorXd::Zero(9); // [x, y, z, vx, vy, vz, roll, pitch, yaw]
    covariance_ = Eigen::MatrixXd::Identity(9, 9);

    // Initialisation des matrices de bruit
    process_noise_ = Eigen::MatrixXd::Zero(9, 9);
    measurement_noise_ = Eigen::MatrixXd::Zero(6, 6); // Mesures : [x, y, z, roll, pitch, yaw]

    // Définition des variances de bruit de processus (écarts-types au carré)
    process_noise_(0, 0) = 0.1;    // Variance de position X
    process_noise_(1, 1) = 0.1;    // Variance de position Y
    process_noise_(2, 2) = 0.1;    // Variance de position Z
    process_noise_(3, 3) = 0.1;    // Variance de vitesse X
    process_noise_(4, 4) = 0.1;    // Variance de vitesse Y
    process_noise_(5, 5) = 0.1;    // Variance de vitesse Z
    process_noise_(6, 6) = pow(0.01 * M_PI / 180.0, 2); // Variance de roll
    process_noise_(7, 7) = pow(0.01 * M_PI / 180.0, 2); // Variance de pitch
    process_noise_(8, 8) = pow(0.01 * M_PI / 180.0, 2); // Variance de yaw

    // Définition des variances de bruit de mesure (écarts-types au carré)
    measurement_noise_(0, 0) = pow(0.85, 2);                     // Variance de position horizontale GPS
    measurement_noise_(1, 1) = pow(0.85, 2);                     // Variance de position horizontale GPS
    measurement_noise_(2, 2) = pow(2.0, 2);                      // Variance de position verticale GPS
    measurement_noise_(3, 3) = pow(0.08 * M_PI / 180.0, 2);      // Variance de roll IMU
    measurement_noise_(4, 4) = pow(0.08 * M_PI / 180.0, 2);      // Variance de pitch IMU
    measurement_noise_(5, 5) = pow(0.8 * M_PI / 180.0, 2);       // Variance de yaw IMU

    // Timer pour publier l'odométrie à 10 Hz
    odometry_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),  // 10 Hz
        std::bind(&SensorFusionNode::publishOdometry, this));

    RCLCPP_INFO(this->get_logger(), "Sensor Fusion Node has started");
}

void SensorFusionNode::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received GPS data - Latitude: %f, Longitude: %f, Altitude: %f",
                msg->latitude, msg->longitude, msg->altitude);

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

    // Prédire l'état jusqu'au temps actuel
    predict(dt);

    // Conversion des données GPS en coordonnées ENU
    double gps_x, gps_y, gps_z;
    latLonToENU(msg->latitude, msg->longitude, msg->altitude, gps_x, gps_y, gps_z);

    // Préparation du vecteur de mesure z (positions uniquement)
    Eigen::VectorXd z(3);
    z << gps_x, gps_y, gps_z;

    // Matrice de mesure H pour la position uniquement
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 9);
    H.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3); // Position

    // Innovation
    Eigen::VectorXd y = z - H * state_;

    // Covariance de l'innovation
    Eigen::MatrixXd S = H * covariance_ * H.transpose() + measurement_noise_.block<3, 3>(0, 0);

    // Gain de Kalman
    Eigen::MatrixXd K = covariance_ * H.transpose() * S.inverse();

    // Mise à jour de l'état et de la covariance
    state_ = state_ + K * y;
    covariance_ = (Eigen::MatrixXd::Identity(9, 9) - K * H) * covariance_;

    // Normalisation des angles
    state_(6) = std::atan2(std::sin(state_(6)), std::cos(state_(6)));
    state_(7) = std::atan2(std::sin(state_(7)), std::cos(state_(7)));
    state_(8) = std::atan2(std::sin(state_(8)), std::cos(state_(8)));
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

    // Prédire l'état jusqu'au temps actuel
    predict(dt);

    // Extraction des vitesses angulaires
    double wx = msg->angular_velocity.x;
    double wy = msg->angular_velocity.y;
    double wz = msg->angular_velocity.z;

    // Mise à jour de l'orientation en utilisant les vitesses angulaires
    state_(6) += wx * dt;
    state_(7) += wy * dt;
    state_(8) += wz * dt;

    // Normalisation des angles entre -pi et pi
    state_(6) = std::atan2(std::sin(state_(6)), std::cos(state_(6)));
    state_(7) = std::atan2(std::sin(state_(7)), std::cos(state_(7)));
    state_(8) = std::atan2(std::sin(state_(8)), std::cos(state_(8)));

    // Extraire l'orientation mesurée par l'IMU
    tf2::Quaternion q_imu;
    tf2::fromMsg(msg->orientation, q_imu);

    // Convertir le quaternion en angles d'Euler
    double roll_meas, pitch_meas, yaw_meas;
    tf2::Matrix3x3(q_imu).getRPY(roll_meas, pitch_meas, yaw_meas);

    // Préparation du vecteur de mesure pour l'orientation
    Eigen::VectorXd z_orientation(3);
    z_orientation << roll_meas, pitch_meas, yaw_meas;

    // Matrice de mesure H pour l'orientation
    Eigen::MatrixXd H_orientation = Eigen::MatrixXd::Zero(3, 9);
    H_orientation.block<3, 3>(0, 6) = Eigen::MatrixXd::Identity(3, 3); // Orientation

    // Innovation
    Eigen::VectorXd y_orientation = z_orientation - state_.segment<3>(6);

    // Gestion des sauts d'angle (par exemple, de π à -π)
    for (int i = 0; i < 3; ++i)
    {
        while (y_orientation(i) > M_PI)
            y_orientation(i) -= 2 * M_PI;
        while (y_orientation(i) < -M_PI)
            y_orientation(i) += 2 * M_PI;
    }

    // Covariance de l'innovation
    Eigen::MatrixXd S_orientation = H_orientation * covariance_ * H_orientation.transpose() + measurement_noise_.block<3, 3>(3, 3);

    // Gain de Kalman
    Eigen::MatrixXd K_orientation = covariance_ * H_orientation.transpose() * S_orientation.inverse();

    // Mise à jour de l'état et de la covariance
    state_ = state_ + K_orientation * y_orientation;
    covariance_ = (Eigen::MatrixXd::Identity(9, 9) - K_orientation * H_orientation) * covariance_;

    // Normalisation des angles
    state_(6) = std::atan2(std::sin(state_(6)), std::cos(state_(6)));
    state_(7) = std::atan2(std::sin(state_(7)), std::cos(state_(7)));
    state_(8) = std::atan2(std::sin(state_(8)), std::cos(state_(8)));

    // Extraction des accélérations linéaires
    double ax = msg->linear_acceleration.x;
    double ay = msg->linear_acceleration.y;
    double az = msg->linear_acceleration.z;

    // Création du quaternion à partir de l'orientation actuelle de l'état
    tf2::Quaternion q_state;
    q_state.setRPY(state_(6), state_(7), state_(8));

    // Création de la matrice de rotation du repère corps au repère navigation
    tf2::Matrix3x3 R_bn(q_state);

    // Vecteur des accélérations dans le repère corps
    tf2::Vector3 accel_body(ax, ay, az);

    // Transformation des accélérations dans le repère navigation
    tf2::Vector3 accel_nav = R_bn * accel_body;

    // Compensation de la gravité (g = 9.80665 m/s²)
    accel_nav.setZ(accel_nav.getZ() - 9.80665);

    // Mise à jour des vitesses dans le repère navigation
    state_(3) += accel_nav.getX() * dt;
    state_(4) += accel_nav.getY() * dt;
    state_(5) += accel_nav.getZ() * dt;

    RCLCPP_DEBUG(this->get_logger(), "IMU prediction step executed.");
}

void SensorFusionNode::predict(double dt)
{
    // Modèle de transition d'état
    // Mise à jour de la position : x = x + vx * dt
    state_(0) += state_(3) * dt;
    state_(1) += state_(4) * dt;
    state_(2) += state_(5) * dt;

    // Les vitesses et les orientations sont mises à jour dans le callback IMU

    // Matrice de transition d'état F
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(9, 9);
    F(0, 3) = dt;
    F(1, 4) = dt;
    F(2, 5) = dt;
    // Les vitesses sont mises à jour avec les accélérations dans le callback IMU
    // Les orientations sont mises à jour avec les vitesses angulaires dans le callback IMU

    // Bruit de processus Q mis à l'échelle par dt
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

        // Remplissage des données de position et d'orientation à partir de l'état
        odom_msg.pose.pose.position.x = state_(0);
        odom_msg.pose.pose.position.y = state_(1);
        odom_msg.pose.pose.position.z = state_(2);

        // Conversion de roll, pitch, yaw en quaternion pour l'orientation
        tf2::Quaternion q;
        q.setRPY(state_(6), state_(7), state_(8));
        odom_msg.pose.pose.orientation = tf2::toMsg(q);

        // Définition des vitesses
        odom_msg.twist.twist.linear.x = state_(3);
        odom_msg.twist.twist.linear.y = state_(4);
        odom_msg.twist.twist.linear.z = state_(5);

        // Remplissage de la matrice de covariance
        for (int i = 0; i < 6; ++i)
        {
            odom_msg.pose.covariance[i * 6 + i] = covariance_(i, i);
        }

        // Publication du message d'odométrie
        odometry_publisher_->publish(odom_msg);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for initial sensor data to publish odometry.");
    }
}

void SensorFusionNode::latLonToENU(double latitude, double longitude, double altitude, double& x, double& y, double& z)
{
    constexpr double a = 6378137.0;                // Demi-grand axe de l'ellipsoïde WGS84 en mètres
    constexpr double f = 1 / 298.257223563;        // Aplatissement de l'ellipsoïde WGS84
    constexpr double e_sq = f * (2 - f);           // Carré de l'excentricité

    // Conversion de la latitude et de la longitude en radians
    double lat_rad = latitude * M_PI / 180.0;
    double lon_rad = longitude * M_PI / 180.0;

    double ref_lat_rad = reference_latitude_ * M_PI / 180.0;
    double ref_lon_rad = reference_longitude_ * M_PI / 180.0;

    // Utiliser l'altitude GPS sans ajustement
    double adjusted_altitude = altitude;

    // Calcul du rayon de courbure en prime verticale
    double N = a / sqrt(1 - e_sq * sin(lat_rad) * sin(lat_rad));

    // Conversion du point actuel GPS en ECEF
    double ecef_x = (N + adjusted_altitude) * cos(lat_rad) * cos(lon_rad);
    double ecef_y = (N + adjusted_altitude) * cos(lat_rad) * sin(lon_rad);
    double ecef_z = (N * (1 - e_sq) + adjusted_altitude) * sin(lat_rad);

    // Conversion du point de référence en ECEF
    N = a / sqrt(1 - e_sq * sin(ref_lat_rad) * sin(ref_lat_rad));
    double ref_ecef_x = (N + reference_altitude_) * cos(ref_lat_rad) * cos(ref_lon_rad);
    double ref_ecef_y = (N + reference_altitude_) * cos(ref_lat_rad) * sin(ref_lon_rad);
    double ref_ecef_z = (N * (1 - e_sq) + reference_altitude_) * sin(ref_lat_rad);

    // Calcul de delta ECEF
    double dx = ecef_x - ref_ecef_x;
    double dy = ecef_y - ref_ecef_y;
    double dz = ecef_z - ref_ecef_z;

    // Conversion de ECEF à ENU
    double sin_lat = sin(ref_lat_rad);
    double cos_lat = cos(ref_lat_rad);
    double sin_lon = sin(ref_lon_rad);
    double cos_lon = cos(ref_lon_rad);

    x = -sin_lon * dx + cos_lon * dy;
    y = -cos_lon * sin_lat * dx - sin_lat * sin_lon * dy + cos_lat * dz;
    z = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz;

    RCLCPP_DEBUG(this->get_logger(), "Converted lat/lon to ENU: (%f, %f, %f) -> (%f, %f, %f)",
                 latitude, longitude, altitude, x, y, z);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
