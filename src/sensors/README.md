# Sensor Fusion Node for ROS 2

## Overview
This project implements a sensor fusion node using an Extended Kalman Filter (EKF) in ROS 2. The node fuses data from a GPS and an IMU to provide an accurate estimation of a vessel's position and orientation within a 1 km × 1 km area. The primary objective is to navigate a vessel through a rocky area centered around the coordinates 48.046300° N, -4.976320° E.

## Introduction
Navigating in environments with obstacles such as rocks requires precise localization and orientation data. This sensor fusion node combines GPS and IMU data to enhance the accuracy of the vessel's state estimation. By implementing an Extended Kalman Filter, the node accounts for sensor noise and provides a robust estimation suitable for navigation tasks.

## Features
- **Sensor Fusion with EKF**: Integrates GPS and IMU data for improved state estimation.
- **Localization in ENU Coordinates**: Converts GPS data from WGS84 to local East-North-Up (ENU) coordinates.
- **Real-Time Odometry Publishing**: Publishes the estimated state as an odometry message at 10 Hz.
- **Configurable Noise Parameters**: Allows adjustment of process and measurement noise covariances.
- **ROS 2 Compatible**: Designed to work with ROS 2 (tested on ROS 2 Humble Hawksbill).

## System Requirements
- **Dependencies**:
  - `rclcpp`
  - `sensor_msgs`
  - `nav_msgs`
  - `tf2` and `tf2_geometry_msgs`
  - `Eigen3`

## Node Details
### Topics Subscribed
- **GPS Data**: `/aquabot/sensors/gps/gps/fix`
  - **Message Type**: `sensor_msgs/msg/NavSatFix`
  - **Update Rate**: 20 Hz
  - **Horizontal Position Noise**: 0.85 m
  - **Vertical Position Noise**: 2.0 m

- **IMU Data**: `/aquabot/sensors/imu/imu/data`
  - **Message Type**: `sensor_msgs/msg/Imu`
  - **Update Rate**: 100 Hz
  - **Acceleration Noise**: 0.275 g
  - **Attitude Rate Noise**: 0.08 degrees/s
  - **Heading Noise**: 0.8 degrees

### Topics Published
- **Odometry**: `/mission/odometry`
  - **Message Type**: `nav_msgs/msg/Odometry`
  - **Publishing Rate**: 10 Hz

### Parameters
- **Reference Latitude**: 48.046300° N
- **Reference Longitude**: -4.976320° E
- **Reference Altitude**: 0.0 m

## Algorithm Explanation
### State Vector
The state vector is defined as:
\[ x = [x, y, z, v_x, v_y, v_z, φ, θ, ψ]^T \]
Where:
- **x, y, z**: Position in ENU coordinates.
- **v_x, v_y, v_z**: Linear velocities in ENU coordinates.
- **φ, θ, ψ**: Roll, pitch, and yaw angles.

### Process Model
The process model predicts the next state based on the current state and control inputs (from the IMU):
\[ x_{k+1} = F x_k + G u_k + w_k \]
- **State Transition Matrix (F)**: Incorporates the effect of velocities on positions over time **dt**.
- **Control Input (u_k)**: Includes accelerations and angular velocities from the IMU.
- **Process Noise (w_k)**: Models the uncertainty in the process (defined in `process_noise_`).

### Measurement Model
The measurement model relates the state to the measurements from the GPS and IMU:
\[ z_k = H x_k + v_k \]
- **Measurement Matrix (H)**: Maps the state vector to the measurement space.
- **Measurement Noise (v_k)**: Models the sensor noise (defined in `measurement_noise_`).

### Extended Kalman Filter Implementation
**Prediction Step**:
- **State Prediction**:
  \[ ˆx_{k|k-1} = F ˆx_{k-1|k-1} \]
- **Covariance Prediction**:
  \[ P_{k|k-1} = F P_{k-1|k-1} F^T + Q \]

**Update Step (GPS Callback)**:
- **Innovation**:
  \[ y_k = z_k - H ˆx_{k|k-1} \]
- **Innovation Covariance**:
  \[ S_k = H P_{k|k-1} H^T + R \]
- **Kalman Gain**:
  \[ K_k = P_{k|k-1} H^T S_k^{-1} \]
- **State Update**:
  \[ ˆx_{k|k} = ˆx_{k|k-1} + K_k y_k \]
- **Covariance Update**:
  \[ P_{k|k} = (I - K_k H) P_{k|k-1} \]

**IMU Data Integration**:
- IMU data is used in the prediction step to update velocities and orientations.
- Accelerations are integrated to update velocities, and angular velocities are integrated to update orientations.

## Coordinate Conversion
The node converts GPS coordinates from latitude, longitude, and altitude (LLA) in WGS84 to local ENU coordinates using the provided reference point.

### Conversion Steps
**LLA to ECEF (Earth-Centered, Earth-Fixed)**:
- Uses WGS84 ellipsoid parameters.
- Calculates ECEF coordinates for both the current position and the reference point.

**ECEF to ENU**:
- Computes the difference between the current ECEF and reference ECEF.
- Applies a rotation to align the axes to the local ENU frame.

### Justification
- For areas of size 1 km × 1 km, the ENU approximation provides sufficient accuracy.
- Simplifies computations compared to using global coordinate systems.

## Customization
- **Adjust Noise Parameters**: Modify `process_noise_` and `measurement_noise_` matrices to reflect the characteristics of your sensors.
- **Change Reference Point**: Update `reference_latitude_`, `reference_longitude_`, and `reference_altitude_` for a different operational area.
- **Extend the State Vector**: Include additional states (e.g., biases, higher-order dynamics) if necessary.
- **Incorporate Additional Sensors**: Extend the measurement model to include data from other sensors like LIDAR or sonar.