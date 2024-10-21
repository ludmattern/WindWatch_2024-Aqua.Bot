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

\[
\mathbf{x} = \begin{bmatrix}
x & y & z & v_x & v_y & v_z & \phi & \theta & \psi
\end{bmatrix}^T
\]

Where:
- \( x, y, z \): Position in ENU coordinates.
- \( v_x, v_y, v_z \): Linear velocities in ENU coordinates.
- \( \phi, \theta, \psi \): Roll, pitch, and yaw angles.

### Process Model

The process model predicts the next state based on the current state and control inputs (from the IMU):

\[
\mathbf{x}_{k+1} = \mathbf{F} \mathbf{x}_k + \mathbf{G} \mathbf{u}_k + \mathbf{w}_k
\]

- **State Transition Matrix (\( \mathbf{F} \))**:
  - Incorporates the effect of velocities on positions over time \( dt \).
- **Control Input (\( \mathbf{u}_k \))**:
  - Includes accelerations and angular velocities from the IMU.
- **Process Noise (\( \mathbf{w}_k \))**:
  - Models the uncertainty in the process (defined in `process_noise_`).

### Measurement Model

The measurement model relates the state to the measurements from the GPS and IMU:

\[
\mathbf{z}_k = \mathbf{H} \mathbf{x}_k + \mathbf{v}_k
\]

- **Measurement Matrix (\( \mathbf{H} \))**:
  - Maps the state vector to the measurement space.
- **Measurement Noise (\( \mathbf{v}_k \))**:
  - Models the sensor noise (defined in `measurement_noise_`).

### Extended Kalman Filter Implementation

1. **Prediction Step**:
   - **State Prediction**:
     \[
     \hat{\mathbf{x}}_{k|k-1} = \mathbf{F} \hat{\mathbf{x}}_{k-1|k-1}
     \]
   - **Covariance Prediction**:
     \[
     \mathbf{P}_{k|k-1} = \mathbf{F} \mathbf{P}_{k-1|k-1} \mathbf{F}^T + \mathbf{Q}
     \]
   - **Process Noise (\( \mathbf{Q} \))**:
     - Scaled by the time difference \( dt \).

2. **Update Step (GPS Callback)**:
   - **Compute Innovation**:
     \[
     \mathbf{y}_k = \mathbf{z}_k - \mathbf{H} \hat{\mathbf{x}}_{k|k-1}
     \]
   - **Innovation Covariance**:
     \[
     \mathbf{S}_k = \mathbf{H} \mathbf{P}_{k|k-1} \mathbf{H}^T + \mathbf{R}
     \]
   - **Kalman Gain**:
     \[
     \mathbf{K}_k = \mathbf{P}_{k|k-1} \mathbf{H}^T \mathbf{S}_k^{-1}
     \]
   - **State Update**:
     \[
     \hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k \mathbf{y}_k
     \]
   - **Covariance Update**:
     \[
     \mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k \mathbf{H}) \mathbf{P}_{k|k-1}
     \]

3. **IMU Data Integration**:
   - IMU data is primarily used in the prediction step to update velocities and orientations.
   - Accelerations are integrated to update velocities.
   - Angular velocities are integrated to update orientations.

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