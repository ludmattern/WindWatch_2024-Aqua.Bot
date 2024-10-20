# Mission WindWatch 2024 - Aqua.Bot

Welcome to the **Mission WindWatch 2024** project. This project aims to develop an autonomous maritime drone for a mission involving **surveillance**, **inspection**, and **autonomous navigation** in a complex maritime environment. The drone must be capable of navigating around obstacles, inspecting targets (wind turbines), and achieving mission objectives such as fault identification and obstacle avoidance. This project focuses on integrating multiple sensors, controlling propulsion, and coordinating the mission in a dynamic environment.

This **README** provides an overview of the project structure, the different **ROS2 packages**, their associated nodes, as well as the **custom topics** used for communication between the nodes.

## How to Add This Package to the Project

To get started, you can add the base package to your project using the following commands:

```bash
# Navigate to the vrx_ws directory
cd ~/mission_windwatch

# Installation
colcon build --merge-install
. install/setup.bash

# Launch the node
ros2 launch ros2 launch mission_manager mission_launch.py
```

These steps allow you to create and launch a base package. The `simple_package` package is a starting point, but it will need to be modified and extended to include the specific functionalities of Mission WindWatch 2024.

## Project Structure

The project is organized into several **ROS2 packages** to ensure a modular, clear, and extensible approach. Each package contains a set of nodes responsible for specific tasks related to the mission.

### Packages and Nodes

#### 1. Package `navigation`

This package contains nodes responsible for **navigation** and **motor control**.

- **Node `navigation_node`**
  - **Role**: Responsible for **autonomous navigation**. Plans paths to reach mission objectives while avoiding obstacles.
  - **Subscribed Topics**:
    - `/mission/odometry`: Fused odometry (position and orientation).
    - `/mission/objective_positions`: Positions of targets to be reached.
    - `/mission/avoidance_course`: Avoidance trajectory to follow to avoid obstacles.
    - `/mission/target_orientations`: Wind turbine orientations for trajectory adjustments.
    - `/aqua_bot/ais_sensor/vessel_positions`: Positions of nearby vessels to avoid collisions.
  - **Published Topics**:
    - `/propulsion/command`: Thrust and orientation commands for the motors.

- **Node `propulsion_control_node`**
  - **Role**: Manages **propulsor commands** (thrust and orientation).
  - **Subscribed Topics**:
    - `/propulsion/command`: Commands from the `navigation_node`.
  - **Published Topics**:
    - `/aqua_bot/thrusters/left/pos`, `/aqua_bot/thrusters/right/pos`: Orientation commands for the motors.
    - `/aqua_bot/thrusters/left/thrust`, `/aqua_bot/thrusters/right/thrust`: Thrust commands for the motors.

#### 2. Package `sensors`

This package contains nodes responsible for **collecting sensor data** and **fusing** it to provide a reliable global view.

- **Node `sensor_fusion_node`**
  - **Role**: Fuses GPS and IMU data using an **EKF** to obtain a reliable estimate of the boat's position and orientation.
  - **Subscribed Topics**:
    - `/aqua_bot/sensors/gps/fix`: GPS data.
    - `/aqua_bot/sensors/imu/data`: IMU data.
  - **Published Topics**:
    - `/mission/odometry`: Fused odometry for other nodes.

- **Node `camera_processing_node`**
  - **Role**: Processes **camera** images to identify targets, read QR codes, and determine wind turbine orientations.
  - **Subscribed Topics**:
    - `/aqua_bot/sensors/cameras/main_camera/image_raw`: Images from the 360° camera.
  - **Published Topics**:
    - `/mission/target_status`: Status of detected wind turbines (functional or defective).
    - `/mission/target_orientations`: Orientation of the targets (wind turbines).

- **Node `tgt_pos_update_node`**
  - **Role**: Processes target positions received by **GPS** and transmits them.
  - **Subscribed Topics**:
    - `/aqua_bot/ais_sensor/windturbines_positions`: List of GPS positions of the wind turbines.
  - **Published Topics**:
    - `/aqua_bot/ais_sensor/target_positions`: Positions of the wind turbines.

#### 3. Package `mission_manager`

This package manages **mission planning**, **mission objectives**, and mission **coordination**.

- **Node `target_manager_node`**
  - **Role**: Centralizes and updates information on wind turbines (position, orientation, status).
  - **Subscribed Topics**:
    - `/aqua_bot/ais_sensor/target_positions`: Positions of the wind turbines.
    - `/mission/target_orientations`: Updated orientations of the wind turbines.
  - **Published Topics**:
    - `/mission/objective_positions`: Positions of the wind turbines to reach.

- **Node `obstacle_avoidance_node`**
  - **Role**: Identifies **obstacles** and suggests trajectory modifications to avoid them.
  - **Data Sources**:
    - **Fixed Obstacles**: Coordinates of fixed obstacles (rocks, islands, lighthouse) are available from the start via a configuration file.
  - **Subscribed Topics**:
    - `/mission/odometry`: Fused odometry.
    - `/aqua_bot/ais_sensor/vessel_positions`: Positions of other vessels to avoid collisions.
  - **Published Topics**:
    - `/mission/avoidance_course`: Proposed avoidance trajectory.

- **Node `mission_coordinator_node`**
  - **Role**: Coordinates **mission phases**, issues mission commands.
  - **Subscribed Topics**:
    - `/mission/target_status`: Status of the targets (wind turbines).
  - **Published Topics**:
    - `/mission/mission_goal`: Mission objectives for the `navigation_node`.

#### 4. Package `visualization`

This package is responsible for **real-time supervision** and **visualization** of the mission.

- **Node `visualization_node`**
  - **Role**: Provides tactical visualization through **RViz2**.
  - **Subscribed Topics**:
    - `/mission/odometry`: Position and orientation of the drone.
    - `/aqua_bot/ais_sensor/target_positions`: Positions of targets and obstacles.
    - `/mission/target_status`: Status of the wind turbines.
    - `/mission/target_orientations`: Orientation of the targets.
    - `/aqua_bot/ais_sensor/vessel_positions`: Positions of other vessels.

### Custom Topics

To ensure effective communication between nodes, several **custom topics** are used:

1. **Topic `/mission/target_orientations`**
   - **Description**: Contains the orientations of the targets (wind turbines).
   - **Published By**: `camera_processing_node`.
   - **Subscribed By**: `target_manager_node`, `navigation_node`.

2. **Topic `/mission/objective_positions`**
   - **Description**: Positions of the targets to be reached.
   - **Published By**: `target_manager_node`.
   - **Subscribed By**: `navigation_node`.

3. **Topic `/propulsion/command`**
   - **Description**: Commands for motor position and thrust.
   - **Published By**: `navigation_node`.
   - **Subscribed By**: `propulsion_control_node`.

4. **Topic `/mission/target_status`**
   - **Description**: Status of the wind turbines (functional or defective).
   - **Published By**: `camera_processing_node`.
   - **Subscribed By**: `mission_coordinator_node`.

5. **Topic `/mission/avoidance_course`**
   - **Description**: Trajectory modifications to avoid obstacles.
   - **Published By**: `obstacle_avoidance_node`.
   - **Subscribed By**: `navigation_node`.

6. **Topic `/mission/mission_goal`**
   - **Description**: Tactical objectives to achieve.
   - **Published By**: `mission_coordinator_node`.
   - **Subscribed By**: `navigation_node`.

7. **Topic `/aqua_bot/ais_sensor/vessel_positions`**
   - **Description**: Contains positions of nearby vessels to avoid collisions.
   - **Published By**: AIS (Automated Identification System).
   - **Subscribed By**: `navigation_node`, `obstacle_avoidance_node`, `visualization_node`.

8. **Topic `/aqua_bot/ais_sensor/target_positions`**
   - **Description**: Positions of detected wind turbines.
   - **Published By**: `tgt_pos_update_node`.
   - **Subscribed By**: `target_manager_node`, `visualization_node`.

## Installation and Launching

1. **Installation**
   - Clone the required repositories into your ROS2 workspace.
   - Compile with `colcon build`.

2. **Launch the Simulation**
   - Use the following command to launch the simulation:
     ```bash
     ros2 launch aquabot_gz competition.launch.py world:=aquabot_regatta
     ```
   - To launch the training mode without the graphical interface:
     ```bash
     ros2 launch aquabot_gz competition.launch.py world:=aquabot_regatta headless:=true
     ```

## Conclusion

This modular and clear structure ensures a **better understanding** and **scalability** of the system. Each node has a specific role in the mission, and the custom topics allow optimal communication between the nodes.

