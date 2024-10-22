cmake_minimum_required(VERSION 3.5)
project(navigation)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Inclure le répertoire des en-têtes
include_directories(include)

# Add executables
add_executable(path_planning_node src/path_planning_node.cpp)
add_executable(obstacle_avoidance_node src/obstacle_avoidance_node.cpp)
add_executable(control_node src/control_node.cpp)
add_executable(propulsion_control_node src/propulsion_control_node.cpp)

# Specify dependencies for executables
ament_target_dependencies(path_planning_node rclcpp std_msgs geometry_msgs sensor_msgs)
ament_target_dependencies(obstacle_avoidance_node rclcpp sensor_msgs)
ament_target_dependencies(control_node rclcpp std_msgs geometry_msgs sensor_msgs tf2 tf2_geometry_msgs nav_msgs)
ament_target_dependencies(propulsion_control_node rclcpp std_msgs geometry_msgs sensor_msgs tf2 tf2_geometry_msgs)

# Install targets
install(TARGETS
  path_planning_node
  obstacle_avoidance_node
  control_node
  propulsion_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install other necessary folders
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ament package macro
ament_package()