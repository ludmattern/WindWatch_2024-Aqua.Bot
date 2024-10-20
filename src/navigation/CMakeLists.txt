cmake_minimum_required(VERSION 3.5)
project(navigation)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Inclure le répertoire des en-têtes
include_directories(include)

# Add executables
add_executable(navigation_node src/navigation_node.cpp)
add_executable(obstacle_avoidance_node src/obstacle_avoidance_node.cpp)

# Specify dependencies for executables
ament_target_dependencies(navigation_node rclcpp std_msgs geometry_msgs sensor_msgs)
ament_target_dependencies(obstacle_avoidance_node rclcpp sensor_msgs)

# Install targets
install(TARGETS
  navigation_node
  obstacle_avoidance_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install other necessary folders
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ament package macro
ament_package()