cmake_minimum_required(VERSION 3.5)
project(visualization)

# Inclure le répertoire des en-têtes
include_directories(include)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensors REQUIRED)
find_package(visualization_msgs REQUIRED)

# Add executables
add_executable(visualization_node src/visualization_node.cpp)

# Specify dependencies for executables
ament_target_dependencies(visualization_node rclcpp sensor_msgs geometry_msgs visualization_msgs nav_msgs sensors)

# Install targets
install(TARGETS
  visualization_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install other necessary folders
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ament package macro
ament_package()
