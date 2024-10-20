cmake_minimum_required(VERSION 3.5)
project(visualization)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include header files
include_directories(include)

# Create executable targets
add_executable(visualization_node src/visualization_node.cpp)

# Link dependencies
ament_target_dependencies(visualization_node rclcpp std_msgs)

# Install targets
install(TARGETS
  visualization_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
