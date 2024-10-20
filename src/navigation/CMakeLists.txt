cmake_minimum_required(VERSION 3.5)
project(navigation)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include header files
include_directories(include)

# Create executable targets
add_executable(navigation_node src/navigation_node.cpp)
add_executable(obstacle_avoidance_node src/obstacle_avoidance_node.cpp)

# Link dependencies
ament_target_dependencies(navigation_node rclcpp std_msgs)
ament_target_dependencies(obstacle_avoidance_node rclcpp std_msgs)

# Install targets
install(TARGETS
  navigation_node
  obstacle_avoidance_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
