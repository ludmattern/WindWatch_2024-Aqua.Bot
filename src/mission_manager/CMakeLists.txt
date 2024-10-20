cmake_minimum_required(VERSION 3.5)
project(mission_manager)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include header files
include_directories(include)

# Create executable targets
add_executable(target_manager_node src/target_manager_node.cpp)
add_executable(mission_coordinator_node src/mission_coordinator_node.cpp)

# Link dependencies
ament_target_dependencies(target_manager_node rclcpp std_msgs)
ament_target_dependencies(mission_coordinator_node rclcpp std_msgs)

# Install targets
install(TARGETS
  target_manager_node
  mission_coordinator_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
