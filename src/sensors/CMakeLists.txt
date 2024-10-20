cmake_minimum_required(VERSION 3.5)
project(sensors)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include header files
include_directories(include)

# Create executable targets
add_executable(sensor_fusion_node src/sensor_fusion_node.cpp)
add_executable(camera_processing_node src/camera_processing_node.cpp)
add_executable(tgt_pos_update_node src/tgt_pos_update_node.cpp)

# Link dependencies
ament_target_dependencies(sensor_fusion_node rclcpp sensor_msgs)
ament_target_dependencies(camera_processing_node rclcpp sensor_msgs)
ament_target_dependencies(tgt_pos_update_node rclcpp sensor_msgs)

# Install targets
install(TARGETS
  sensor_fusion_node
  camera_processing_node
  tgt_pos_update_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
