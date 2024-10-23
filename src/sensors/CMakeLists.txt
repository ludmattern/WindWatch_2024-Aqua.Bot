cmake_minimum_required(VERSION 3.5)
project(sensors)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(include)

# Add executables
add_executable(sensor_fusion_node src/sensor_fusion_node.cpp src/ConvertCoordinates.cpp)
add_executable(camera_control_node src/camera_control_node.cpp)
add_executable(tgt_pos_update_node src/tgt_pos_update_node.cpp src/ConvertCoordinates.cpp)
add_executable(camera_processing_node src/camera_processing_node.cpp)

# Specify libraries for executables
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera_processing_node ${OpenCV_LIBS})

# Specify dependencies for executables
ament_target_dependencies(sensor_fusion_node rclcpp sensor_msgs nav_msgs tf2 tf2_geometry_msgs rosidl_typesupport_c)
ament_target_dependencies(camera_control_node rclcpp image_transport OpenCV sensor_msgs rosidl_typesupport_c std_msgs tf2 tf2_geometry_msgs nav_msgs)
ament_target_dependencies(tgt_pos_update_node rclcpp sensor_msgs rosidl_typesupport_c)
ament_target_dependencies(camera_processing_node rclcpp sensor_msgs cv_bridge)

# Install targets
install(TARGETS
  sensor_fusion_node
  camera_control_node
  tgt_pos_update_node
  camera_processing_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install other necessary folders
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ament package macro
ament_package()
