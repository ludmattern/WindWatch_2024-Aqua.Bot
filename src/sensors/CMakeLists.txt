cmake_minimum_required(VERSION 3.5)
project(sensors)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(action_msgs REQUIRED)


# Use pkg-config to find the ZBar library
pkg_check_modules(ZBar REQUIRED zbar)

# Generate ROS2 interfaces (services)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/TargetPositions.srv"
  "action/CameraControl.action"
  DEPENDENCIES std_msgs nav_msgs geometry_msgs action_msgs
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime rclcpp geometry_msgs nav_msgs sensor_msgs tf2 tf2_geometry_msgs)

# Include header files
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  ${ZBar_INCLUDE_DIRS}
)

# Export include directories
ament_export_include_directories(include)

# Add executables
add_executable(sensor_fusion_node src/sensor_fusion_node.cpp src/ConvertCoordinates.cpp)
add_executable(camera_control_node src/camera_control_node.cpp)
add_executable(tgt_pos_update_node src/tgt_pos_update_node.cpp src/ConvertCoordinates.cpp)

# Specify dependencies for executables
ament_target_dependencies(sensor_fusion_node 
  rclcpp 
  sensor_msgs 
  nav_msgs 
  tf2 
  tf2_geometry_msgs 
  rosidl_typesupport_c 
  rosidl_typesupport_cpp
  geometry_msgs
)

ament_target_dependencies(camera_control_node 
  rclcpp 
  image_transport 
  OpenCV 
  sensor_msgs
  rosidl_typesupport_c 
  rosidl_typesupport_cpp
  std_msgs 
  tf2 
  tf2_geometry_msgs 
  nav_msgs
  cv_bridge
  rosidl_default_runtime
)

ament_target_dependencies(tgt_pos_update_node 
  rclcpp 
  sensor_msgs 
  rosidl_typesupport_c 
  rosidl_typesupport_cpp
  geometry_msgs
)

# Get the typesupport target to link generated libraries
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Link typesupport libraries to executables
target_link_libraries(sensor_fusion_node ${cpp_typesupport_target})
target_link_libraries(camera_control_node ${cpp_typesupport_target} ${OpenCV_LIBS} ${ZBar_LIBRARIES})
target_link_libraries(tgt_pos_update_node ${cpp_typesupport_target})

# Install executables
install(TARGETS
  sensor_fusion_node
  camera_control_node
  tgt_pos_update_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install service files
install(DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}/
)

# Install other necessary folders
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Finalize the package
ament_package()