cmake_minimum_required(VERSION 3.5)
project(mission_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Navigation.action"
  "action/Inspection.action"
  "action/Stabilization.action"
  "action/Rotation.action"
  "srv/TargetManagerServ.srv"
  DEPENDENCIES std_msgs nav_msgs geometry_msgs
)

# Export dependencies before ament_package()
ament_export_dependencies(rosidl_default_runtime rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)

# Include header files
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# Export include directories
ament_export_include_directories(include)

# Add executables
add_executable(navigation_server
  src/navigation_server.cpp
  src/PIDController.cpp
  src/controlUtils.cpp
)
ament_target_dependencies(navigation_server
  rclcpp
  rclcpp_action
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rosidl_default_runtime
)

add_executable(inspection_server
  src/inspection_server.cpp
  src/PIDController.cpp
  src/controlUtils.cpp
)
ament_target_dependencies(inspection_server
  rclcpp
  rclcpp_action
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rosidl_default_runtime
)

add_executable(stabilization_server
  src/stabilization_server.cpp
  src/PIDController.cpp
  src/controlUtils.cpp  
)
ament_target_dependencies(stabilization_server
  rclcpp
  rclcpp_action
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rosidl_default_runtime
)

add_executable(rotation_server src/rotation_server.cpp)
ament_target_dependencies(rotation_server
  rclcpp
  rclcpp_action
  std_msgs
  rosidl_default_runtime
)

add_executable(mission_manager_node src/mission_manager.cpp)
ament_target_dependencies(mission_manager_node
  rclcpp
  rclcpp_action
  std_msgs
  rosidl_default_runtime
)

add_executable(target_manager_node src/target_manager_node.cpp)
ament_target_dependencies(target_manager_node
  rclcpp
  rclcpp_action
  std_msgs
  rosidl_default_runtime
)

# Get the typesupport target to link generated libraries
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Link typesupport libraries to executables
target_link_libraries(navigation_server ${cpp_typesupport_target})
target_link_libraries(inspection_server ${cpp_typesupport_target})
target_link_libraries(stabilization_server ${cpp_typesupport_target})
target_link_libraries(rotation_server ${cpp_typesupport_target})
target_link_libraries(mission_manager_node ${cpp_typesupport_target})
target_link_libraries(target_manager_node ${cpp_typesupport_target})

# Install executables
install(TARGETS
  navigation_server
  inspection_server
  stabilization_server
  rotation_server
  mission_manager_node
  target_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install action files
install(DIRECTORY action
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch and config files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Finalize the package
ament_package()
