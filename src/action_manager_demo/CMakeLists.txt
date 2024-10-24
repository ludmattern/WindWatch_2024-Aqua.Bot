cmake_minimum_required(VERSION 3.5)
project(action_manager_demo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Navigation.action"
  "action/Inspection.action"
  "action/Stabilization.action"
  "action/Rotation.action"
  DEPENDENCIES std_msgs
)

# Define action servers
add_executable(navigation_server src/navigation_server.cpp)
ament_target_dependencies(navigation_server rclcpp rclcpp_action std_msgs)

add_executable(inspection_server src/inspection_server.cpp)
ament_target_dependencies(inspection_server rclcpp rclcpp_action std_msgs)

add_executable(stabilization_server src/stabilization_server.cpp)
ament_target_dependencies(stabilization_server rclcpp rclcpp_action std_msgs)

add_executable(rotation_server src/rotation_server.cpp)
ament_target_dependencies(rotation_server rclcpp rclcpp_action std_msgs)

# Define action manager
add_executable(action_manager src/action_manager.cpp)
ament_target_dependencies(action_manager rclcpp rclcpp_action std_msgs)

# Specify the typesupport for the generated interfaces
rosidl_get_typesupport_target(navigation_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Link the typesupport target to the executables
target_link_libraries(navigation_server ${navigation_typesupport_target})
target_link_libraries(inspection_server ${navigation_typesupport_target})
target_link_libraries(stabilization_server ${navigation_typesupport_target})
target_link_libraries(rotation_server ${navigation_typesupport_target})
target_link_libraries(action_manager ${navigation_typesupport_target})

# Install executables
install(TARGETS
  navigation_server
  inspection_server
  stabilization_server
  rotation_server
  action_manager
  DESTINATION lib/${PROJECT_NAME}
)

# Install action files
install(DIRECTORY action
  DESTINATION share/${PROJECT_NAME}/
)

# Finalize package
ament_package()
