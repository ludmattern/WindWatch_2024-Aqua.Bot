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
  "action/Action1.action"
  "action/Action2.action"
  DEPENDENCIES std_msgs
)

# Define action servers
add_executable(action1_server src/action1_server.cpp)
ament_target_dependencies(action1_server rclcpp rclcpp_action std_msgs)

add_executable(action2_server src/action2_server.cpp)
ament_target_dependencies(action2_server rclcpp rclcpp_action std_msgs)

# Define action manager
add_executable(action_manager src/action_manager.cpp src/action_manager_main.cpp)
ament_target_dependencies(action_manager rclcpp rclcpp_action std_msgs)

# Specify the typesupport for the generated interfaces
rosidl_get_typesupport_target(action1_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Link the typesupport target to the executables
target_link_libraries(action1_server ${action1_typesupport_target})
target_link_libraries(action2_server ${action1_typesupport_target})
target_link_libraries(action_manager ${action1_typesupport_target})

# Install executables
install(TARGETS
  action1_server
  action2_server
  action_manager
  DESTINATION lib/${PROJECT_NAME}
)

# Install action files
install(DIRECTORY action
  DESTINATION share/${PROJECT_NAME}/
)

# Finalize package
ament_package()
