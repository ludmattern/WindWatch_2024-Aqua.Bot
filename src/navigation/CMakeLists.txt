cmake_minimum_required(VERSION 3.5)
project(navigation)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensors REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "srv/Path.srv"
        "srv/PathLast.srv"
        DEPENDENCIES std_msgs geometry_msgs nav_msgs
)

# Inclure le répertoire des en-têtes
include_directories(include)

# Add executables
add_executable(path_planning_node src/path_planning_node/path_planning_node.cpp src/path_planning_node/path_planning_node_create_graph.cpp 
              src/path_planning_node/check_inter_poly.cpp src/path_planning_node/check_inter_line.cpp src/path_planning_node/dijkstra.cpp
              src/path_planning_node/tsp.cpp src/path_planning_node/create_path.cpp)
add_executable(obstacle_avoidance_node src/obstacle_avoidance_node.cpp)
add_executable(control_node src/control_node.cpp)
add_executable(propulsion_control_node src/propulsion_control_node.cpp)

# Specify dependencies for executables
ament_target_dependencies(path_planning_node rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs sensors rosidl_typesupport_c rosidl_default_runtime)
ament_target_dependencies(obstacle_avoidance_node rclcpp sensor_msgs)
ament_target_dependencies(control_node rclcpp std_msgs geometry_msgs sensor_msgs tf2 tf2_geometry_msgs nav_msgs)
ament_target_dependencies(propulsion_control_node rclcpp std_msgs geometry_msgs sensor_msgs tf2 tf2_geometry_msgs)


rosidl_get_typesupport_target(ts_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(path_planning_node ${ts_target})

# Install targets
install(TARGETS
path_planning_node
obstacle_avoidance_node
control_node
propulsion_control_node
DESTINATION lib/${PROJECT_NAME}
)

# Install other necessary folders
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ament package macro
ament_package()