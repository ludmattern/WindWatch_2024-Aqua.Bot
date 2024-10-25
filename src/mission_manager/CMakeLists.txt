cmake_minimum_required(VERSION 3.5)
project(mission_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Trouver les packages requis
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# Générer les interfaces d'action
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Navigation.action"
  "action/Inspection.action"
  "action/Stabilization.action"
  "action/Rotation.action"
  DEPENDENCIES std_msgs
)

# Assurer que les messages sont générés avant de construire les exécutables
ament_export_dependencies(rosidl_default_runtime)

# Inclure les fichiers header
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# Exporter les répertoires d'inclusion
ament_export_include_directories(include)

# Ajouter les exécutables
add_executable(navigation_server src/navigation_server.cpp)
ament_target_dependencies(navigation_server rclcpp rclcpp_action std_msgs rosidl_default_runtime)

add_executable(inspection_server src/inspection_server.cpp)
ament_target_dependencies(inspection_server rclcpp rclcpp_action std_msgs rosidl_default_runtime)

add_executable(stabilization_server src/stabilization_server.cpp)
ament_target_dependencies(stabilization_server rclcpp rclcpp_action std_msgs rosidl_default_runtime)

add_executable(rotation_server src/rotation_server.cpp)
ament_target_dependencies(rotation_server rclcpp rclcpp_action std_msgs rosidl_default_runtime)

add_executable(mission_manager_node src/mission_manager.cpp)
ament_target_dependencies(mission_manager_node rclcpp rclcpp_action std_msgs rosidl_default_runtime)

add_executable(target_manager_node src/target_manager_node.cpp)
ament_target_dependencies(target_manager_node rclcpp rclcpp_action std_msgs rosidl_default_runtime)

# Obtenir la cible de typesupport pour lier les bibliothèques générées
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Lier les bibliothèques de typesupport aux exécutables
target_link_libraries(navigation_server ${cpp_typesupport_target})
target_link_libraries(inspection_server ${cpp_typesupport_target})
target_link_libraries(stabilization_server ${cpp_typesupport_target})
target_link_libraries(rotation_server ${cpp_typesupport_target})
target_link_libraries(mission_manager_node ${cpp_typesupport_target})

# Installer les exécutables
install(TARGETS
  navigation_server
  inspection_server
  stabilization_server
  rotation_server
  mission_manager_node
  target_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

# Installer les fichiers header
install(DIRECTORY include/
  DESTINATION include/
)

# Installer les fichiers d'action
install(DIRECTORY action
  DESTINATION share/${PROJECT_NAME}/
)

# Installer les fichiers de lancement
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Finaliser le package
ament_package()
