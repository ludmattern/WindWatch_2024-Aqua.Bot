cmake_minimum_required(VERSION 3.22)

project(simple_package)

# Trouver les dépendances requises pour votre package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Ajouter l'exécutable de votre nœud
add_executable(simple_node src/simple_node.cpp)
ament_target_dependencies(simple_node rclcpp std_msgs sensor_msgs)

# Installer l'exécutable dans le répertoire de lib de votre package
install(TARGETS
  simple_node
  DESTINATION lib/${PROJECT_NAME})

# Inclure les répertoires d'en-têtes
ament_export_dependencies(rclcpp std_msgs)
ament_package()

# Installer le fichier de lancement
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)
