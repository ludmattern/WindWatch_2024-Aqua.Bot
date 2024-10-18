# How to add this package to the project
ros2 pkg create --build-type ament_cmake --node-name simple_node simple_package

# Go to vrx_ws directory
cd ~/vrx_ws

# install
colcon build --merge-install
. install/setup.bash

# launch
ros2 launch simple_package simple_node_launch.py