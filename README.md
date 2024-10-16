# How to add this package to the project

# Go to vrx_ws directory
cd ~/vrx_ws

# install
colcon build --merge-install
. install/setup.bash

# launch
ros2 launch simple_package simple_node_launch.py