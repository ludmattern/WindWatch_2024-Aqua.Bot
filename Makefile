# ==============================
# Configuration Variables
# ==============================

ROS_SETUP = /opt/ros/humble/setup
VRX_SETUP = install/setup

BASH_ROS_SOURCE = $(ROS_SETUP).sh
BASH_VRX_SOURCE = $(VRX_SETUP).sh
ZSH_VRX_SOURCE = $(VRX_SETUP).zsh
ZSH_ROS_SOURCE = $(ROS_SETUP).zsh

SRC_PATH = src/
BUILD_PATH = build/
INSTALL_PATH = install/
LOG_PATH = log/

DEFAULT_WORLD = aquabot_regatta
COMPET_WORLD = aquabot_windturbines_hard

# ==============================
# Targets - Main Commands
# ==============================

.PHONY: all
all: build run

.PHONY: build
build:
	@echo "\e[32m[INFO] Compiling ROS2 project with colcon...\e[0m"
	colcon build --base-paths $(SRC_PATH) --merge-install
	@echo "\e[32m[INFO] Please execute '. $(BASH_VRX_SOURCE)' to set up the environment\e[0m"

.PHONY: run
run:
	@echo "\e[32m[INFO] Launching Aquabot competition with the default world: $(DEFAULT_WORLD)...\e[0m"
	. $(BASH_ROS_SOURCE) && . $(BASH_VRX_SOURCE) && ros2 launch aquabot_gz competition.launch.py world:=$(DEFAULT_WORLD) competition_mode:=false > /dev/null &

.PHONY: runfront
runfront:
	@echo "\e[32m[INFO] Launching Aquabot competition with the default world: $(DEFAULT_WORLD)...\e[0m"
	. $(BASH_ROS_SOURCE) && . $(BASH_VRX_SOURCE) && ros2 launch aquabot_gz competition.launch.py world:=$(DEFAULT_WORLD) competition_mode:=false

.PHONY: launch_mission
launch_mission:
	@echo "\e[32m[INFO] Launching the complete mission (all packages)...\e[0m"
	. $(BASH_ROS_SOURCE) && . $(BASH_VRX_SOURCE) && ros2 launch mission_manager mission_launch.py

.PHONY: launch_headless
launch_headless:
	@echo "\e[32m[INFO] Launching the mission in headless mode...\e[0m"
	. $(BASH_ROS_SOURCE) && . $(BASH_VRX_SOURCE) && ros2 launch mission_manager mission_launch.py headless:=true

.PHONY: fclean
fclean:
	@echo "\e[31m[INFO] Performing a complete clean-up of generated files...\e[0m"
	rm -rf $(BUILD_PATH) $(INSTALL_PATH) $(LOG_PATH)

.PHONY: re
re: fclean build run

.PHONY: perception
perception:
	@echo "\e[32m[INFO] Launching the perception node...\e[0m"
	. $(BASH_ROS_SOURCE) && . $(BASH_VRX_SOURCE) && ros2 run sensors camera_processing_node

.PHONY: navigation
navigation:
	@echo "\e[32m[INFO] Launching the navigation node...\e[0m"
	. $(BASH_ROS_SOURCE) && . $(BASH_VRX_SOURCE) && ros2 run navigation navigation_node

.PHONY: obstacle_avoidance
obstacle_avoidance:
	@echo "\e[32m[INFO] Launching the obstacle avoidance node...\e[0m"
	. $(BASH_ROS_SOURCE) && . $(BASH_VRX_SOURCE) && ros2 run navigation obstacle_avoidance_node

.PHONY: mission_coordinator
mission_coordinator:
	@echo "\e[32m[INFO] Launching the mission coordinator node...\e[0m"
	. $(BASH_ROS_SOURCE) && . $(BASH_VRX_SOURCE) && ros2 run mission_manager mission_coordinator_node

.PHONY: rviz
rviz:
	@echo "\e[32m[INFO] Launching RViz for visualization...\e[0m"
	. $(BASH_ROS_SOURCE) && . $(BASH_VRX_SOURCE) && ros2 launch visualization rviz.launch.py > /dev/null &

.PHONY: teleop
teleop:
	@echo "\e[32m[INFO] Manual control via teleoperation...\e[0m"
	. $(BASH_ROS_SOURCE) && . $(BASH_VRX_SOURCE) && ros2 run aquabot_python teleop_keyboard.py

.PHONY: debug
debug:
	@echo "\e[32m[INFO] Launching in Debug mode (Run, RViz, and RQT)...\e[0m"
	$(MAKE) run
	$(MAKE) rviz
	. $(BASH_ROS_SOURCE) && . $(BASH_VRX_SOURCE) && rqt &

.PHONY: add_shell_source
add_shell_source:
	@echo "\e[32m[INFO] Adding source commands to the shell...\e[0m"
	echo "# Source ROS Humble" >> ~/.bashrc
	echo "source $(BASH_ROS_SOURCE)" >> ~/.bashrc
	echo "# Source VRX" >> ~/.bashrc
	echo "source $(BASH_VRX_SOURCE)" >> ~/.bashrc
	echo "# Source ROS Humble" >> ~/.zshrc
	echo "source $(ZSH_ROS_SOURCE)" >> ~/.zshrc
	echo "# Source VRX" >> ~/.zshrc
	echo "source $(ZSH_VRX_SOURCE)" >> ~/.zshrc

.PHONY: check_env
check_env:
	@echo "\e[32m[INFO] Checking environment variables...\e[0m"
	. $(BASH_ROS_SOURCE) && . $(BASH_VRX_SOURCE) && \
	if [ -z "$ROS_VERSION" ]; then \
	    echo "\e[31m[ERROR] ROS is not sourced. Please source ROS Humble before continuing.\e[0m"; \
	    exit 1; \
	fi; \
	if [ -z "$AMENT_PREFIX_PATH" ]; then \
	    echo "\e[31m[ERROR] VRX environment is not sourced. Please source the VRX environment.\e[0m"; \
	    exit 1; \
	fi; \
	echo "\e[32m[INFO] Environment configured correctly.\e[0m"

.PHONY: prepare
prepare:
	@echo "\e[32m[INFO] Preparing the project (build and verification)...\e[0m"
	$(MAKE) check_env
	$(MAKE) build

.PHONY: dependencies
dependencies:
	@echo "\e[32m[INFO] Installing necessary dependencies...\e[0m"
	apt update
	apt install -y \
		ros-humble-rosidl-typesupport-c \
		libeigen3-dev

# ==============================
# Usage Instructions
# ==============================
# To build the project, use: make build
# To run the project, use: make run
# To clean the project, use: make fclean
# To check the environment, use: make check_env
# To rebuild completely, use: make re
