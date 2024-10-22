/*
Node obstacle_avoidance_node
(Moved from the mission_manager package to the navigation package for better coherence.)

Role: Detects obstacles in the environment and generates avoidance maneuvers to prevent collisions.
Subscribed Topics:
/mission/odometry: Current position and orientation of the boat.
/aqua_bot/ais_sensor/vessel_positions: Positions of other vessels to avoid collisions.
/aqua_bot/ais_sensor/obstacle_positions: Positions of static obstacles.
/mission/objective_positions: Positions of mission objectives to consider in avoidance planning.
Published Topics:
/mission/avoidance_course: Suggested avoidance trajectory modifications for the path_planning_node.
*/
#include "navigation/obstacle_avoidance_node.hpp"

ObstacleAvoidanceNode::ObstacleAvoidanceNode() : Node("obstacle_avoidance_node")
{
    RCLCPP_INFO(this->get_logger(), "Obstacle Avoidance Node has started");
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleAvoidanceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
