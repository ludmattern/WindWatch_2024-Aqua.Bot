// obstacle_avoidance_node.cpp

#include "visualization/visualization_node.hpp"

geometry_msgs::msg::PointStamped LightHouse;
geometry_msgs::msg::PointStamped Island1;
geometry_msgs::msg::PointStamped Island2;
geometry_msgs::msg::PointStamped RockIsland0;
geometry_msgs::msg::PointStamped RockIsland1;
geometry_msgs::msg::PointStamped Rock0;
geometry_msgs::msg::PointStamped Rock1;
geometry_msgs::msg::PointStamped Rock2;
geometry_msgs::msg::PointStamped Rock3;

VisualizationNode::VisualizationNode() : Node("visualization_node")
{
    // Subscriptions
    //odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //    "/mission/odometry", 10,
    //    std::bind(&VisualizationNode::odometryCallback, this, std::placeholders::_1));

	// Publishers
    light_house_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/minimap/light_house", 10);
	island_1_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/minimap/island_1", 10);
	island_2_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/minimap/island_2", 10);
	rock_island_0_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/minimap/rock_island_0", 10);
	rock_island_1_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/minimap/rock_island_1", 10);
	rock_0_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/minimap/rock_0", 10);
	rock_1_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/minimap/rock_1", 10);
	rock_2_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/minimap/rock_2", 10);
	rock_3_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/minimap/rock_3", 10);

	// Fixed frame name = odom
	LightHouse.header.frame_id = "odom";
	Island1.header.frame_id = "odom";
	Island2.header.frame_id = "odom";
	RockIsland0.header.frame_id = "odom";
	RockIsland1.header.frame_id = "odom";
	Rock0.header.frame_id = "odom";
	Rock1.header.frame_id = "odom";
	Rock2.header.frame_id = "odom";
	Rock3.header.frame_id = "odom";

	// Coordinates reference on Discord
	VisualizationNode::setCoordinates(&LightHouse, 120, -50);
	VisualizationNode::setCoordinates(&Island1, -152, -6);
	VisualizationNode::setCoordinates(&Island2, 110, 135);
	VisualizationNode::setCoordinates(&RockIsland0, 12, -102);
	VisualizationNode::setCoordinates(&RockIsland1, 92, 170);
	VisualizationNode::setCoordinates(&Rock0, -92, 176);
	VisualizationNode::setCoordinates(&Rock1, -40, 220);
	VisualizationNode::setCoordinates(&Rock2, -44, -95);
	VisualizationNode::setCoordinates(&Rock3, -30, -150);

	point_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),  // 10 Hz
        std::bind(&VisualizationNode::pointsPublisher, this));

	RCLCPP_INFO(this->get_logger(), "Visualization Node has started");
}

void VisualizationNode::setCoordinates(geometry_msgs::msg::PointStamped *Point, double x, double z)
{
	Point->point.set__x(x);
	Point->point.set__y(z);
	Point->point.set__z(0);
}

void VisualizationNode::pointsPublisher()
{
	light_house_publisher_->publish(LightHouse);
	island_1_publisher_->publish(Island1);
	island_2_publisher_->publish(Island2);
	rock_island_0_publisher_->publish(RockIsland0);
	rock_island_1_publisher_->publish(RockIsland1);
	rock_0_publisher_->publish(Rock0);
	rock_1_publisher_->publish(Rock1);
	rock_2_publisher_->publish(Rock2);
	rock_3_publisher_->publish(Rock3);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisualizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
