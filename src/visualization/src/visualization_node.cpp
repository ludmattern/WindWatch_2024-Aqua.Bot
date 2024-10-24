// obstacle_avoidance_node.cpp

#include "visualization/visualization_node.hpp"

VisualizationNode::VisualizationNode() : Node("visualization_node")
{
	Visual_Client_ = this->create_client<sensors::srv::TargetPositions>("mission/target_positions");

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

	VisualizationNode::pointsPublisher();

	RCLCPP_INFO(this->get_logger(), "Visualization Node has started");
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

void VisualizationNode::setCoordinates(geometry_msgs::msg::PointStamped *Point, double x, double z)
{
	Point->point.set__x(x);
	Point->point.set__y(z);
	Point->point.set__z(0);
}

void VisualizationNode::VisualRegister(geometry_msgs::msg::PoseArray msg)
{
    for (int i = 0; i < msg.poses.size(); ++i)
	{
		this->_markers[i].header.frame_id = "odom";
		this->_markers[i].type = visualization_msgs::msg::Marker::CYLINDER;
		this->_markers[i].action = visualization_msgs::msg::Marker::ADD;

		this->_markers[i].pose.position.set__x(msg.poses[i].position.x);
		this->_markers[i].pose.position.set__y(msg.poses[i].position.y);
		this->_markers[i].pose.position.set__z(msg.poses[i].position.z);

		this->_markers[i].scale.x = 5;
		this->_markers[i].scale.y = 5;
		this->_markers[i].scale.z = 5;

		this->_markers[i].color.r = 0.5f;
		this->_markers[i].color.g = 0.5f;
		this->_markers[i].color.b = 0.5f;
		this->_markers[i].color.a = 1.0f;

		turbines_publisher_[i] = this->create_publisher<visualization_msgs::msg::Marker>(
			"/minimap/turbine_marker_" + std::to_string(i), 10);
		turbines_publisher_[i]->publish(this->_markers[i]);
    }
}

geometry_msgs::msg::PoseArray MakeRequest(std::shared_ptr<VisualizationNode> node, 
	sensors::srv::TargetPositions::Request::SharedPtr request)
{
    //Send request to the service
    rclcpp::Client<sensors::srv::TargetPositions>::FutureAndRequestId future = node->Visual_Client_->async_send_request(request);
    //Wait until a response
	//RCLCPP_INFO(node->get_logger(), "AA TEST");
    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) //If success
		return(future.get()->poses);
	else
		RCLCPP_ERROR(node->get_logger(), "Failed to call service target_positions");
    //RCLCPP_INFO(node->get_logger(), "AA TEST2");
	return (future.get()->poses);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisualizationNode>();

	// Log on the server
    sensors::srv::TargetPositions::Request::SharedPtr request = std::make_shared<sensors::srv::TargetPositions::Request>();
    while (node->Visual_Client_->wait_for_service(std::chrono::seconds(1)) == false)
    {
        if (rclcpp::ok() == false)
        {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return (1);
        }
    }
    geometry_msgs::msg::PoseArray TgtPos;
	while (TgtPos.poses.empty())
		TgtPos = MakeRequest(node, request);
	node->VisualRegister(TgtPos);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
