#include "visualization/visualization_node.hpp"

VisualizationNode::VisualizationNode() : Node("visualization_node")
{
	Visual_Client_ = this->create_client<sensors::srv::TargetPositions>("mission/target_positions");

    // Subscriptions
    odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/mission/odometry", 10,
        std::bind(&VisualizationNode::odometryCallback, this, std::placeholders::_1));

	camera_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
		"/aquabot/thrusters/main_camera_sensor/pos", 10,
		std::bind(&VisualizationNode::cameraCallback, this, std::placeholders::_1));

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
	camera_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/minimap/camera", 10);
	actual_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/minimap/path/actual", 10);
    full_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/minimap/path/full", 10);
	polygon_publisher_1_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("minimap/polygon_1", 10);
	polygon_publisher_2_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("minimap/polygon_2", 10);
	polygon_publisher_3_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("minimap/polygon_3", 10);
	polygon_publisher_4_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("minimap/polygon_4", 10);
	polygon_publisher_5_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("minimap/polygon_5", 10);

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
	Camera.header.frame_id = "odom";
    FullPath.header.frame_id = "odom";
	ActualPath.header.frame_id = "odom";
	Polygon1.header.frame_id = "odom";
	Polygon2.header.frame_id = "odom";
	Polygon3.header.frame_id = "odom";
	Polygon4.header.frame_id = "odom";
	Polygon5.header.frame_id = "odom";

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

	VisualizationNode::addPolygonPoint(Polygon1, -122,206);
	VisualizationNode::addPolygonPoint(Polygon1, -76,255);
	VisualizationNode::addPolygonPoint(Polygon1, -32,260);
	VisualizationNode::addPolygonPoint(Polygon1, -15,215);
	VisualizationNode::addPolygonPoint(Polygon1, -45,190);
	VisualizationNode::addPolygonPoint(Polygon1, -62,146);
	VisualizationNode::addPolygonPoint(Polygon1, -122,146);

	VisualizationNode::addPolygonPoint(Polygon2, 100,-30);
	VisualizationNode::addPolygonPoint(Polygon2, 145, -30);
	VisualizationNode::addPolygonPoint(Polygon2, 145, -75);
	VisualizationNode::addPolygonPoint(Polygon2, 100, -75);

	VisualizationNode::addPolygonPoint(Polygon3, -190,33);
	VisualizationNode::addPolygonPoint(Polygon3, -120,30);
	VisualizationNode::addPolygonPoint(Polygon3, -127,-50);
	VisualizationNode::addPolygonPoint(Polygon3, -167,-54);

	VisualizationNode::addPolygonPoint(Polygon4, 42,165);
	VisualizationNode::addPolygonPoint(Polygon4, 100,210);
	VisualizationNode::addPolygonPoint(Polygon4, 153,135);
	VisualizationNode::addPolygonPoint(Polygon4, 105,90);

	VisualizationNode::addPolygonPoint(Polygon5, -67,-75);
	VisualizationNode::addPolygonPoint(Polygon5, -20,-75);
	VisualizationNode::addPolygonPoint(Polygon5, 23,-75);
	VisualizationNode::addPolygonPoint(Polygon5, 43,-110);
	VisualizationNode::addPolygonPoint(Polygon5, -5,-120);
	VisualizationNode::addPolygonPoint(Polygon5, -7,-170);
	VisualizationNode::addPolygonPoint(Polygon5, -60,-165);

	sleep(2);

	// Test path
	VisualizationNode::addPathPoint(FullPath, 0, 0);
	VisualizationNode::addPathPoint(FullPath, 219, 0);
	VisualizationNode::addPathPoint(FullPath, 219, 290);
	VisualizationNode::addPathPoint(FullPath, -233, 290);
	VisualizationNode::addPathPoint(FullPath, -233, 27);
	VisualizationNode::addPathPoint(FullPath, -270, -187);
	
	VisualizationNode::addPathPoint(ActualPath, 0, 0);
	VisualizationNode::addPathPoint(ActualPath, 600, -600);
	VisualizationNode::addPathPoint(ActualPath, 550, -550);
	VisualizationNode::addPathPoint(ActualPath, 300, -100);

	VisualizationNode::pointsPublisher();

	RCLCPP_INFO(this->get_logger(), "Visualization Node has started");
}

void VisualizationNode::addPolygonPoint(geometry_msgs::msg::PolygonStamped& Polygon, double x, double y)
{
	geometry_msgs::msg::Point32 Point32;

	Point32.x = x;
	Point32.y = y;
	Point32.z = 5;

	Polygon.polygon.points.push_back(Point32);
}

void VisualizationNode::addPathPoint(nav_msgs::msg::Path& path, double x, double y)
{
	geometry_msgs::msg::PoseStamped newPos;

	newPos.header.frame_id = "odom";
	newPos.pose.position.x = x;
	newPos.pose.position.y = y;
	newPos.pose.position.z = 0;

	path.poses.push_back(newPos);
}

void VisualizationNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	Camera.pose.pose.position.x = msg.get()->pose.pose.position.x;
	Camera.pose.pose.position.y = msg.get()->pose.pose.position.y;
	Camera.pose.pose.position.z = msg.get()->pose.pose.position.z;

	camera_publisher_->publish(Camera);

	geometry_msgs::msg::PoseStamped boatPos;
	boatPos.header.frame_id = "odom";
	boatPos.pose = msg.get()->pose.pose;

	ActualPath.poses.front().pose = msg.get()->pose.pose;
	actual_path_publisher_->publish(ActualPath);

	//geometry_msgs::msg::PoseStamped PoseStamped;
//
	//PoseStamped.pose.position.x = msg.get()->pose.pose.position.x;
	//PoseStamped.pose.position.y = msg.get()->pose.pose.position.y;
	//PoseStamped.pose.position.z = msg.get()->pose.pose.position.z;
//
	//Path.poses.push_back(PoseStamped);
//
	//path_publisher_->publish(Path);
}

void VisualizationNode::cameraCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    //Camera.pose.pose.orientation.z = -msg.get()->data;
//
    //camera_publisher->publish(Camera);

	   // Position de la cible vers laquelle la caméra doit faire face
    double target_x = 120 /* coordonnée x de la cible */;
    double target_y = -50 /* coordonnée y de la cible */;

    // Calculez la direction vers la cible
    double delta_x = target_x - Camera.pose.pose.position.x;
    double delta_y = target_y - Camera.pose.pose.position.y;

    // Calculez l'angle en radians
    double angle_radians = atan2(delta_y, delta_x);

    // Définir l'orientation de la caméra
    tf2::Quaternion q;
    q.setRPY(0, 0, angle_radians); // RPY : roll, pitch, yaw (ici, roll et pitch sont 0)

    // Appliquer la rotation
    Camera.pose.pose.orientation.x = q.x();
    Camera.pose.pose.orientation.y = q.y();
    Camera.pose.pose.orientation.z = q.z();
    Camera.pose.pose.orientation.w = q.w();

    // Publier la pose de la caméra
    camera_publisher_->publish(Camera);
}

void VisualizationNode::pointsPublisher()
{
	RCLCPP_INFO(this->get_logger(), "Publishing");

	light_house_publisher_->publish(LightHouse);
	island_1_publisher_->publish(Island1);
	island_2_publisher_->publish(Island2);
	rock_island_0_publisher_->publish(RockIsland0);
	rock_island_1_publisher_->publish(RockIsland1);
	rock_0_publisher_->publish(Rock0);
	rock_1_publisher_->publish(Rock1);
	rock_2_publisher_->publish(Rock2);
	rock_3_publisher_->publish(Rock3);

	full_path_publisher_->publish(FullPath);

	polygon_publisher_1_->publish(Polygon1);
	polygon_publisher_2_->publish(Polygon2);
	polygon_publisher_3_->publish(Polygon3);
	polygon_publisher_4_->publish(Polygon4);
	polygon_publisher_5_->publish(Polygon5);
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

       // geometry_msgs::msg::PoseStamped PoseStamped;
//
       // PoseStamped.header.frame_id = "odom";
       // PoseStamped.pose.position.set__x(msg.poses[i].position.x);
       // PoseStamped.pose.position.set__y(msg.poses[i].position.y);
       // PoseStamped.pose.position.set__z(msg.poses[i].position.z);
//
       // Path.poses.push_back(PoseStamped);
    }
}

geometry_msgs::msg::PoseArray MakeRequest(std::shared_ptr<VisualizationNode> node, 
	sensors::srv::TargetPositions::Request::SharedPtr request)
{
    //Send request to the service
    rclcpp::Client<sensors::srv::TargetPositions>::FutureAndRequestId future = node->Visual_Client_->async_send_request(request);
    //Wait until a response
    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) //If success
		return(future.get()->poses);
	else
		RCLCPP_ERROR(node->get_logger(), "Failed to call service target_positions");
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
