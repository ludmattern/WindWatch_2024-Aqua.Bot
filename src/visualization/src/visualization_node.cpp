#include "visualization/visualization_node.hpp"

std::shared_ptr<cv::Mat> background = std::make_shared<cv::Mat>(cv::Mat::zeros(MINIMAP_SIZE, MINIMAP_SIZE, CV_8UC3));
int center = MINIMAP_SIZE / 2;

VisualizationNode::VisualizationNode() : Node("visualization_node")
{
	Visual_Client_ = this->create_client<sensors::srv::TargetPositions>("mission/target_positions");

	_camAngle = 0;

	// Subscriptions
	odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"/mission/odometry", 10,
		std::bind(&VisualizationNode::odometryCallback, this, std::placeholders::_1));

	camera_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
		"/aquabot/thrusters/main_camera_sensor/pos", 10,
		std::bind(&VisualizationNode::cameraCallback, this, std::placeholders::_1));

	// Coordinates reference on Discord

	VisualizationNode::createCircle(*background, 120, -50, 35);
	VisualizationNode::createCircle(*background, -150, 0, 32);
	VisualizationNode::createCircle(*background, -150, 32, 32);
	VisualizationNode::createCircle(*background, 110, 130, 32);
	VisualizationNode::createCircle(*background, 110, 162, 32);
	VisualizationNode::createCircle(*background, 12, -102, 30);
	VisualizationNode::createCircle(*background, 92, 170, 30);
	VisualizationNode::createCircle(*background, -88, 180, 40);
	VisualizationNode::createCircle(*background, -40, 220, 32);
	VisualizationNode::createCircle(*background, -48, -92, 32);
	VisualizationNode::createCircle(*background, -30, -150, 32);

	VisualizationNode::addPolygonPoint(Polygon1, -122, 206);
	VisualizationNode::addPolygonPoint(Polygon1, -76, 255);
	VisualizationNode::addPolygonPoint(Polygon1, -32, 260);
	VisualizationNode::addPolygonPoint(Polygon1, -15, 215);
	VisualizationNode::addPolygonPoint(Polygon1, -45, 190);
	VisualizationNode::addPolygonPoint(Polygon1, -62, 146);
	VisualizationNode::addPolygonPoint(Polygon1, -122, 146);

	VisualizationNode::addPolygonPoint(Polygon2, 100, -30);
	VisualizationNode::addPolygonPoint(Polygon2, 145, -30);
	VisualizationNode::addPolygonPoint(Polygon2, 145, -75);
	VisualizationNode::addPolygonPoint(Polygon2, 100, -75);

	VisualizationNode::addPolygonPoint(Polygon3, -190, 33);
	VisualizationNode::addPolygonPoint(Polygon3, -120, 30);
	VisualizationNode::addPolygonPoint(Polygon3, -127, -50);
	VisualizationNode::addPolygonPoint(Polygon3, -167, -54);

	VisualizationNode::addPolygonPoint(Polygon4, 42, 165);
	VisualizationNode::addPolygonPoint(Polygon4, 100, 210);
	VisualizationNode::addPolygonPoint(Polygon4, 153, 135);
	VisualizationNode::addPolygonPoint(Polygon4, 105, 90);

	VisualizationNode::addPolygonPoint(Polygon5, -67, -75);
	VisualizationNode::addPolygonPoint(Polygon5, -20, -75);
	VisualizationNode::addPolygonPoint(Polygon5, 23, -75);
	VisualizationNode::addPolygonPoint(Polygon5, 43, -110);
	VisualizationNode::addPolygonPoint(Polygon5, -5, -120);
	VisualizationNode::addPolygonPoint(Polygon5, -7, -170);
	VisualizationNode::addPolygonPoint(Polygon5, -60, -165);

	sleep(2);

	// Test path
	VisualizationNode::addPathPoint(FullPath, 0, 0);
	VisualizationNode::addPathPoint(FullPath, 219, 0);
	VisualizationNode::addPathPoint(FullPath, 219, 290);
	VisualizationNode::addPathPoint(FullPath, -233, 290);
	VisualizationNode::addPathPoint(FullPath, -233, 27);
	VisualizationNode::addPathPoint(FullPath, -270, -187);

	VisualizationNode::createLine(*background, 0, 0, FullPath.poses[0].pose.position.x, FullPath.poses[0].pose.position.y);
	for (size_t i = 1; i < this->FullPath.poses.size(); i++)
		VisualizationNode::createLine(*background, FullPath.poses[i - 1].pose.position.x, FullPath.poses[i - 1].pose.position.y,
									FullPath.poses[i].pose.position.x, FullPath.poses[i].pose.position.y);
	
	VisualizationNode::addPathPoint(ActualPath, 0, 0);
	VisualizationNode::addPathPoint(ActualPath, 219, 290);
	VisualizationNode::addPathPoint(ActualPath, 400, 600);
	VisualizationNode::addPathPoint(ActualPath, -300, 100);

	RCLCPP_INFO(this->get_logger(), "Visualization Node has started");
}

void VisualizationNode::createCircle(cv::Mat &mat, double x, double y, int radius)
{
	cv::circle(mat, cv::Point(x + center, mat.rows - (y + center)), radius, cv::Scalar(252, 192, 15), -1);
}

void VisualizationNode::createLine(cv::Mat &mat, double x1, double y1, double x2, double y2)
{
	cv::Point2f start(x1 + center, mat.rows - (y1 + center));
	cv::Point2f end(x2 + center, mat.rows - (y2 + center));

	cv::line(mat, start, end, cv::Scalar(50, 50, 50), 2);
}

void VisualizationNode::createPolygon(cv::Mat &mat, size_t pointsNumber, ...)
{
	std::vector<cv::Point> points;
	va_list args;

	va_start(args, pointsNumber);

	for (size_t i = 0; i < pointsNumber; i++)
	{
		int x = va_arg(args, int);
		int y = va_arg(args, int);
		points.emplace_back();
	}
}

void VisualizationNode::addPolygonPoint(geometry_msgs::msg::PolygonStamped &Polygon, double x, double y)
{
	geometry_msgs::msg::Point32 Point32;

	Point32.x = x;
	Point32.y = y;
	Point32.z = 5;

	Polygon.polygon.points.push_back(Point32);
}

void VisualizationNode::addPathPoint(nav_msgs::msg::Path &path, double x, double y)
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
	cv::Mat minimap = (*background).clone();

	// Calcul de l'angle (yaw) du bateau
	double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
							msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
								msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);

	double boatAngle = std::atan2(siny_cosp, cosy_cosp); // Yaw en radians

	// Position du bateau sur la minimap
	int dx = static_cast<int>(msg->pose.pose.position.x + center);
	int dy = static_cast<int>(minimap.rows - (msg->pose.pose.position.y + center));

	int arrowLength = 18;
	cv::Point2f boatCenter(dx, dy);

	// Dessiner la flèche du bateau
	cv::Point2f boatFront(boatCenter.x + (arrowLength / 2) * cos(boatAngle),
						  boatCenter.y - (arrowLength / 2) * sin(boatAngle));
	cv::Point2f boatBack(boatCenter.x - (arrowLength / 2) * cos(boatAngle),
						 boatCenter.y + (arrowLength / 2) * sin(boatAngle));

	cv::arrowedLine(minimap, boatBack, boatFront, cv::Scalar(0, 255, 0), 2, cv::LINE_4, 0, 0.25);

	// Ajouter la rotation du bateau à l'angle de la caméra
	double effectiveCamAngle = _camAngle + boatAngle;

	// Calcul des sommets du triangle représentant la caméra
	int triangleLength = 18;
	cv::Point2f cameraPoint1(boatCenter.x + triangleLength * cos(effectiveCamAngle),
							 boatCenter.y - triangleLength * sin(effectiveCamAngle));
	cv::Point2f cameraPoint2(boatCenter.x + (triangleLength / 2) * cos(effectiveCamAngle + CV_PI * 2 / 3),
							 boatCenter.y - (triangleLength / 2) * sin(effectiveCamAngle + CV_PI * 2 / 3));
	cv::Point2f cameraPoint3(boatCenter.x + (triangleLength / 2) * cos(effectiveCamAngle - CV_PI * 2 / 3),
							 boatCenter.y - (triangleLength / 2) * sin(effectiveCamAngle - CV_PI * 2 / 3));

	// Dessiner le triangle pour représenter la caméra
	std::vector<cv::Point> cameraTrianglePoints = {cameraPoint1, cameraPoint2, cameraPoint3};
	cv::fillConvexPoly(minimap, cameraTrianglePoints, cv::Scalar(0, 0, 255));

	// Afficher la minimap
	cv::imshow("Minimap", minimap);
	cv::waitKey(1);
}

void VisualizationNode::cameraCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
	_camAngle = msg.get()->data;
}

void VisualizationNode::wayPointCallback()
{
	ActualPath.poses.erase(ActualPath.poses.begin() + 1);
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
		cv::circle((*background), cv::Point(msg.poses[i].position.x + center, (*background).rows - (msg.poses[i].position.y + center)), 5, cv::Scalar(125, 125, 125), -1);
}

geometry_msgs::msg::PoseArray MakeRequest(std::shared_ptr<VisualizationNode> node,
										  sensors::srv::TargetPositions::Request::SharedPtr request)
{
	// Send request to the service
	rclcpp::Client<sensors::srv::TargetPositions>::FutureAndRequestId future = node->Visual_Client_->async_send_request(request);
	// Wait until a response
	if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) // If success
		return (future.get()->poses);
	else
		RCLCPP_ERROR(node->get_logger(), "Failed to call service target_positions");
	return (future.get()->poses);
}

int main(int argc, char *argv[])
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

	cv::destroyAllWindows();
	rclcpp::shutdown();
	return 0;
}
