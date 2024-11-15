#include "visualization/visualization_node.hpp"

std::shared_ptr<cv::Mat> background = std::make_shared<cv::Mat>(cv::Mat::zeros(MINIMAP_SIZE, MINIMAP_SIZE, CV_8UC3));

VisualizationNode::VisualizationNode() : Node("visualization_node"), _center(MINIMAP_SIZE / 2), _camAngle(0), _nb_turbines(0)
{
	Visual_Client_ = this->create_client<navigation::srv::Path>("/navigation/path");

	if (background->empty()) {
    	RCLCPP_ERROR(this->get_logger(), "Background image is empty.");
    	return;
	}

	// Request to server to get Turbines and Path
	timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&VisualizationNode::launch, this));

	// Subscriptions
	odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"/mission/odometry", 10,
		std::bind(&VisualizationNode::odometryCallback, this, std::placeholders::_1));

	camera_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
		"/aquabot/thrusters/main_camera_sensor/pos", 10,
		std::bind(&VisualizationNode::cameraCallback, this, std::placeholders::_1));

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

	VisualizationNode::createPolygon(*background, 7,
		-122, 206,
		-76, 255,
		-32, 260,
		-15, 215,	
		-45, 190,
		-62, 146,
		-122, 146);

	VisualizationNode::createPolygon(*background, 4,
		100, -30,
		145, -30,
		145, -75,
		100, -75);

	VisualizationNode::createPolygon(*background, 4,
		-190, 33 + 20,
		-120, 30 + 20,
		-127, -50 + 20,
		-167, -54 + 20);

	VisualizationNode::createPolygon(*background, 4,
		42, 165,
		100, 210,
		153, 135,
		105, 90);

	VisualizationNode::createPolygon(*background, 7,
		-67, -75,
		-20, -75,
		23, -75,
		43, -110,	
		-5, -120,
		-7, -170,
		-60, -165);
	
	VisualizationNode::addPathPoint(ActualPath, 0, 0);
	VisualizationNode::addPathPoint(ActualPath, 219, 290);
	VisualizationNode::addPathPoint(ActualPath, 400, 600);
	VisualizationNode::addPathPoint(ActualPath, -300, 100);

	RCLCPP_INFO(this->get_logger(), "Visualization Node has started");
}

void VisualizationNode::launch()
{
	RCLCPP_INFO(this->get_logger(), "VisualizationNode Launch");

    if (!this->Visual_Client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_ERROR(this->get_logger(), "Service '/navigation/path' not available");
        return;
    }
	timer_->cancel();

    RCLCPP_INFO(this->get_logger(), "Service is available. Sending request...");

    auto request = std::make_shared<navigation::srv::Path::Request>();
    auto future = this->Visual_Client_->async_send_request(
        request,
        std::bind(&VisualizationNode::service_response_callback, this, std::placeholders::_1)
    );
}

void VisualizationNode::service_response_callback(
    rclcpp::Client<navigation::srv::Path>::SharedFuture future)
{
    RCLCPP_INFO(this->get_logger(), "Received response from service (Visualization)");

    auto response = future.get();

    if (response->path.poses.empty() || response->pose_array.poses.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Received empty poses from service. Retrying...");
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&VisualizationNode::launch, this));
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Processing received path");
        this->PathPlan(response->path);
        if (_nb_turbines == 0)
            this->VisualRegister(response->pose_array);
    }
}

void VisualizationNode::PathPlan(nav_msgs::msg::Path path)
{
    if (path.poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received message without poses.");
        return;
    }

    for (size_t i = 0; i < path.poses.size(); ++i)
		VisualizationNode::addPathPoint(FullPath, path.poses[i].pose.position.x, path.poses[i].pose.position.y);

	VisualizationNode::createLine(*background, 0, 0, FullPath.poses[0].pose.position.x, FullPath.poses[0].pose.position.y);
	for (size_t i = 1; i < this->FullPath.poses.size(); i++)
		VisualizationNode::createLine(*background, FullPath.poses[i - 1].pose.position.x, FullPath.poses[i - 1].pose.position.y,
			FullPath.poses[i].pose.position.x, FullPath.poses[i].pose.position.y);
}

void VisualizationNode::VisualRegister(geometry_msgs::msg::PoseArray msg)
{
	if (msg.poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received message without poses.");
        return;
    }

	for (int i = 0; i < msg.poses.size(); ++i)
		cv::circle((*background), cv::Point(msg.poses[i].position.x + _center, (*background).rows - (msg.poses[i].position.y + _center)),
			5, cv::Scalar(125, 125, 125), -1);

	_nb_turbines = static_cast<int>(msg.poses.size());
}

void VisualizationNode::createCircle(cv::Mat &mat, double x, double y, int radius)
{
	cv::circle(mat, cv::Point(x + _center, mat.rows - (y + _center)), radius - 5, cv::Scalar(252, 192, 15), -1);
}

void VisualizationNode::createLine(cv::Mat &mat, double x1, double y1, double x2, double y2)
{
	cv::Point2f start(x1 + _center, mat.rows - (y1 + _center));
	cv::Point2f end(x2 + _center, mat.rows - (y2 + _center));

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
		points.push_back(cv::Point(x + _center, (*background).rows - (y + _center)));
	}
	cv::polylines((*background), points, true, cv::Scalar(255, 255, 255));
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

	if (minimap.empty()) {
    	RCLCPP_ERROR(this->get_logger(), "Minimap image is empty after cloning background.");
    	return;
	}
	// Calcul de l'angle (yaw) du bateau
	double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
							msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
								msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);

	double boatAngle = std::atan2(siny_cosp, cosy_cosp); // Yaw en radians

	// Position du bateau sur la minimap
	int dx = static_cast<int>(msg->pose.pose.position.x + _center);
	int dy = static_cast<int>(minimap.rows - (msg->pose.pose.position.y + _center));

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

    // Longueur du triangle (distance entre le bateau et la base du FOV)
    double triangleLength = 20.0;

    // Angle du demi-champ de vision
    double halfFOV = CV_PI / 10;

    // Pointe du triangle (au niveau du bateau)
    cv::Point2f tipPoint = boatFront;

    // Calcul des points de la base en décalant l'angle de `+halfFOV` et `-halfFOV`
    cv::Point2f basePoint1(
        boatCenter.x + triangleLength * cos(effectiveCamAngle + halfFOV),
        boatCenter.y - triangleLength * sin(effectiveCamAngle + halfFOV)
    );

    cv::Point2f basePoint2(
        boatCenter.x + triangleLength * cos(effectiveCamAngle - halfFOV),
        boatCenter.y - triangleLength * sin(effectiveCamAngle - halfFOV)
    );

    // Dessiner le triangle sur un calque transparent
    cv::Mat overlay;
    minimap.copyTo(overlay);

    // Couleur du triangle (BGR)
    cv::Scalar triangleColor(0, 0, 255); // Rouge

    // Dessiner le triangle sur le calque
    std::vector<cv::Point> cameraTrianglePoints = {tipPoint, basePoint1, basePoint2};
    cv::fillConvexPoly(overlay, cameraTrianglePoints, triangleColor);

    // Fusionner le calque avec l'image de base en utilisant la transparence
    double alpha = 0.5;
    cv::addWeighted(overlay, alpha, minimap, 1 - alpha, 0, minimap);

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

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<VisualizationNode>();
	rclcpp::spin(node);
	cv::destroyAllWindows();
	rclcpp::shutdown();
	return 0;
}
