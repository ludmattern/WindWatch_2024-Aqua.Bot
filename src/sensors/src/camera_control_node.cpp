#include "sensors/camera_control_node.hpp"

CameraControlNode::CameraControlNode() : Node("camera_control_node"), targetProcessed_(false), previous_theta_(0)
{
	camera_pub_ = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/main_camera_sensor/pos", 10);
	
	// Subscriber pour la position et orientation du bateau via nav_msgs::msg::Odometry
	
	// Subscriber pour la position du point à suivre (cela reste un PoseStamped)
	// point_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
	// "/target/pose", 10, std::bind(&CameraControlNode::targetPoseCallback, this, std::placeholders::_1));

	CameraControlService_ = this->create_service<sensors::srv::CameraControlServ>(
		"mission/camera_control", std::bind(&CameraControlNode::ServerCallback, this, std::placeholders::_1, std::placeholders::_2));
	
	RCLCPP_INFO(this->get_logger(), "Camera Control Node has started");
}

void CameraControlNode::ServerCallback(const std::shared_ptr<sensors::srv::CameraControlServ::Request> request,
	const std::shared_ptr<sensors::srv::CameraControlServ::Response> response)
{
	currentTarget_ = request->target;

	RCLCPP_INFO(this->get_logger(), "Received target: x=%f, y=%f", currentTarget_.x, currentTarget_.y);

	odometrySub_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"/mission/odometry", 10, std::bind(&CameraControlNode::boatPoseCallback, this, std::placeholders::_1));

	ImageFeedSub_ = this->create_subscription<sensor_msgs::msg::Image>(
		"/aquabot/sensors/cameras/main_camera_sensor/optical/image_raw", 10, 
		std::bind(&CameraControlNode::scanQRCode, this, std::placeholders::_1));

	// mutex on target processed
    std::unique_lock<std::mutex> lock(targetMutex_);
    targetCondition_.wait(lock, [this] { return targetProcessed_; });

	odometrySub_.reset();
	ImageFeedSub_.reset();

	// Publish the QR code data
	response->orientation = orientation_;
	response->qrcodedata = QRCodeData_;
	response->id = id_;
	response->state = state_;

	// Reset the target processed flag
	targetProcessed_ = false;
	previous_theta_ = 0;
	controlCamera(0.0);
}

void CameraControlNode::boatPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	boat_position_ = msg->pose.pose.position;
	auto orientation = msg->pose.pose.orientation;

	// Convertir le quaternion en angles de roulis, tangage, lacet (RPY)
	tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
	tf2::Matrix3x3(q).getRPY(roll_, pitch_, yaw_);  // Ici, on récupère surtout le yaw (lacet)

	targetPoseCallback();
}

void CameraControlNode::targetPoseCallback(void)
{
	// Calculer le vecteur directionnel vers le point à fixer
	double dx = currentTarget_.x - boat_position_.x;
	double dy = currentTarget_.y - boat_position_.y;

	// Calculer l'angle entre l'axe X global et le point cible
	double theta = atan2(dy, dx);

	// Calculer l'angle relatif à l'avant du bateau
	double theta_relative = theta - yaw_;

	if (theta_relative - this->previous_theta_ > M_PI)
		theta_relative -= 2 * M_PI;
	else if (this->previous_theta_ - theta_relative > M_PI)
		theta_relative += 2 * M_PI;

	// Mettre à jour la valeur précédente de l'angle
	previous_theta_ = theta_relative;

	// Ici, orienter la caméra en fonction de theta_relative
	controlCamera(theta_relative);
}

void CameraControlNode::controlCamera(double angle_in_radians)
{
	auto camera_msg = std_msgs::msg::Float64();
	camera_msg.data = angle_in_radians;
	camera_pub_->publish(camera_msg);
}

void CameraControlNode::scanQRCode(const sensor_msgs::msg::Image::SharedPtr msg)
{
	if (targetProcessed_)
		return;

	cv_bridge::CvImagePtr cv_ptr;
	try 
	{
		cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
	}
	catch (cv_bridge::Exception& e)
	{
		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
		return;
	}

	cv::Mat img = cv_ptr->image;

	if (img.empty()) {
		RCLCPP_ERROR(this->get_logger(), "Empty image received");
		return;
	}

	// Calculer les dimensions de la région rognée (1/3 de l'image)
	int croppedWidth = img.cols / 3;
	int croppedHeight = img.rows / 3;

	// Calculer les coordonnées du coin supérieur gauche pour centrer le rectangle
	int x = (img.cols - croppedWidth) / 2;
	int y = (img.rows - croppedHeight) / 2;

	// Définir la région d'intérêt (ROI) centrée
	cv::Rect roi(x, y, croppedWidth, croppedHeight);

	// Rogner l'image en utilisant la ROI
	cv::Mat croppedImage = img(roi);

	// Convertir en niveaux de gris, car ZBar fonctionne mieux avec les images en noir et blanc
	cv::Mat gray;
	cv::cvtColor(croppedImage, gray, cv::COLOR_BGR2GRAY);

	// Initialiser le scanner ZBar
	zbar::ImageScanner scanner;
	scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

	// Convertir l'image OpenCV en image ZBar
	zbar::Image zbarImage(gray.cols, gray.rows, "Y800", gray.data, gray.cols * gray.rows);

	// Scanner l'image pour détecter les QR codes
	int n = scanner.scan(zbarImage);

	if (n > 0)
	{
		auto symbol = zbarImage.symbol_begin();
		std::string decodedText = symbol->get_data();
		RCLCPP_INFO(this->get_logger(), "QR code text : %s", decodedText.c_str());

		QRCodeData_.data = decodedText;
		orientation_.data = 0; //temporaire
		id_.data = std::stoi(QRCodeData_.data.substr(QRCodeData_.data.find_first_of(DIGITS)));
		state_.data = (QRCodeData_.data.find("KO") == std::string::npos);

		{
			std::lock_guard<std::mutex> lock(targetMutex_);
			targetProcessed_ = true;
		}

		targetCondition_.notify_one();
	}
	// if (n > 0) {
	//     for (auto symbol = zbarImage.symbol_begin(); symbol != zbarImage.symbol_end(); ++symbol) {
	//         RCLCPP_INFO(this->get_logger(), "QR code text : %s", decodedText.c_str());

	//         // Dessiner le contour du QR code sur l'image
	//         for (int i = 0; i < symbol->get_location_size(); i++) {
	//             cv::line(
	//                 croppedImage,
	//                 cv::Point(symbol->get_location_x(i), symbol->get_location_y(i)),
	//                 cv::Point(symbol->get_location_x((i + 1) % symbol->get_location_size()), symbol->get_location_y((i + 1) % symbol->get_location_size())),
	//                 cv::Scalar(255, 0, 0), 2
	//             );
	//         }
	//     }
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<CameraControlNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
