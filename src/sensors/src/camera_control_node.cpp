#include "sensors/camera_control_node.hpp"

CameraControlNode::CameraControlNode() : Node("camera_control_node")
{
	camera_pub_ = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/main_camera_sensor/pos", 10);
	
	// Subscriber pour la position et orientation du bateau via nav_msgs::msg::Odometry
	boat_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"/mission/odometry", 10, std::bind(&CameraControlNode::boatPoseCallback, this, std::placeholders::_1));
	
	// Subscriber pour la position du point à suivre (cela reste un PoseStamped)
	// point_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
	// "/target/pose", 10, std::bind(&CameraControlNode::targetPoseCallback, this, std::placeholders::_1));
	
	RCLCPP_INFO(this->get_logger(), "Camera Control Node has started");
}

void CameraControlNode::boatPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Récupérer la position du bateau
	boat_position_ = msg->pose.pose.position;

	// Récupérer l'orientation du bateau sous forme de quaternion
	auto orientation = msg->pose.pose.orientation;

	// Convertir le quaternion en angles de roulis, tangage, lacet (RPY)
	tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
	tf2::Matrix3x3(q).getRPY(roll_, pitch_, yaw_);  // Ici, on récupère surtout le yaw (lacet)

	// a virer 
	this->targetPoseCallback();
}

void CameraControlNode::targetPoseCallback(void)
{
	// Calculer le vecteur directionnel vers le point à fixer
    double dx = 219.19 - boat_position_.x;
    double dy = 290.79 - boat_position_.y;

    // Calculer l'angle entre l'axe X global et le point cible
    double theta = atan2(dy, dx);

    // Calculer l'angle relatif à l'avant du bateau
    double theta_relative = theta - yaw_;

    if (theta_relative - this->previous_theta_ > M_PI) {
        // Si la différence est trop grande, ajuster en ajoutant 2 * PI pour permettre la continuité
        theta_relative -= 2 * M_PI;
    }
    else if (this->previous_theta_ - theta_relative > M_PI) {
        // Si la différence est trop petite, ajuster en ajoutant 2 * PI pour permettre la continuité
        theta_relative += 2 * M_PI;
    }

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

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<CameraControlNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
