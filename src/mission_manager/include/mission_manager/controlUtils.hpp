// ControlUtils.hpp

#ifndef CONTROLUTILS_HPP
#define CONTROLUTILS_HPP

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_msgs/msg/odometry.hpp"

#define SIMPLE_CMD 99.99
#define PRECISE_CMD 100.100

namespace controlUtils {

	struct OdometryData
	{
		double pos_x;
		double pos_y;
		double pos_z;
		double yaw;
		double linear_velocity;
		double distanceToTarget;
	};

	OdometryData getOdometryData(const geometry_msgs::msg::PoseStamped& target, const nav_msgs::msg::Odometry& current_odometry_);

	void sendThrustersCommands(double speedOutput, double headingOutput, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdPublisher_);

	double getTgtAngleError(const OdometryData& odometryData, const geometry_msgs::msg::PoseStamped& target);

	geometry_msgs::msg::Pose ClosestPointOnOrbit(const OdometryData& odometryData, const geometry_msgs::msg::PoseStamped& target, double orbitRadius);

	double OrbitHeadingAdjustment(const OdometryData& odometryData, double angle_to_target, double distanceError, double orbitRadius);
	
	double calculateDistance(const OdometryData& odometryData, const geometry_msgs::msg::PoseStamped& target);

	double normalizeAngle(double angle);

} // namespace controlUtils

#endif // CONTROLUTILS_HPP
