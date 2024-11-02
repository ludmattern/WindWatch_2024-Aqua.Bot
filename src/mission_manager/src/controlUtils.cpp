#include "mission_manager/controlUtils.hpp"

namespace controlUtils
{

OdometryData getOdometryData(const geometry_msgs::msg::PoseStamped & target, const nav_msgs::msg::Odometry & current_odometry_)
{
	nav_msgs::msg::Odometry odom = current_odometry_;

	OdometryData data;
	data.pos_x = odom.pose.pose.position.x;
	data.pos_y = odom.pose.pose.position.y;

	tf2::Quaternion q(
    odom.pose.pose.orientation.x,
    odom.pose.pose.orientation.y,
    odom.pose.pose.orientation.z,
    odom.pose.pose.orientation.w);

	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;

	m.getRPY(roll, pitch, yaw);
	data.yaw = yaw;

	data.linear_velocity = std::sqrt(
		std::pow(odom.twist.twist.linear.x, 2) +
		std::pow(odom.twist.twist.linear.y, 2));

	data.distanceToTarget = std::sqrt(
		std::pow(target.pose.position.x - data.pos_x, 2) +
		std::pow(target.pose.position.y - data.pos_y, 2));

	return data;
}

double getTgtAngleError(const OdometryData & odometryData, const geometry_msgs::msg::PoseStamped & target)
{
	double targetAngleError = std::atan2(
		target.pose.position.y - odometryData.pos_y,
		target.pose.position.x - odometryData.pos_x);

	double angleError = targetAngleError - odometryData.yaw;
    while (angleError > M_PI) angleError -= 2 * M_PI;
    while (angleError < -M_PI) angleError += 2 * M_PI;

	return angleError;
}

void sendThrustersCommands(double speedOutput, double headingOutput, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdPublisher_)
{
	geometry_msgs::msg::Twist cmdMsg;
	cmdMsg.linear.x = speedOutput;
	cmdMsg.angular.z = headingOutput;
	cmdPublisher_->publish(cmdMsg);
}

geometry_msgs::msg::Pose ClosestPointOnOrbit(const OdometryData& odometryData, const geometry_msgs::msg::PoseStamped& target, double orbitRadius)
{
	double angle_to_target = std::atan2(odometryData.pos_y - target.pose.position.y, odometryData.pos_x - target.pose.position.x);

	geometry_msgs::msg::Pose entry_point;
	entry_point.position.x = target.pose.position.x + orbitRadius * std::cos(angle_to_target);
	entry_point.position.y = target.pose.position.y + orbitRadius * std::sin(angle_to_target);

	return entry_point;
}

double OrbitHeadingAdjustment(const OdometryData& odometryData, double angle_to_target, double distanceError, double orbitRadius)
{
    double angle_adjustment = std::atan2(distanceError, orbitRadius);
    double orbit_angle_error = angle_to_target + M_PI / 2.0 - angle_adjustment;

    while (orbit_angle_error > M_PI) orbit_angle_error -= 2.0 * M_PI;
    while (orbit_angle_error < -M_PI) orbit_angle_error += 2.0 * M_PI;

    return orbit_angle_error;
}

double calculateDistance(const OdometryData& odometryData, const geometry_msgs::msg::PoseStamped& target) {
    double dx = target.pose.position.x - odometryData.pos_x;
    double dy = target.pose.position.y - odometryData.pos_y;
    return std::sqrt(dx * dx + dy * dy);
}

} // namespace controlUtils