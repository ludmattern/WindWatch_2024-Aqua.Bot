#include "mission_manager/controlUtils.hpp"

controlUtils::OdometryData controlUtils::getOdometryData(const geometry_msgs::msg::PoseStamped & target, const nav_msgs::msg::Odometry & current_odometry_)
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

double controlUtils::getTgtAngleError(const OdometryData & odometryData, const geometry_msgs::msg::PoseStamped & target)
{
	double targetAngleError = std::atan2(
		target.pose.position.y - odometryData.pos_y,
		target.pose.position.x - odometryData.pos_x);

	double angleError = targetAngleError - odometryData.yaw;
    while (angleError > M_PI) angleError -= 2 * M_PI;
    while (angleError < -M_PI) angleError += 2 * M_PI;

	return angleError;
}

void controlUtils::sendThrustersCommands(double speedOutput, double headingOutput, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdPublisher_)
{
	geometry_msgs::msg::Twist cmdMsg;
	cmdMsg.linear.x = speedOutput;
	cmdMsg.angular.z = headingOutput;
	cmdPublisher_->publish(cmdMsg);
}