#ifndef CAMERA_PROCESSING_NODE_HPP
#define CAMERA_PROCESSING_NODE_HPP

// Camera processing node declarations
#include <rclcpp/rclcpp.hpp>
#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

class CameraProcessingNode : public rclcpp::Node
{
public:
    CameraProcessingNode();

private:
	void	scanQRCode(const sensor_msgs::msg::Image::SharedPtr msg);

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr	image_sub_;
};


#endif // CAMERA_PROCESSING_NODE_HPP
