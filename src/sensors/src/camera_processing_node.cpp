#include "sensors/camera_processing_node.hpp"

CameraProcessingNode::CameraProcessingNode() : Node("camera_processing_node")
{
	
	RCLCPP_INFO(this->get_logger(), "Camera Processing Node has started");

	image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
		"/aquabot/sensors/cameras/main_camera_sensor/optical/image_raw" \
		, 10, std::bind(&CameraProcessingNode::scanQRCode, this, std::placeholders::_1));
}

void CameraProcessingNode::scanQRCode(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat img = cv_ptr->image;

    if (img.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Empty image received");
        return;
    }

    if (img.cols < 100 || img.rows < 100) {
        RCLCPP_WARN(this->get_logger(), "Image size too small for QR code detection");
        return;
    }

    // Convertir en niveaux de gris
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    cv::QRCodeDetector qrDecoder;
    cv::Mat bbox, rectifiedImage;
    std::string decodedText = qrDecoder.detectAndDecode(gray, bbox, rectifiedImage);

    if (!decodedText.empty()) {
        RCLCPP_INFO(this->get_logger(), "QR code text : %s", decodedText.c_str());
        for (int i = 0; i < bbox.rows; i++) {
            cv::line(img, cv::Point2i(bbox.at<float>(i, 0), bbox.at<float>(i, 1)),
                     cv::Point2i(bbox.at<float>((i + 1) % bbox.rows, 0), bbox.at<float>((i + 1) % bbox.rows, 1)),
                     cv::Scalar(255, 0, 0), 2);
        }
        if (!rectifiedImage.empty()) {
            cv::imshow("Rectified QR code", rectifiedImage);
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "No QR code detected");
    }

    cv::imshow("Image with QR code", img);
    cv::waitKey(1);  // Changez à 1 pour une mise à jour continue
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<CameraProcessingNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
