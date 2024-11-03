#include "sensors/camera_processing_node.hpp"
#include <zbar.h>

CameraProcessingNode::CameraProcessingNode() : Node("camera_processing_node")
{
    RCLCPP_INFO(this->get_logger(), "Camera Processing Node has started");

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/aquabot/sensors/cameras/main_camera_sensor/optical/image_raw", 
        10, 
        std::bind(&CameraProcessingNode::scanQRCode, this, std::placeholders::_1)
    );
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

    // Convertir en niveaux de gris, car ZBar fonctionne mieux avec les images en noir et blanc
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    // Initialiser le scanner ZBar
    zbar::ImageScanner scanner;
    scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

    // Convertir l'image OpenCV en image ZBar
    zbar::Image zbarImage(gray.cols, gray.rows, "Y800", gray.data, gray.cols * gray.rows);

    // Scanner l'image pour détecter les QR codes
    int n = scanner.scan(zbarImage);

    if (n > 0) {
        for (auto symbol = zbarImage.symbol_begin(); symbol != zbarImage.symbol_end(); ++symbol) {
            std::string decodedText = symbol->get_data();
            RCLCPP_INFO(this->get_logger(), "QR code text : %s", decodedText.c_str());

            // Dessiner le contour du QR code sur l'image
            for (int i = 0; i < symbol->get_location_size(); i++) {
                cv::line(
                    img,
                    cv::Point(symbol->get_location_x(i), symbol->get_location_y(i)),
                    cv::Point(symbol->get_location_x((i + 1) % symbol->get_location_size()), symbol->get_location_y((i + 1) % symbol->get_location_size())),
                    cv::Scalar(255, 0, 0), 2
                );
            }
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
