#include <aruco_detector/detector.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoDetector>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}