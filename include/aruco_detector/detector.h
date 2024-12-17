#ifndef ARUCO_DETECTOR_H
#define ARUCO_DETECTOR_H

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include "quaternion.hpp"

class ArucoDetector : public rclcpp::Node {
public:
    ArucoDetector();
    void initialize();

private:
    struct MarkerInfo {
        float x;
        float y;
        float size;
    };
    
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
    void loadCameraParameters(const std::string& filename);
    void loadMarkerInfo(const std::string& filename);
    void displayAndPublishImage(cv::Mat& frame);

    std::string camera_parameters_yaml_;
    std::string marker_info_yaml_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    std::unordered_map<int, MarkerInfo> marker_info_;

    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr landing_pad_position_pub_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Mat frame;

    std::string input_image_topic_;
    std::string output_image_topic_;
    std::string landing_pad_position_topic_;
    bool is_display_;

    // For some unknown reasons, the rotation vector from estimatePoseSingleMarkers is not correct
    // It is always flipped 180 degrees around the (-0.7071, -0.7071, 0) axis
    // This corrective quaternion is used to fix the rotation
    const Quaternion corrective_q_ = Quaternion{0, 0.7071, 0.7071, 0};    // w, x, y, z

    cv::Point2d pad_position_2d_norm_prev_ = cv::Point2d(-1, -1);
    const double increment_point2d_ = 0.0075;
    double height_prev_ = -1;
    const double increment_double_ = 0.025;
};

cv::Point2d incrementPoint2d(const cv::Point2d& previous, const cv::Point2d& current, double increment);
double incrementDouble(const double previous, const double current, double increment);

#endif // ARUCO_DETECTOR_H