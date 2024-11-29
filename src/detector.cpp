#include <fiducial_detector/detector.h>


ArucoDetector::ArucoDetector() : Node("aruco_detector"), it_(node_) {
    // Declare parameters
    this->declare_parameter<std::string>("input_image_topic", "/camera/image_raw");
    this->declare_parameter<std::string>("output_image_topic", "/camera/image_processed");
    this->declare_parameter<std::string>("bbox_pub_topic", "/aruco/bbox");
    this->declare_parameter<bool>("is_display", true);
    this->declare_parameter<std::string>("camera_parameters_yaml", "camera_params.yaml");
    this->declare_parameter<std::string>("marker_info_yaml", "marker_info.yaml");

    // Get parameters
    this->get_parameter("input_image_topic", input_image_topic_);
    this->get_parameter("output_image_topic", output_image_topic_);
    this->get_parameter("bbox_pub_topic", bbox_pub_topic_);
    this->get_parameter("is_display", is_display_);
    this->get_parameter("camera_parameters_yaml", camera_parameters_yaml_);
    this->get_parameter("marker_info_yaml", marker_info_yaml_);

    loadCameraParameters(camera_parameters_yaml_);
    loadMarkerInfo(marker_info_yaml_);

    // Subscribe to the image topic
    image_sub_ = it_.subscribe(input_image_topic_, 2, &ArucoDetector::imageCallback, this);
    image_pub_ = it_.advertise(output_image_topic_, 2);

    // Initialize the dictionary for ArUco markers
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);

    // Initialize the publisher for bounding boxes
    bbox_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(bbox_pub_topic_, 1);
}

void ArucoDetector::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
    try {
        // Convert the ROS image message to an OpenCV image
        frame = cv_bridge::toCvShare(msg, "bgr8")->image;        
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Detect ArUco markers
    std::vector<int> marker_ids, filtered_marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners, filtered_marker_corners;
    cv::aruco::detectMarkers(frame, dictionary_, marker_corners, marker_ids);

    if (marker_ids.empty()) {
        displayAndPublishImage(frame);
        return;
    }

    cv::aruco::drawDetectedMarkers(frame, marker_corners, marker_ids);

    std::vector<cv::Vec3d> rvecs, tvecs;
    std::vector<std::vector<cv::Point2f>> marker_corner = {marker_corners[0]};
    float marker_size = marker_info_[marker_ids[0]].size / 100;
    cv::aruco::estimatePoseSingleMarkers(marker_corner, marker_size, camera_matrix_, dist_coeffs_, rvecs, tvecs);

    tvecs[0][0] -= marker_info_[marker_ids[0]].x / 100;
    tvecs[0][1] -= marker_info_[marker_ids[0]].y / 100;
    cv::aruco::drawAxis(frame, camera_matrix_, dist_coeffs_, rvecs[0], tvecs[0], 0.1);
    displayAndPublishImage(frame);
}

void ArucoDetector::displayAndPublishImage(cv::Mat& frame) {
    // Display the image with detected markers
    if (is_display_) {
        cv::imshow("Aruco Detector", frame);
        cv::waitKey(1);
    }
    sensor_msgs::msg::Image::SharedPtr labeled_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    image_pub_.publish(labeled_image_msg);
}

void ArucoDetector::loadCameraParameters(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) 
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera calibration file: %s", filename.c_str());
        rclcpp::shutdown();
    }

    fs["camera_matrix"] >> camera_matrix_;
    fs["distortion_coefficients"] >> dist_coeffs_;
    fs.release();
}

void ArucoDetector::loadMarkerInfo(const std::string& filename) {
    YAML::Node config = YAML::LoadFile(filename);

    for (YAML::const_iterator it = config.begin(); it != config.end(); ++it) {
        int id = it->first.as<int>();
        YAML::Node marker_data = it->second;
        MarkerInfo info;
        info.size = marker_data["size"].as<float>();
        info.x = marker_data["x"].as<float>();
        info.y = marker_data["y"].as<float>();
        marker_info_[id] = info;
    }
}

    // float x_sum = 0, y_sum = 0, z_sum = 0;
    // float x_size_sum = 0, y_size_sum = 0, z_size_sum = 0;
    // cv::Vec3d rvec;

    // Calculate the marker-size-weighted average position of the markers
    // x_sum += (tvecs[0][0] - marker_info_[marker_ids[i]].x / 100) * marker_size;
    // y_sum += (tvecs[0][1] - marker_info_[marker_ids[i]].y / 100) * marker_size;
    // z_sum += tvecs[0][2] * marker_size;
    // x_size_sum += marker_size;
    // y_size_sum += marker_size;
    // z_size_sum += marker_size;
    // rvec = rvecs[0];

    // float x_avg = x_sum / x_size_sum;
    // float y_avg = y_sum / y_size_sum;
    // float z_avg = z_sum / z_size_sum;
    // cv::Vec3d tvec(x_avg, y_avg, z_avg);