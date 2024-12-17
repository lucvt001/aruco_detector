#include <aruco_detector/detector.h>


ArucoDetector::ArucoDetector() : Node("aruco_detector")
{
}

void ArucoDetector::initialize() {
        // Declare parameters
    this->declare_parameter<std::string>("input_image_topic", "/camera/image_raw");
    this->declare_parameter<std::string>("output_image_topic", "/camera/image_processed");
    this->declare_parameter<std::string>("landing_pad_position_topic", "/aruco/landing_pad_position");
    this->declare_parameter<bool>("is_display", true);
    this->declare_parameter<std::string>("camera_parameters_yaml", "camera_params.yaml");
    this->declare_parameter<std::string>("marker_info_yaml", "marker_info.yaml");

    // Get parameters
    this->get_parameter("input_image_topic", input_image_topic_);
    this->get_parameter("output_image_topic", output_image_topic_);
    this->get_parameter("landing_pad_position_topic", landing_pad_position_topic_);
    this->get_parameter("is_display", is_display_);
    this->get_parameter("camera_parameters_yaml", camera_parameters_yaml_);
    this->get_parameter("marker_info_yaml", marker_info_yaml_);

    loadCameraParameters(camera_parameters_yaml_);
    loadMarkerInfo(marker_info_yaml_);

    // Subscribe to the image topic
    image_transport::ImageTransport it_(shared_from_this());
    image_sub_ = it_.subscribe(input_image_topic_, 2, &ArucoDetector::imageCallback, this);
    image_pub_ = it_.advertise(output_image_topic_, 2);

    // Initialize the dictionary for ArUco markers
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);

    // Initialize the publisher for bounding boxes
    landing_pad_position_pub_ = this->create_publisher<geometry_msgs::msg::Point>(landing_pad_position_topic_, 1);
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

    std::vector<cv::Vec3d> rvecs, tvecs;
    std::vector<std::vector<cv::Point2f>> marker_corner = {marker_corners[0]};
    float marker_size = marker_info_[marker_ids[0]].size / 100;
    cv::aruco::estimatePoseSingleMarkers(marker_corner, marker_size, camera_matrix_, dist_coeffs_, rvecs, tvecs);

    // In this code, we only take one marker to estimate the pose
    // Because averaging the pose from multiple markers is complicated and not worth the effort
    cv::Vec3d tvec_marker = tvecs[0];
    cv::Vec3d rvec_marker = rvecs[0];


    Quaternion rvec_q = rotationVectorToQuaternion(rvec_marker);
    cv::Vec3d original_translation = cv::Vec3d(marker_info_[marker_ids[0]].x / 100, marker_info_[marker_ids[0]].y / 100, 0.0);
    cv::Vec3d transformed_translation = corrective_q_.transform(original_translation);
    transformed_translation = rvec_q.transform(transformed_translation);

    cv::Vec3d tvec_pad = tvec_marker - transformed_translation;
    cv::Vec3d rvec_pad = rvec_marker;

    // Remove outliers
    // if (cv::norm(rvec_marker) >= M_PI || cv::norm(rvec_marker) <= 0 ||
    //     tvec_marker[0] > 100 || tvec_marker[1] > 100 || tvec_marker[2] > 100) {
    //     displayAndPublishImage(frame);
    //     return;
    // }
    std::vector<cv::Point3d> object_points;
    object_points.push_back(cv::Point3d(0, 0, 0));
    std::vector<cv::Point2d> image_points;
    cv::projectPoints(object_points, rvec_pad, tvec_pad, camera_matrix_, dist_coeffs_, image_points);

    cv::Point2d pad_position_2d = image_points[0];

    int frame_width = frame.cols;
    int frame_height = frame.rows;
    cv::Point2d pad_position_2d_norm(pad_position_2d.x / frame_width, pad_position_2d.y / frame_height);

    // Increment towards the current normalized position
    if (pad_position_2d_norm_prev_.x > 0) {
        pad_position_2d_norm = incrementPoint2d(pad_position_2d_norm_prev_, pad_position_2d_norm, increment_point2d_);
        pad_position_2d_norm_prev_ = pad_position_2d_norm; // Update the previous position
    } else
        pad_position_2d_norm_prev_ = pad_position_2d_norm;

    double height = tvec_pad[2];
    if (height_prev_ > 0) {
        height = incrementDouble(height_prev_, height, increment_double_);
        height_prev_ = height;
    } else
        height_prev_ = height;       

    std::cout << "Pad position: " << pad_position_2d_norm << " Height: " << height << std::endl;

    // Publish the landing pad position
    geometry_msgs::msg::Point pad_position;
    pad_position.x = pad_position_2d_norm.x;
    pad_position.y = pad_position_2d_norm.y;
    pad_position.z = height;
    landing_pad_position_pub_->publish(pad_position);

    if (is_display_) {
        cv::aruco::drawDetectedMarkers(frame, marker_corners, marker_ids);
        // Draw a crosshair at the 2D position
        int crosshair_size = 30;
        pad_position_2d = cv::Point2d(pad_position_2d_norm.x * frame_width, pad_position_2d_norm.y * frame_height);
        cv::line(frame, cv::Point(pad_position_2d.x - crosshair_size, pad_position_2d.y), 
                cv::Point(pad_position_2d.x + crosshair_size, pad_position_2d.y), cv::Scalar(0, 0, 255), 2);
        cv::line(frame, cv::Point(pad_position_2d.x, pad_position_2d.y - crosshair_size), 
                cv::Point(pad_position_2d.x, pad_position_2d.y + crosshair_size), cv::Scalar(0, 0, 255), 2);
        // Draw the 3D axis
        cv::aruco::drawAxis(frame, camera_matrix_, dist_coeffs_, rvec_pad, tvec_pad, 0.1);
    }

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

cv::Point2d incrementPoint2d(const cv::Point2d& previous, const cv::Point2d& current, double increment) {
    cv::Point2d result = previous;

    if (previous.x < current.x) {
        result.x = std::min(previous.x + increment, current.x);
    } else if (previous.x > current.x) {
        result.x = std::max(previous.x - increment, current.x);
    }

    if (previous.y < current.y) {
        result.y = std::min(previous.y + increment, current.y);
    } else if (previous.y > current.y) {
        result.y = std::max(previous.y - increment, current.y);
    }

    return result;
}

double incrementDouble(const double previous, const double current, double increment) {
    double result = previous;

    if (previous < current) {
        result = std::min(previous + increment, current);
    } else if (previous > current) {
        result = std::max(previous - increment, current);
    }
    return result;
}