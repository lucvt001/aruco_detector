#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <stdexcept>

class ArucoDetector
{
public:
    ArucoDetector(const std::string& image_path, const std::string& output_path);
    void detectAndAnnotate();

private:
    cv::Mat image_;
    std::string output_path_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;

    void loadImage(const std::string& image_path);
    void saveImage(const std::string& output_path);
};

ArucoDetector::ArucoDetector(const std::string& image_path, const std::string& output_path)
    : output_path_(output_path)
{
    // Load the image
    loadImage(image_path);

    // Initialize the dictionary for APRILTAG_36h11
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
}

void ArucoDetector::loadImage(const std::string& image_path)
{
    image_ = cv::imread(image_path, cv::IMREAD_COLOR);
    if (image_.empty()) {
        throw std::runtime_error("Could not open or find the image: " + image_path);
    }
}

void ArucoDetector::saveImage(const std::string& output_path)
{
    if (!cv::imwrite(output_path, image_)) {
        throw std::runtime_error("Could not save the image: " + output_path);
    }
}

void ArucoDetector::detectAndAnnotate()
{
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;

    // Detect ArUco markers
    cv::aruco::detectMarkers(image_, dictionary_, markerCorners, markerIds);

    // Draw detected markers on the image
    if (!markerIds.empty()) {
        cv::aruco::drawDetectedMarkers(image_, markerCorners, markerIds);
    }

    // Save the annotated image
    saveImage(output_path_);
}

int main(int argc, char** argv)
{
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <input_image> <output_image>" << std::endl;
        return -1;
    }

    try {
        ArucoDetector detector(argv[1], argv[2]);
        detector.detectAndAnnotate();
        std::cout << "Annotated image saved to " << argv[2] << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}