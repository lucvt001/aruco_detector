#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <stdexcept>

class ArucoDetector
{
public:
    ArucoDetector(const std::string& image_path, const std::string& output_image_path, const std::string& output_yaml_path, int reference_id);
    void detectAndAnnotate();

private:
    cv::Mat image_;
    std::string output_image_path_;
    std::string output_yaml_path_;
    int reference_id_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;

    void loadImage(const std::string& image_path);
    void saveImage(const std::string& output_path);
    void saveOffsetsToYAML(const std::map<int, std::pair<float, float>>& offsets);
};

ArucoDetector::ArucoDetector(const std::string& image_path, const std::string& output_image_path, const std::string& output_yaml_path, int reference_id)
    : output_image_path_(output_image_path), output_yaml_path_(output_yaml_path), reference_id_(reference_id)
{
    // Load the image
    loadImage(image_path);

    // Initialize the dictionary for ArUco markers
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

void ArucoDetector::saveOffsetsToYAML(const std::map<int, std::pair<float, float>>& offsets)
{
    YAML::Emitter out;
    out << YAML::BeginMap;
    for (const auto& offset : offsets) {
        out << YAML::Key << offset.first;
        out << YAML::Value << YAML::Flow << YAML::BeginSeq << offset.second.first << offset.second.second << YAML::EndSeq;
    }
    out << YAML::EndMap;

    std::ofstream fout(output_yaml_path_);
    if (!fout.is_open()) {
        throw std::runtime_error("Could not open YAML file for writing: " + output_yaml_path_);
    }
    fout << out.c_str();
    fout.close();
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

        // Find the reference marker
        auto it = std::find(markerIds.begin(), markerIds.end(), reference_id_);
        if (it == markerIds.end()) {
            throw std::runtime_error("Reference marker ID not found in the image.");
        }

        int reference_index = std::distance(markerIds.begin(), it);
        cv::Point2f reference_center = (markerCorners[reference_index][0] + markerCorners[reference_index][2]) * 0.5;

        // Calculate offsets
        std::map<int, std::pair<float, float>> offsets;
        for (size_t i = 0; i < markerIds.size(); ++i) {
            if (markerIds[i] == reference_id_) continue;

            cv::Point2f center = (markerCorners[i][0] + markerCorners[i][2]) * 0.5;
            float offset_x = (center.x - reference_center.x) / image_.cols;
            float offset_y = (center.y - reference_center.y) / image_.rows;
            offsets[markerIds[i]] = std::make_pair(offset_x, offset_y);
        }

        // Save the offsets to a YAML file
        saveOffsetsToYAML(offsets);
    }

    // Save the annotated image
    // saveImage(output_image_path_);
}

int main(int argc, char** argv)
{
    if (argc != 5) {
        std::cerr << "Usage: " << argv[0] << " <input_image> <output_image> <output_yaml> <reference_id>" << std::endl;
        return -1;
    }

    std::string input_image_path = argv[1];
    std::string output_image_path = argv[2];
    std::string output_yaml_path = argv[3];
    int reference_id = std::stoi(argv[4]);

    try {
        ArucoDetector detector(input_image_path, output_image_path, output_yaml_path, reference_id);
        detector.detectAndAnnotate();
        // std::cout << "Annotated image saved to " << output_image_path << std::endl;
        std::cout << "Offsets saved to " << output_yaml_path << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}