#include <fiducial_detector/detector.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "fiducial_detector");
    ros::NodeHandle nh("~");

    ArucoDetector detector(nh);

    ros::spin();

    return 0;
}