#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include <opencv2/opencv.hpp>

struct Quaternion {
    double w, x, y, z;

    // Transform a translation vector
    cv::Vec3d transform(const cv::Vec3d& t) const;

    // Quaternion multiplication
    Quaternion operator*(const Quaternion& q) const;
};

// Function to convert rotation vector to quaternion
Quaternion rotationVectorToQuaternion(const cv::Vec3d& rvec);

#endif // QUATERNION_HPP
