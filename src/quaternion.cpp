#include <fiducial_detector/quaternion.hpp>
#include <cmath>

cv::Vec3d Quaternion::transform(const cv::Vec3d& t) const {
    Quaternion t_quat{0, t[0], t[1], t[2]};
    Quaternion q_conj{-w, x, y, z};
    Quaternion result = (*this * t_quat) * q_conj;
    return cv::Vec3d(result.x, result.y, result.z);
}

Quaternion Quaternion::operator*(const Quaternion& q) const {
    return {
        w * q.w - x * q.x - y * q.y - z * q.z,
        w * q.x + x * q.w + y * q.z - z * q.y,
        w * q.y - x * q.z + y * q.w + z * q.x,
        w * q.z + x * q.y - y * q.x + z * q.w
    };
}

Quaternion rotationVectorToQuaternion(const cv::Vec3d& rvec) {
    double theta = cv::norm(rvec);
    if (theta < 1e-6) {
        return {1.0, 0.0, 0.0, 0.0};
    }
    cv::Vec3d axis = rvec / theta;
    double half_theta = theta / 2.0;
    double sin_half_theta = std::sin(half_theta);
    return Quaternion{
        std::cos(half_theta),
        axis[0] * sin_half_theta,
        axis[1] * sin_half_theta,
        axis[2] * sin_half_theta
    };
}
