#pragma once

#include <Eigen/Eigen>

using VectorXd = Eigen::VectorXd;
using Vector2d = Eigen::Vector2d;
using Vector3d = Eigen::Vector3d;
using Vector4d = Eigen::Vector4d;
using Vector6d = Eigen::Vector<double, 6>;
using MatrixXd = Eigen::MatrixXd;
using Quaterniond = Eigen::Quaterniond;
using AngleAxisd = Eigen::AngleAxisd;

#ifndef M_PI
#define M_PI 3.141592653589793
#endif

inline Vector3d quaternion_to_euler(const Quaterniond &quaterion)
{
    return quaterion.toRotationMatrix().eulerAngles(2, 0, 2);
}

/**
 * @brief Convert xzx euler angles to a quaternion.
 */
inline Quaterniond euler_to_quaternion(const Vector3d &euler)
{
    return (
        AngleAxisd(euler.x(), Vector3d::UnitZ()) *
        AngleAxisd(euler.y(), Vector3d::UnitX()) *
        AngleAxisd(euler.z(), Vector3d::UnitZ())
    );
}

// inline Vector3d euler_difference(const Vector3d &first, const Vector3d &second)
// {
// }
