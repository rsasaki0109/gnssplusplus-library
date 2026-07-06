#pragma once

#include <Eigen/Dense>

namespace libgnss {
namespace attitude {

/**
 * @brief Skew-symmetric ("cross-product") matrix of a 3-vector, so that
 * skew(v) * x == v.cross(x).
 */
Eigen::Matrix3d skew(const Eigen::Vector3d& v);

/**
 * @brief Exact rotation-vector exponential map: converts a 3D rotation vector
 * (axis * angle, radians) into the corresponding unit quaternion.
 *
 * For the error-state EKF's injection step (docs/design.md 3.6) `dtheta` is a
 * small angle by construction, but the exact exponential map (rather than a
 * first-order `[1, dtheta/2]` truncation) is used throughout so this same
 * helper is correct for both the small-angle injection case and the
 * larger-angle mechanization increments exercised in tests.
 */
Eigen::Quaterniond smallAngleQuaternion(const Eigen::Vector3d& dtheta);

/**
 * @brief Inverse of smallAngleQuaternion(): the rotation-vector logarithmic
 * map of a unit quaternion (axis * angle, radians).
 */
Eigen::Vector3d quaternionToRotationVector(const Eigen::Quaterniond& q);

}  // namespace attitude
}  // namespace libgnss
