#include <libgnss++/fusion/attitude.hpp>
#include <cmath>

namespace libgnss {
namespace attitude {

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m <<     0.0, -v.z(),  v.y(),
           v.z(),    0.0, -v.x(),
          -v.y(),  v.x(),    0.0;
    return m;
}

Eigen::Quaterniond smallAngleQuaternion(const Eigen::Vector3d& dtheta) {
    const double angle = dtheta.norm();
    if (angle < 1e-9) {
        // First-order approximation, avoids a 0/0 division for a
        // near-zero rotation vector; renormalized to stay a unit quaternion.
        Eigen::Quaterniond q(1.0, 0.5 * dtheta.x(), 0.5 * dtheta.y(), 0.5 * dtheta.z());
        return q.normalized();
    }
    const Eigen::Vector3d axis = dtheta / angle;
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
}

Eigen::Vector3d quaternionToRotationVector(const Eigen::Quaterniond& q) {
    const Eigen::AngleAxisd aa(q.normalized());
    return aa.angle() * aa.axis();
}

Eigen::Vector3d fluEnuToFrdNedRpyDegrees(const Eigen::Quaterniond& q) {
    Eigen::Matrix3d d = Eigen::Matrix3d::Identity();
    d(1, 1) = -1.0;
    d(2, 2) = -1.0;
    Eigen::Matrix3d p;
    p << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0;
    const Eigen::Matrix3d r = p * q.toRotationMatrix() * d;
    const double degrees = 180.0 / std::acos(-1.0);
    double heading = std::atan2(r(1, 0), r(0, 0)) * degrees;
    if (heading < 0.0) heading += 360.0;
    if (heading >= 360.0) heading = 0.0; // roundoff just below zero
    return Eigen::Vector3d(std::atan2(r(2, 1), r(2, 2)) * degrees,
        std::atan2(-r(2, 0), std::sqrt(r(2, 1) * r(2, 1) + r(2, 2) * r(2, 2))) * degrees,
        heading);
}

}  // namespace attitude
}  // namespace libgnss
