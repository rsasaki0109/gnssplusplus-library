#include <libgnss++/fusion/attitude.hpp>

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

}  // namespace attitude
}  // namespace libgnss
