#include <gtest/gtest.h>

#include <libgnss++/fusion/attitude.hpp>

namespace libgnss {
namespace {

TEST(AttitudeTest, SkewMatchesCrossProduct) {
    const Eigen::Vector3d v(1.0, -2.0, 3.0);
    const Eigen::Vector3d x(0.5, 0.25, -1.5);
    const Eigen::Matrix3d skew_v = attitude::skew(v);
    EXPECT_TRUE((skew_v * x).isApprox(v.cross(x), 1e-12));
    EXPECT_TRUE(skew_v.isApprox(-skew_v.transpose(), 1e-12));  // skew-symmetric
}

TEST(AttitudeTest, SmallAngleQuaternionRoundTripsThroughLogMap) {
    const Eigen::Vector3d dtheta(0.01, -0.02, 0.03);
    const Eigen::Quaterniond q = attitude::smallAngleQuaternion(dtheta);
    const Eigen::Vector3d recovered = attitude::quaternionToRotationVector(q);
    EXPECT_TRUE(recovered.isApprox(dtheta, 1e-9));
}

TEST(AttitudeTest, LargerAngleRoundTripsThroughLogMap) {
    const Eigen::Vector3d dtheta(0.3, -0.5, 0.8);
    const Eigen::Quaterniond q = attitude::smallAngleQuaternion(dtheta);
    const Eigen::Vector3d recovered = attitude::quaternionToRotationVector(q);
    EXPECT_TRUE(recovered.isApprox(dtheta, 1e-9));
}

TEST(AttitudeTest, ZeroRotationIsIdentity) {
    const Eigen::Quaterniond q = attitude::smallAngleQuaternion(Eigen::Vector3d::Zero());
    EXPECT_NEAR(q.w(), 1.0, 1e-12);
    EXPECT_NEAR(q.x(), 0.0, 1e-12);
    EXPECT_NEAR(q.y(), 0.0, 1e-12);
    EXPECT_NEAR(q.z(), 0.0, 1e-12);
}

TEST(AttitudeTest, DerivedRotationMatrixIsOrthonormal) {
    const Eigen::Vector3d dtheta(0.15, 0.42, -0.31);
    const Eigen::Quaterniond q = attitude::smallAngleQuaternion(dtheta);
    const Eigen::Matrix3d R = q.toRotationMatrix();
    EXPECT_TRUE((R * R.transpose()).isApprox(Eigen::Matrix3d::Identity(), 1e-10));
    EXPECT_NEAR(R.determinant(), 1.0, 1e-10);
}

TEST(AttitudeTest, CompositionMatchesRotationMatrixProduct) {
    const Eigen::Vector3d a(0.1, 0.0, 0.0);
    const Eigen::Vector3d b(0.0, 0.2, 0.0);
    const Eigen::Quaterniond qa = attitude::smallAngleQuaternion(a);
    const Eigen::Quaterniond qb = attitude::smallAngleQuaternion(b);
    const Eigen::Quaterniond qab = (qa * qb).normalized();

    const Eigen::Matrix3d Ra = qa.toRotationMatrix();
    const Eigen::Matrix3d Rb = qb.toRotationMatrix();
    const Eigen::Matrix3d Rab_expected = Ra * Rb;

    EXPECT_TRUE(qab.toRotationMatrix().isApprox(Rab_expected, 1e-10));
}

TEST(AttitudeTest, QuaternionRotatesVectorLikeAngleAxis) {
    const double angle = 0.7;
    const Eigen::Vector3d axis = Eigen::Vector3d(1.0, 1.0, 1.0).normalized();
    const Eigen::Quaterniond q = attitude::smallAngleQuaternion(angle * axis);
    const Eigen::Vector3d v(1.0, 0.0, 0.0);
    const Eigen::Vector3d rotated_by_q = q * v;
    const Eigen::Vector3d rotated_by_angle_axis = Eigen::AngleAxisd(angle, axis) * v;
    EXPECT_TRUE(rotated_by_q.isApprox(rotated_by_angle_axis, 1e-10));
}

}  // namespace
}  // namespace libgnss
