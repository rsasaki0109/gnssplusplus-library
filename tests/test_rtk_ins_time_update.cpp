#include <gtest/gtest.h>

#include <limits>

#include <libgnss++/algorithms/rtk_ins_time_update.hpp>

namespace libgnss::rtk_ins_time_update {
namespace {

TEST(RTKInsTimeUpdateTest, AdvancesPositionAndPreservesEveryCrossCovariance) {
    Eigen::VectorXd state(7);
    state << 10.0, 20.0, 30.0, 4.0, 5.0, 6.0, 7.0;
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(7, 7);
    covariance.topRightCorner(3, 4) <<
        0.1, 0.2, 0.3, 0.4,
        0.5, 0.6, 0.7, 0.8,
        0.9, 1.0, 1.1, 1.2;
    covariance.bottomLeftCorner(4, 3) = covariance.topRightCorner(3, 4).transpose();
    const Eigen::MatrixXd cross_before = covariance.topRightCorner(3, 4);
    const Eigen::MatrixXd tail_before = covariance.bottomRightCorner(4, 4);

    const Eigen::Vector3d delta(1.0, -2.0, 3.0);
    const Eigen::Matrix3d noise =
        (Eigen::Vector3d(0.01, 0.02, 0.03)).asDiagonal();
    ASSERT_TRUE(apply(state, covariance, delta, noise, 0.001));

    EXPECT_TRUE(state.head<3>().isApprox(Eigen::Vector3d(11.0, 18.0, 33.0), 1e-15));
    EXPECT_TRUE(state.tail(4).isApprox((Eigen::Vector4d() << 4.0, 5.0, 6.0, 7.0).finished()));
    EXPECT_TRUE(covariance.topRightCorner(3, 4).isApprox(cross_before, 0.0));
    EXPECT_TRUE(covariance.bottomLeftCorner(4, 3).isApprox(cross_before.transpose(), 0.0));
    EXPECT_TRUE(covariance.bottomRightCorner(4, 4).isApprox(tail_before, 0.0));
    EXPECT_TRUE((covariance.topLeftCorner<3, 3>().isApprox(
        Eigen::Matrix3d::Identity() + noise + 0.001 * Eigen::Matrix3d::Identity(), 1e-15)));
}

TEST(RTKInsTimeUpdateTest, SymmetrizesRotatedProcessNoise) {
    Eigen::VectorXd state = Eigen::VectorXd::Ones(3);
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(3, 3);
    Eigen::Matrix3d noise;
    noise << 0.04, 0.01 + 1e-13, 0.0,
             0.01, 0.09, 0.0,
             0.0, 0.0, 0.16;
    ASSERT_TRUE(apply(state, covariance, Eigen::Vector3d::Zero(), noise, 0.0));
    EXPECT_TRUE(covariance.isApprox(covariance.transpose(), 0.0));
}

TEST(RTKInsTimeUpdateTest, RejectsIndefiniteNoiseWithoutMutation) {
    Eigen::VectorXd state = Eigen::VectorXd::Ones(5);
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(5, 5);
    const Eigen::VectorXd state_before = state;
    const Eigen::MatrixXd covariance_before = covariance;
    Eigen::Matrix3d noise = Eigen::Matrix3d::Zero();
    noise(0, 0) = -0.1;

    EXPECT_FALSE(apply(state, covariance, Eigen::Vector3d::Ones(), noise, 0.0));
    EXPECT_TRUE(state.isApprox(state_before, 0.0));
    EXPECT_TRUE(covariance.isApprox(covariance_before, 0.0));
}

TEST(RTKInsTimeUpdateTest, RejectsInvalidDimensionsAndNonFiniteInput) {
    Eigen::VectorXd short_state = Eigen::VectorXd::Ones(2);
    Eigen::MatrixXd short_covariance = Eigen::MatrixXd::Identity(2, 2);
    EXPECT_FALSE(apply(short_state, short_covariance, Eigen::Vector3d::Zero(),
                       Eigen::Matrix3d::Zero(), 0.0));

    Eigen::VectorXd state = Eigen::VectorXd::Ones(3);
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(3, 3);
    Eigen::Vector3d delta = Eigen::Vector3d::Zero();
    delta.z() = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(apply(state, covariance, delta, Eigen::Matrix3d::Zero(), 0.0));
}

TEST(RTKInsTimeUpdateTest, RejectsNegativeFloorWithoutMutation) {
    Eigen::VectorXd state = Eigen::VectorXd::Ones(3);
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(3, 3);
    const Eigen::VectorXd state_before = state;
    const Eigen::MatrixXd covariance_before = covariance;
    EXPECT_FALSE(apply(state, covariance, Eigen::Vector3d::Ones(),
                       Eigen::Matrix3d::Zero(), -1.0));
    EXPECT_TRUE(state.isApprox(state_before, 0.0));
    EXPECT_TRUE(covariance.isApprox(covariance_before, 0.0));
}

TEST(RTKInsTimeUpdateTest, AppendsVelocityWithoutMovingLegacyCrossCovariance) {
    Eigen::VectorXd state = Eigen::VectorXd::Zero(9);
    state.head<3>() << 10.0, 20.0, 30.0;
    state.segment<3>(3).setConstant(4.0);  // legacy states
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(9, 9);
    covariance.block<3, 3>(6, 6).setZero();
    covariance.block<3, 3>(0, 3).setConstant(0.25);
    covariance.block<3, 3>(3, 0).setConstant(0.25);
    const Eigen::Matrix3d legacy_cross_before = covariance.block<3, 3>(0, 3);
    Eigen::Matrix<double, 6, 6> noise =
        Eigen::Matrix<double, 6, 6>::Identity() * 0.01;
    noise.block<3, 3>(0, 3).setIdentity();
    noise.block<3, 3>(0, 3) *= 0.002;
    noise.block<3, 3>(3, 0) = noise.block<3, 3>(0, 3).transpose();

    ASSERT_TRUE(applyPositionVelocity(
        state, covariance, Eigen::Vector3d(1.0, -2.0, 3.0),
        Eigen::Vector3d(4.0, 5.0, 6.0), noise,
        2.0 * Eigen::Matrix3d::Identity(), 0.5, 6));

    EXPECT_TRUE(state.head<3>().isApprox(Eigen::Vector3d(11.0, 18.0, 33.0)));
    EXPECT_TRUE(state.segment<3>(3).isApprox(Eigen::Vector3d::Constant(4.0)));
    EXPECT_TRUE(state.tail<3>().isApprox(Eigen::Vector3d(4.0, 5.0, 6.0)));
    EXPECT_TRUE((covariance.block<3, 3>(0, 3).isApprox(legacy_cross_before, 0.0)));
    EXPECT_TRUE((covariance.block<3, 3>(6, 6).isApprox(
        2.01 * Eigen::Matrix3d::Identity(), 1e-12)));
    EXPECT_TRUE((covariance.block<3, 3>(0, 6).isApprox(
        0.002 * Eigen::Matrix3d::Identity(), 1e-12)));
}

}  // namespace
}  // namespace libgnss::rtk_ins_time_update
