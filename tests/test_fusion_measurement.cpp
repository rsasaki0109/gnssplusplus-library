#include <gtest/gtest.h>

#include <libgnss++/fusion/fusion_measurement.hpp>

#include <functional>

#include <libgnss++/fusion/attitude.hpp>

namespace libgnss {
namespace {

using BuildFn = std::function<fusion_measurement::FusionMeasurementSystem(const FusionState&)>;

void injectErrorState(FusionState& state, const Eigen::Matrix<double, 15, 1>& dx) {
    state.nominal.position_enu += dx.segment<3>(fusion_index::POSITION);
    state.nominal.velocity_enu += dx.segment<3>(fusion_index::VELOCITY);
    state.nominal.attitude_body_to_enu =
        (state.nominal.attitude_body_to_enu *
         attitude::smallAngleQuaternion(dx.segment<3>(fusion_index::ATTITUDE)))
            .normalized();
    state.nominal.accel_bias += dx.segment<3>(fusion_index::ACCEL_BIAS);
    state.nominal.gyro_bias += dx.segment<3>(fusion_index::GYRO_BIAS);
}

// Central-difference Jacobian check: the single highest-leverage test in the
// fusion test plan (docs/design.md 6.6) -- catches analytic-Jacobian sign
// errors, especially in the lever-arm coupling terms, independent of whether
// an end-to-end run "looks" convergent.
void checkJacobian(const FusionState& state, const BuildFn& build, double eps = 1e-6,
                   double tol = 1e-6) {
    const auto system0 = build(state);
    const int m = static_cast<int>(system0.residuals.size());
    ASSERT_EQ(system0.design_matrix.rows(), m);
    ASSERT_EQ(system0.design_matrix.cols(), fusion_index::SIZE);

    for (int col = 0; col < fusion_index::SIZE; ++col) {
        Eigen::Matrix<double, 15, 1> dx = Eigen::Matrix<double, 15, 1>::Zero();
        dx(col) = eps;

        FusionState state_plus = state;
        injectErrorState(state_plus, dx);
        FusionState state_minus = state;
        injectErrorState(state_minus, -dx);

        const auto system_plus = build(state_plus);
        const auto system_minus = build(state_minus);
        ASSERT_EQ(system_plus.residuals.size(), m);
        ASSERT_EQ(system_minus.residuals.size(), m);

        // h(x) = z - residual(x); central difference of h w.r.t. dx_col:
        //   dh/d(dx_col) ~= (h(x+eps) - h(x-eps)) / (2 eps)
        //                = (residual(x-eps) - residual(x+eps)) / (2 eps)
        const Eigen::VectorXd numerical_column =
            (system_minus.residuals - system_plus.residuals) / (2.0 * eps);

        for (int row = 0; row < m; ++row) {
            EXPECT_NEAR(numerical_column(row), system0.design_matrix(row, col), tol)
                << "row=" << row << " col=" << col;
        }
    }
}

// A representative, non-degenerate nominal state that stresses every
// coupling term (nonzero tilt+yaw attitude, nonzero velocity, nonzero
// 3-axis lever arm, nonzero 3-axis angular rate).
FusionState makeStressState() {
    FusionState state;
    state.nominal.position_enu = Eigen::Vector3d(10.0, 20.0, 30.0);
    state.nominal.velocity_enu = Eigen::Vector3d(2.0, -1.0, 0.5);
    state.nominal.attitude_body_to_enu = Eigen::Quaterniond(
        Eigen::AngleAxisd(0.3, Eigen::Vector3d(1.0, 0.5, 0.2).normalized()));
    state.nominal.accel_bias = Eigen::Vector3d(0.02, -0.01, 0.03);
    state.nominal.gyro_bias = Eigen::Vector3d(0.001, -0.002, 0.0015);
    state.covariance = Eigen::Matrix<double, 15, 15>::Identity();
    return state;
}

const Eigen::Vector3d kLeverArm(0.31, 0.05, -0.55);
const Eigen::Vector3d kAngularRateBody(0.05, -0.03, 0.2);

TEST(FusionMeasurementJacobianTest, GnssPositionUpdate) {
    const FusionState state = makeStressState();
    const Eigen::Vector3d z(1.0, 2.0, 3.0);  // arbitrary; cancels out of the Jacobian check
    const Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    checkJacobian(state, [&](const FusionState& s) {
        return fusion_measurement::buildGnssPositionUpdate(s, z, R, kLeverArm);
    });
}

TEST(FusionMeasurementJacobianTest, GnssVelocityUpdate) {
    const FusionState state = makeStressState();
    const Eigen::Vector3d z(0.5, -0.5, 0.1);
    const Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    checkJacobian(state, [&](const FusionState& s) {
        return fusion_measurement::buildGnssVelocityUpdate(s, z, R, kLeverArm, kAngularRateBody);
    });
}

TEST(FusionMeasurementJacobianTest, ZuptUpdate) {
    const FusionState state = makeStressState();
    checkJacobian(state, [&](const FusionState& s) {
        return fusion_measurement::buildZuptUpdate(s, 0.5);
    });
}

TEST(FusionMeasurementJacobianTest, NhcUpdate) {
    const FusionState state = makeStressState();
    checkJacobian(state, [&](const FusionState& s) {
        return fusion_measurement::buildNhcUpdate(s, kLeverArm, kAngularRateBody, 0.3, 0.2);
    });
}

TEST(FusionMeasurementJacobianTest, GnssPositionUpdateWithZeroLeverArm) {
    // Degenerate but common case (no lever-arm offset configured): the
    // attitude coupling block should vanish entirely.
    const FusionState state = makeStressState();
    const Eigen::Vector3d z(0.0, 0.0, 0.0);
    const Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    const auto system = fusion_measurement::buildGnssPositionUpdate(state, z, R, Eigen::Vector3d::Zero());
    const Eigen::MatrixXd attitude_block = system.design_matrix.block<3, 3>(0, fusion_index::ATTITUDE);
    EXPECT_TRUE(attitude_block.isZero(1e-15));
    checkJacobian(state, [&](const FusionState& s) {
        return fusion_measurement::buildGnssPositionUpdate(s, z, R, Eigen::Vector3d::Zero());
    });
}

TEST(FusionMeasurementResidualTest, ZuptResidualIsNegativeVelocity) {
    FusionState state;
    state.nominal.velocity_enu = Eigen::Vector3d(0.3, -0.2, 0.05);
    const auto system = fusion_measurement::buildZuptUpdate(state, 0.5);
    EXPECT_TRUE(system.residuals.isApprox(-state.nominal.velocity_enu, 1e-12));
    EXPECT_NEAR(system.covariance(0, 0), 0.25, 1e-12);
}

}  // namespace
}  // namespace libgnss
