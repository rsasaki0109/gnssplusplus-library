#include <gtest/gtest.h>

#include <libgnss++/fusion/fusion_measurement.hpp>
#include <libgnss++/fusion/fusion_update.hpp>

#include <cmath>

#include <libgnss++/fusion/attitude.hpp>

namespace libgnss {
namespace {

// Extracts the yaw angle of a body_to_enu attitude that is a pure rotation
// about the ENU Up axis (no roll/pitch), by reading off the rotated
// body-forward direction's heading in the horizontal plane.
double yawOf(const Eigen::Quaterniond& attitude_body_to_enu) {
    const Eigen::Vector3d forward_enu = attitude_body_to_enu * Eigen::Vector3d::UnitX();
    return std::atan2(forward_enu.y(), forward_enu.x());
}

void runNhcLoop(FusionState& state, const Eigen::Vector3d& lever_arm,
                const Eigen::Vector3d& angular_rate_body, int iterations) {
    for (int i = 0; i < iterations; ++i) {
        Eigen::Matrix<double, 15, 1> dx = Eigen::Matrix<double, 15, 1>::Zero();
        const auto system = fusion_measurement::buildNhcUpdate(state, lever_arm, angular_rate_body,
                                                                0.3, 0.2);
        const auto result = fusion_update::applyDenseUpdate(dx, state.covariance, system);
        ASSERT_TRUE(result.ok) << "NHC update failed at iteration " << i;

        state.nominal.velocity_enu += dx.segment<3>(fusion_index::VELOCITY);
        state.nominal.attitude_body_to_enu =
            (state.nominal.attitude_body_to_enu *
             attitude::smallAngleQuaternion(dx.segment<3>(fusion_index::ATTITUDE)))
                .normalized();
        state.nominal.gyro_bias += dx.segment<3>(fusion_index::GYRO_BIAS);
    }
}

TEST(FusionNhcTest, ReducesInjectedHeadingBiasOverSyntheticStraightMotion) {
    // Truth: vehicle moving due East at 5 m/s in a straight line (no
    // turning). Estimate: attitude carries a +5 deg yaw error relative to
    // truth, with zero GNSS input -- NHC alone (asserting body-lateral and
    // vertical velocity ~= 0) should observe and reduce this heading bias.
    FusionState state;
    state.nominal.velocity_enu = Eigen::Vector3d(5.0, 0.0, 0.0);
    const double injected_yaw_error_rad = 5.0 * M_PI / 180.0;
    state.nominal.attitude_body_to_enu =
        Eigen::Quaterniond(Eigen::AngleAxisd(injected_yaw_error_rad, Eigen::Vector3d::UnitZ()));
    state.covariance = 0.05 * Eigen::Matrix<double, 15, 15>::Identity();

    const double initial_yaw_error = std::abs(yawOf(state.nominal.attitude_body_to_enu));
    ASSERT_NEAR(initial_yaw_error, injected_yaw_error_rad, 1e-9);

    runNhcLoop(state, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 20);

    const double final_yaw_error = std::abs(yawOf(state.nominal.attitude_body_to_enu));
    EXPECT_LT(final_yaw_error, initial_yaw_error);
    EXPECT_LT(final_yaw_error, 0.5 * initial_yaw_error);
}

TEST(FusionNhcTest, ReducesInjectedHeadingBiasWithNonzeroLeverArm) {
    // Same idea, but with a nonzero (static) lever arm configured -- exercises
    // that code path without also introducing rotation, since a nonzero
    // angular rate together with a nonzero lever arm implies a real
    // omega x lever velocity contribution at the lever point that this test
    // does not attempt to construct a self-consistent "true" trajectory for
    // (that combination is exercised, self-consistently, by
    // FusionProcessorSyntheticTest's turn phase instead). With
    // angular_rate_body == 0 here, omega x lever == 0 regardless of the
    // lever arm value, so h(x) reduces to the same R^T*v_enu form as the
    // straight-motion test above; only the plumbing of a nonzero lever arm
    // is exercised.
    // Same self-consistent construction as the straight-motion test above
    // (velocity East, zero-error attitude's forward direction also East):
    // a nonzero-but-static lever arm should not change the fact that a yaw
    // error is observable and correctable via NHC.
    FusionState state;
    state.nominal.velocity_enu = Eigen::Vector3d(5.0, 0.0, 0.0);
    const double injected_yaw_error_rad = -8.0 * M_PI / 180.0;
    state.nominal.attitude_body_to_enu =
        Eigen::Quaterniond(Eigen::AngleAxisd(injected_yaw_error_rad, Eigen::Vector3d::UnitZ()));
    state.covariance = 0.05 * Eigen::Matrix<double, 15, 15>::Identity();

    const Eigen::Vector3d lever_arm(0.3, 0.0, -0.5);
    const Eigen::Vector3d angular_rate_body = Eigen::Vector3d::Zero();

    const double initial_yaw_error = std::abs(yawOf(state.nominal.attitude_body_to_enu));
    runNhcLoop(state, lever_arm, angular_rate_body, 20);
    const double final_yaw_error = std::abs(yawOf(state.nominal.attitude_body_to_enu));

    EXPECT_LT(final_yaw_error, initial_yaw_error);
}

}  // namespace
}  // namespace libgnss
