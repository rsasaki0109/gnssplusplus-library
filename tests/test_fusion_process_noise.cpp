#include <gtest/gtest.h>

#include <libgnss++/fusion/fusion_process_noise.hpp>

#include <cmath>

namespace libgnss {
namespace {

TEST(FusionProcessNoiseTest, TransitionMatrixIsIdentityAtZeroDt) {
    NominalState state;
    state.attitude_body_to_enu = Eigen::Quaterniond::Identity();
    const Eigen::Vector3d specific_force_body(0.1, 0.2, 9.81);
    const Eigen::Vector3d angular_rate_body(0.01, -0.02, 0.03);

    const auto phi = fusion_process_noise::transitionMatrix(state, specific_force_body,
                                                            angular_rate_body, 0.0);
    EXPECT_TRUE(phi.isApprox(Eigen::Matrix<double, 15, 15>::Identity(), 1e-12));
}

TEST(FusionProcessNoiseTest, TransitionMatrixCouplesPositionToVelocity) {
    NominalState state;
    state.attitude_body_to_enu = Eigen::Quaterniond::Identity();
    const Eigen::Vector3d specific_force_body = Eigen::Vector3d::Zero();
    const Eigen::Vector3d angular_rate_body = Eigen::Vector3d::Zero();
    const double dt = 0.05;

    const auto phi =
        fusion_process_noise::transitionMatrix(state, specific_force_body, angular_rate_body, dt);
    const Eigen::Matrix3d position_velocity_block =
        phi.block<3, 3>(fusion_index::POSITION, fusion_index::VELOCITY);
    EXPECT_TRUE(position_velocity_block.isApprox(dt * Eigen::Matrix3d::Identity(), 1e-12));

    // With zero specific force/angular rate and R=I, Fc has exactly three
    // nonzero blocks -- POSITION<-VELOCITY=I, VELOCITY<-ACCEL_BIAS=-R=-I,
    // ATTITUDE<-GYRO_BIAS=-I (these bias couplings are constant, independent
    // of specific force/angular rate) -- and Fc is nilpotent of index 3
    // (Fc^2 has a single nonzero block POSITION<-ACCEL_BIAS = -I, chained
    // through VELOCITY; Fc^3 = 0). So Phi = I + Fc*dt + (Fc*dt)^2/2 is exact
    // (no truncation error) and fully predictable in closed form here.
    Eigen::Matrix<double, 15, 15> expected = Eigen::Matrix<double, 15, 15>::Identity();
    expected.block<3, 3>(fusion_index::POSITION, fusion_index::VELOCITY) =
        dt * Eigen::Matrix3d::Identity();
    expected.block<3, 3>(fusion_index::VELOCITY, fusion_index::ACCEL_BIAS) =
        -dt * Eigen::Matrix3d::Identity();
    expected.block<3, 3>(fusion_index::ATTITUDE, fusion_index::GYRO_BIAS) =
        -dt * Eigen::Matrix3d::Identity();
    expected.block<3, 3>(fusion_index::POSITION, fusion_index::ACCEL_BIAS) =
        -0.5 * dt * dt * Eigen::Matrix3d::Identity();
    EXPECT_TRUE(phi.isApprox(expected, 1e-12));
}

TEST(FusionProcessNoiseTest, PositionVarianceGrowthMatchesDoubleIntegratedWhiteNoiseAnalytic) {
    // Isolate the position<-velocity<-accel-noise double integrator: zero
    // specific force / angular rate so attitude/bias couplings vanish, and
    // zero out every process-noise term except accel_noise_density so only
    // the classic double-integrated white noise result applies:
    //   Var(p)(T) = sigma_a^2 * T^3 / 3
    NominalState state;
    state.attitude_body_to_enu = Eigen::Quaterniond::Identity();
    const Eigen::Vector3d specific_force_body = Eigen::Vector3d::Zero();
    const Eigen::Vector3d angular_rate_body = Eigen::Vector3d::Zero();

    ProcessNoiseConfig cfg;
    cfg.accel_noise_density = 0.01;
    cfg.gyro_noise_density = 0.0;
    cfg.accel_bias_random_walk = 0.0;
    cfg.gyro_bias_random_walk = 0.0;

    const double dt = 0.001;
    const int steps = 2000;  // T = 2 s

    Eigen::Matrix<double, 15, 15> covariance = Eigen::Matrix<double, 15, 15>::Zero();
    for (int i = 0; i < steps; ++i) {
        const auto phi = fusion_process_noise::transitionMatrix(state, specific_force_body,
                                                                 angular_rate_body, dt);
        const auto qd = fusion_process_noise::processNoiseCovariance(phi, cfg, dt);
        covariance = phi * covariance * phi.transpose() + qd;
    }

    const double total_time = steps * dt;
    const double analytic_position_variance =
        cfg.accel_noise_density * cfg.accel_noise_density * total_time * total_time * total_time / 3.0;

    // Only the X axis of the position/velocity block is populated (accel
    // noise is isotropic, so every axis behaves identically); check one.
    const double simulated_position_variance =
        covariance(fusion_index::POSITION, fusion_index::POSITION);

    EXPECT_NEAR(simulated_position_variance, analytic_position_variance,
               0.02 * analytic_position_variance);
}

}  // namespace
}  // namespace libgnss
