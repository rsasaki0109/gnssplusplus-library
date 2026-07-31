#include <gtest/gtest.h>

#include <limits>

#include <libgnss++/fusion/mechanization.hpp>
#include <libgnss++/fusion/preintegration.hpp>

namespace libgnss {
namespace {

NominalState stationaryAnchor() {
    NominalState anchor;
    anchor.time = GNSSTime(2200, 100.0);
    anchor.attitude_body_to_enu = Eigen::Quaterniond::Identity();
    return anchor;
}

ImuSample stationarySample(const GNSSTime& time) {
    ImuSample sample;
    sample.time = time;
    sample.accel_raw = Eigen::Vector3d(0.0, 0.0, kStandardGravityMps2);
    sample.gyro_raw_radps.setZero();
    return sample;
}

TEST(ImuPreintegratorTest, RejectsSamplesBeforeReset) {
    ImuPreintegrator preintegrator;
    EXPECT_EQ(preintegrator.integrate(stationarySample(GNSSTime(2200, 100.01))),
              PreintegrationStatus::NOT_INITIALIZED);
    EXPECT_FALSE(preintegrator.result().valid);
}

TEST(ImuPreintegratorTest, PropagatesConstantVelocityAndComposesDuration) {
    ImuPreintegrator preintegrator;
    NominalState anchor = stationaryAnchor();
    anchor.velocity_enu = Eigen::Vector3d(5.0, -1.0, 0.0);
    ASSERT_EQ(preintegrator.reset(anchor), PreintegrationStatus::ACCEPTED);

    constexpr double dt = 0.01;
    constexpr int count = 100;
    for (int i = 1; i <= count; ++i) {
        ASSERT_EQ(preintegrator.integrate(stationarySample(anchor.time + i * dt)),
                  PreintegrationStatus::ACCEPTED);
    }

    const auto result = preintegrator.result();
    ASSERT_TRUE(result.valid);
    EXPECT_EQ(result.sample_count, count);
    EXPECT_NEAR(result.duration_s, 1.0, 1e-9);
    EXPECT_TRUE(result.predicted_state.position_enu.isApprox(
        Eigen::Vector3d(5.0, -1.0, 0.0), 1e-9));
    EXPECT_TRUE(result.predicted_state.velocity_enu.isApprox(anchor.velocity_enu, 1e-9));
}

TEST(ImuPreintegratorTest, TransitionCompositionMatchesStepwiseProduct) {
    ImuPreintegrator preintegrator;
    const NominalState anchor = stationaryAnchor();
    ASSERT_EQ(preintegrator.reset(anchor), PreintegrationStatus::ACCEPTED);

    const ImuSample first = stationarySample(anchor.time + 0.01);
    const ImuSample second = stationarySample(anchor.time + 0.03);
    const Eigen::Vector3d force(0.0, 0.0, kStandardGravityMps2);
    const Eigen::Vector3d rate = Eigen::Vector3d::Zero();
    const FusionMatrix15 phi1 =
        fusion_process_noise::transitionMatrix(anchor, force, rate, 0.01);
    const NominalState after_first = mechanization::propagate(
        anchor, first, 0.01, Eigen::Vector3d(0.0, 0.0, -kStandardGravityMps2));
    const FusionMatrix15 phi2 =
        fusion_process_noise::transitionMatrix(after_first, force, rate, 0.02);

    ASSERT_EQ(preintegrator.integrate(first), PreintegrationStatus::ACCEPTED);
    ASSERT_EQ(preintegrator.integrate(second), PreintegrationStatus::ACCEPTED);
    EXPECT_TRUE(preintegrator.result().transition.isApprox(phi2 * phi1, 1e-12));
}

TEST(ImuPreintegratorTest, AccumulatesSymmetricPositiveProcessNoise) {
    ImuPreintegrationConfig config;
    config.process_noise.accel_noise_density = 0.01;
    config.process_noise.gyro_noise_density = 0.001;
    ImuPreintegrator preintegrator(config);
    const NominalState anchor = stationaryAnchor();
    ASSERT_EQ(preintegrator.reset(anchor), PreintegrationStatus::ACCEPTED);

    for (int i = 1; i <= 100; ++i) {
        ASSERT_EQ(preintegrator.integrate(stationarySample(anchor.time + i * 0.01)),
                  PreintegrationStatus::ACCEPTED);
    }

    const FusionMatrix15 covariance = preintegrator.result().process_noise;
    EXPECT_TRUE(covariance.isApprox(covariance.transpose(), 1e-14));
    EXPECT_GT(covariance(fusion_index::POSITION, fusion_index::POSITION), 0.0);
    EXPECT_GE(covariance.selfadjointView<Eigen::Lower>().eigenvalues().minCoeff(), -1e-14);
}

TEST(ImuPreintegratorTest, ResetClearsPreviousInterval) {
    ImuPreintegrator preintegrator;
    const NominalState anchor = stationaryAnchor();
    ASSERT_EQ(preintegrator.reset(anchor), PreintegrationStatus::ACCEPTED);
    ASSERT_EQ(preintegrator.integrate(stationarySample(anchor.time + 0.01)),
              PreintegrationStatus::ACCEPTED);
    ASSERT_TRUE(preintegrator.result().valid);

    NominalState next_anchor = anchor;
    next_anchor.time = anchor.time + 5.0;
    ASSERT_EQ(preintegrator.reset(next_anchor), PreintegrationStatus::ACCEPTED);
    const auto result = preintegrator.result();
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.sample_count, 0u);
    EXPECT_DOUBLE_EQ(result.duration_s, 0.0);
    EXPECT_TRUE(result.transition.isIdentity(1e-15));
    EXPECT_TRUE(result.process_noise.isZero(0.0));
}

TEST(ImuPreintegratorTest, NonMonotonicTimeInvalidatesWholeInterval) {
    ImuPreintegrator preintegrator;
    const NominalState anchor = stationaryAnchor();
    ASSERT_EQ(preintegrator.reset(anchor), PreintegrationStatus::ACCEPTED);
    ASSERT_EQ(preintegrator.integrate(stationarySample(anchor.time + 0.02)),
              PreintegrationStatus::ACCEPTED);
    EXPECT_EQ(preintegrator.integrate(stationarySample(anchor.time + 0.01)),
              PreintegrationStatus::NON_MONOTONIC_TIME);
    EXPECT_FALSE(preintegrator.result().valid);
    EXPECT_EQ(preintegrator.integrate(stationarySample(anchor.time + 0.03)),
              PreintegrationStatus::INTERVAL_INVALID);
}

TEST(ImuPreintegratorTest, ExcessiveGapInvalidatesWholeInterval) {
    ImuPreintegrationConfig config;
    config.max_sample_gap_s = 0.05;
    ImuPreintegrator preintegrator(config);
    const NominalState anchor = stationaryAnchor();
    ASSERT_EQ(preintegrator.reset(anchor), PreintegrationStatus::ACCEPTED);
    EXPECT_EQ(preintegrator.integrate(stationarySample(anchor.time + 0.06)),
              PreintegrationStatus::GAP_TOO_LARGE);
    EXPECT_FALSE(preintegrator.result().valid);
}

TEST(ImuPreintegratorTest, NonFiniteSampleInvalidatesWholeInterval) {
    ImuPreintegrator preintegrator;
    const NominalState anchor = stationaryAnchor();
    ASSERT_EQ(preintegrator.reset(anchor), PreintegrationStatus::ACCEPTED);
    ImuSample sample = stationarySample(anchor.time + 0.01);
    sample.accel_raw.x() = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(preintegrator.integrate(sample), PreintegrationStatus::NON_FINITE_SAMPLE);
    EXPECT_FALSE(preintegrator.result().valid);
}

TEST(ImuPreintegratorTest, NonFiniteAnchorIsRejected) {
    ImuPreintegrator preintegrator;
    NominalState anchor = stationaryAnchor();
    anchor.velocity_enu.y() = std::numeric_limits<double>::infinity();
    EXPECT_EQ(preintegrator.reset(anchor), PreintegrationStatus::INVALID_ANCHOR);
    EXPECT_FALSE(preintegrator.initialized());
    EXPECT_FALSE(preintegrator.result().valid);
}

TEST(ImuPreintegratorTest, InvalidConfigIsRejectedAtReset) {
    ImuPreintegrationConfig config;
    config.process_noise.gyro_noise_density =
        std::numeric_limits<double>::quiet_NaN();
    ImuPreintegrator preintegrator(config);
    EXPECT_EQ(preintegrator.reset(stationaryAnchor()),
              PreintegrationStatus::INVALID_CONFIG);
    EXPECT_FALSE(preintegrator.initialized());
}

TEST(ImuPreintegratorTest, OverflowDuringPropagationInvalidatesWholeInterval) {
    ImuPreintegrator preintegrator;
    const NominalState anchor = stationaryAnchor();
    ASSERT_EQ(preintegrator.reset(anchor), PreintegrationStatus::ACCEPTED);
    ImuSample sample = stationarySample(anchor.time + 0.01);
    sample.accel_raw.setConstant(std::numeric_limits<double>::max());
    EXPECT_EQ(preintegrator.integrate(sample),
              PreintegrationStatus::PROPAGATION_FAILURE);
    EXPECT_FALSE(preintegrator.result().valid);
}

}  // namespace
}  // namespace libgnss
