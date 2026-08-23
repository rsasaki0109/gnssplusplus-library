#include <gtest/gtest.h>

#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/fusion/fusion_processor.hpp>

namespace libgnss {
namespace {

constexpr double kGravity = 9.80665;
constexpr double kDt = 0.01;  // 100 Hz

// Builds a level-attitude, stationary IMU sample: accel reads gravity plus
// the injected true accel bias and an optional alternating dither of
// amplitude `accel_amp` (zero-mean over any even-length window, so it
// contributes exactly `accel_amp` to the per-sample-norm population std used
// by the ZUPT detector without shifting the mean); gyro reads the injected
// true gyro bias plus an alternating dither of amplitude `gyro_amp`.
ImuSample makeStationarySample(int index, const GNSSTime& time,
                               const Eigen::Vector3d& true_accel_bias,
                               const Eigen::Vector3d& true_gyro_bias, double accel_amp,
                               double gyro_amp) {
    ImuSample sample;
    sample.time = time;
    const double sign = (index % 2 == 0) ? 1.0 : -1.0;
    sample.accel_raw = Eigen::Vector3d(0.0, 0.0, kGravity) + true_accel_bias +
                       Eigen::Vector3d(sign * accel_amp, 0.0, 0.0);
    sample.gyro_raw_radps = true_gyro_bias + Eigen::Vector3d(0.0, sign * gyro_amp, 0.0);
    return sample;
}

LooseCouplingProcessor::Config makeConfig(bool zupt_enable) {
    LooseCouplingProcessor::Config config;
    config.zupt_enable = zupt_enable;
    config.align_static_window_s = 1.0;  // 100 samples
    return config;
}

PositionSolution makeGnssSolution(const LooseCouplingProcessor& processor,
                                  const Eigen::Vector3d& velocity_enu,
                                  bool has_velocity) {
    constexpr double kLat = 35.6 * M_PI / 180.0;
    constexpr double kLon = 139.7 * M_PI / 180.0;
    PositionSolution solution;
    solution.time = processor.state().nominal.time;
    solution.status = SolutionStatus::FIXED;
    solution.num_satellites = 10;
    solution.position_ecef = geodetic2ecef(kLat, kLon, 50.0);
    solution.position_covariance = Eigen::Matrix3d::Identity();
    solution.has_velocity = has_velocity;
    if (has_velocity) {
        solution.velocity_ecef = enu2ecef(velocity_enu, kLat, kLon);
        solution.velocity_covariance = 0.01 * Eigen::Matrix3d::Identity();
    }
    return solution;
}

void feedQuietSamples(LooseCouplingProcessor& processor, int count, int start_index = 0) {
    GNSSTime time = processor.state().nominal.time;
    for (int i = 0; i < count; ++i) {
        processor.processImuSample(
            makeStationarySample(start_index + i, time, Eigen::Vector3d::Zero(),
                                 Eigen::Vector3d::Zero(), 0.0, 0.0));
        time = time + kDt;
    }
}

void feedStationaryRun(LooseCouplingProcessor& processor, int sample_count,
                      const Eigen::Vector3d& true_accel_bias,
                      const Eigen::Vector3d& true_gyro_bias, double accel_amp, double gyro_amp,
                      const Eigen::Vector3d& extra_constant_accel_offset = Eigen::Vector3d::Zero()) {
    GNSSTime time(2000, 100.0);
    for (int i = 0; i < sample_count; ++i) {
        ImuSample sample =
            makeStationarySample(i, time, true_accel_bias, true_gyro_bias, accel_amp, gyro_amp);
        sample.accel_raw += extra_constant_accel_offset;
        processor.processImuSample(sample);
        time = time + kDt;
    }
}

TEST(FusionZuptTest, QuietStationaryRunBelowDocumentedThresholdsShrinksVelocityCovariance) {
    // Documented defaults: zupt_max_accel_std=0.55, zupt_max_gyro_std=0.030.
    // Use amplitudes comfortably below both.
    LooseCouplingProcessor processor(makeConfig(/*zupt_enable=*/true));
    const Eigen::Vector3d true_accel_bias(0.01, -0.02, 0.03);
    const Eigen::Vector3d true_gyro_bias(0.001, 0.0005, -0.0008);

    feedStationaryRun(processor, 150, true_accel_bias, true_gyro_bias, 0.05, 0.005);  // init window (>1s)
    const double covariance_after_init =
        processor.state().covariance(fusion_index::VELOCITY, fusion_index::VELOCITY);

    feedStationaryRun(processor, 300, true_accel_bias, true_gyro_bias, 0.05, 0.005);
    const double covariance_after_zupt =
        processor.state().covariance(fusion_index::VELOCITY, fusion_index::VELOCITY);

    EXPECT_LT(covariance_after_zupt, covariance_after_init);
}

TEST(FusionZuptTest, NoisyStationaryRunAboveDocumentedThresholdsDoesNotShrinkVelocityCovariance) {
    // Amplitudes clearly above both documented thresholds: the stationarity
    // gate should never fire, so no ZUPT correction is ever applied and the
    // velocity covariance only grows from process noise (never shrinks
    // below its post-alignment value).
    LooseCouplingProcessor processor(makeConfig(/*zupt_enable=*/true));
    const Eigen::Vector3d true_accel_bias(0.01, -0.02, 0.03);
    const Eigen::Vector3d true_gyro_bias(0.001, 0.0005, -0.0008);

    feedStationaryRun(processor, 150, true_accel_bias, true_gyro_bias, 0.05, 0.005);  // quiet init window (>1s)
    const double covariance_after_init =
        processor.state().covariance(fusion_index::VELOCITY, fusion_index::VELOCITY);

    feedStationaryRun(processor, 300, true_accel_bias, true_gyro_bias, /*accel_amp=*/2.0,
                     /*gyro_amp=*/0.5);
    const double covariance_after_noisy_run =
        processor.state().covariance(fusion_index::VELOCITY, fusion_index::VELOCITY);

    EXPECT_GE(covariance_after_noisy_run, covariance_after_init);
}

TEST(FusionZuptTest, SuppressesVelocityDriftFromResidualBiasError) {
    // A small residual accel-bias error left uncorrected by static alignment
    // (simulated here as a constant offset applied only after
    // initialization) causes mechanization to integrate a fictitious
    // velocity. With ZUPT enabled, the continuous "velocity ~= 0" constraint
    // should keep the estimated velocity much smaller than with ZUPT
    // disabled over the same run.
    const Eigen::Vector3d true_accel_bias(0.0, 0.0, 0.0);
    const Eigen::Vector3d true_gyro_bias(0.0, 0.0, 0.0);
    const Eigen::Vector3d residual_offset(0.05, 0.0, 0.0);  // uncompensated accel error, m/s^2

    LooseCouplingProcessor with_zupt(makeConfig(/*zupt_enable=*/true));
    feedStationaryRun(with_zupt, 150, true_accel_bias, true_gyro_bias, 0.0, 0.0);
    feedStationaryRun(with_zupt, 500, true_accel_bias, true_gyro_bias, 0.0, 0.0, residual_offset);
    const double velocity_with_zupt = with_zupt.state().nominal.velocity_enu.norm();

    LooseCouplingProcessor without_zupt(makeConfig(/*zupt_enable=*/false));
    feedStationaryRun(without_zupt, 150, true_accel_bias, true_gyro_bias, 0.0, 0.0);
    feedStationaryRun(without_zupt, 500, true_accel_bias, true_gyro_bias, 0.0, 0.0, residual_offset);
    const double velocity_without_zupt = without_zupt.state().nominal.velocity_enu.norm();

    EXPECT_LT(velocity_with_zupt, velocity_without_zupt);
    // The uncorrected case should show clearly unbounded-looking drift
    // (0.05 m/s^2 * 5 s ~= 0.25 m/s of integrated velocity error).
    EXPECT_GT(velocity_without_zupt, 0.1);
    EXPECT_LT(velocity_with_zupt, 0.05);
}

TEST(FusionZuptTest, GyroAndAccelBiasRemainCloseToTrueValuesDuringExtendedStationaryRun) {
    LooseCouplingProcessor processor(makeConfig(/*zupt_enable=*/true));
    const Eigen::Vector3d true_accel_bias(0.02, -0.01, 0.015);
    const Eigen::Vector3d true_gyro_bias(0.002, -0.0015, 0.001);

    feedStationaryRun(processor, 150, true_accel_bias, true_gyro_bias, 0.05, 0.005);
    feedStationaryRun(processor, 1000, true_accel_bias, true_gyro_bias, 0.05, 0.005);

    const Eigen::Vector3d gyro_bias_error = processor.state().nominal.gyro_bias - true_gyro_bias;
    const Eigen::Vector3d accel_bias_error = processor.state().nominal.accel_bias - true_accel_bias;
    EXPECT_LT(gyro_bias_error.norm(), 0.01);
    EXPECT_LT(accel_bias_error.norm(), 0.1);
}

TEST(FusionZuptTest, FreshMovingGnssVelocitySuppressesFalseZupt) {
    LooseCouplingProcessor processor(makeConfig(/*zupt_enable=*/true));
    feedStationaryRun(processor, 150, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                       0.0, 0.0);
    const std::size_t before = processor.zuptUpdateCount();
    processor.processGnssSolution(
        makeGnssSolution(processor, Eigen::Vector3d(2.0, 0.0, 0.0), true));
    feedQuietSamples(processor, 40);
    EXPECT_EQ(processor.zuptUpdateCount(), before)
        << "fresh GNSS speed above the gate must block a stationary-IMU false ZUPT";
}

TEST(FusionZuptTest, FreshLowSpeedGnssVelocityKeepsLegacyZuptBehavior) {
    LooseCouplingProcessor processor(makeConfig(/*zupt_enable=*/true));
    feedStationaryRun(processor, 150, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                       0.0, 0.0);
    const std::size_t before = processor.zuptUpdateCount();
    processor.processGnssSolution(
        makeGnssSolution(processor, Eigen::Vector3d(0.2, 0.0, 0.0), true));
    feedQuietSamples(processor, 40);
    EXPECT_GT(processor.zuptUpdateCount(), before)
        << "fresh GNSS speed below the gate must preserve ZUPT behavior";
}

TEST(FusionZuptTest, LastKnownMovingGnssVelocityContinuesSuppressingZupt) {
    LooseCouplingProcessor processor(makeConfig(/*zupt_enable=*/true));
    feedStationaryRun(processor, 150, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                       0.0, 0.0);
    const std::size_t before = processor.zuptUpdateCount();
    processor.processGnssSolution(
        makeGnssSolution(processor, Eigen::Vector3d(2.0, 0.0, 0.0), true));
    feedQuietSamples(processor, 140);
    EXPECT_EQ(processor.zuptUpdateCount(), before)
        << "a stale last-known moving velocity must keep suppressing false ZUPT";
}

TEST(FusionZuptTest, FreshLowSpeedGnssVelocityReleasesMovingLatch) {
    LooseCouplingProcessor processor(makeConfig(/*zupt_enable=*/true));
    feedStationaryRun(processor, 150, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                       0.0, 0.0);
    const std::size_t before = processor.zuptUpdateCount();
    processor.processGnssSolution(
        makeGnssSolution(processor, Eigen::Vector3d(2.0, 0.0, 0.0), true));
    feedQuietSamples(processor, 40);
    EXPECT_EQ(processor.zuptUpdateCount(), before);
    processor.processGnssSolution(
        makeGnssSolution(processor, Eigen::Vector3d(0.2, 0.0, 0.0), true));
    feedQuietSamples(processor, 40);
    EXPECT_GT(processor.zuptUpdateCount(), before)
        << "a fresh low-speed velocity must release the moving latch";
}

TEST(FusionZuptTest, PositionOnlyGnssKeepsLegacyZuptBehavior) {
    LooseCouplingProcessor processor(makeConfig(/*zupt_enable=*/true));
    feedStationaryRun(processor, 150, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                       0.0, 0.0);
    const std::size_t before = processor.zuptUpdateCount();
    processor.processGnssSolution(
        makeGnssSolution(processor, Eigen::Vector3d::Zero(), false));
    feedQuietSamples(processor, 40);
    EXPECT_GT(processor.zuptUpdateCount(), before)
        << "without a GNSS velocity, the compatibility path must allow ZUPT";
}

}  // namespace
}  // namespace libgnss
