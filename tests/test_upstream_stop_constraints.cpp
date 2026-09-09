#include <gtest/gtest.h>

#include <libgnss++/algorithms/upstream_stop_constraints.hpp>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace {

libgnss::ImuSample sample(double tow, double accel_norm, double gyro_norm) {
    libgnss::ImuSample value;
    value.time = libgnss::GNSSTime(2200, tow);
    value.accel_raw = libgnss::Vector3d(accel_norm, 0.0, 0.0);
    value.gyro_raw_radps = libgnss::Vector3d(gyro_norm, 0.0, 0.0);
    return value;
}

std::vector<libgnss::ImuSample> stationarySamples(std::size_t count = 600) {
    std::vector<libgnss::ImuSample> samples;
    samples.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        samples.push_back(sample(100.0 + 0.01 * static_cast<double>(i),
                                 9.80665, 0.0));
    }
    return samples;
}

void expectCenteredImpulseWindow(std::size_t window_samples,
                                 std::size_t impulse_index,
                                 std::size_t inspected_index,
                                 std::size_t sample_count) {
    auto samples = stationarySamples(sample_count);
    samples[impulse_index].accel_raw.x() = 20.0;
    libgnss::upstream_stop::Config config;
    config.window_samples = window_samples;
    const auto result = libgnss::upstream_stop::detect(
        samples,
        {{2200, 100.0 + 0.01 * static_cast<double>(inspected_index)}},
        config);
    ASSERT_TRUE(result.ok) << result.error;
    // The impulse is just outside the MATLAB centered window at the
    // inspected sample.  A right-shifted even window would include it.
    ASSERT_LT(inspected_index, result.imu_stop.size());
    EXPECT_TRUE(result.imu_stop[inspected_index]);
}

}  // namespace

TEST(UpstreamStopConstraints, MirrorsGlobalMinimumWindowThresholds) {
    const auto samples = stationarySamples();
    const std::vector<libgnss::GNSSTime> epochs = {
        {2200, 100.00}, {2200, 100.25}, {2200, 104.00}};
    const auto result = libgnss::upstream_stop::detect(samples, epochs);
    ASSERT_TRUE(result.ok) << result.error;
    EXPECT_EQ(result.finite_samples, samples.size());
    EXPECT_EQ(result.stop_samples, samples.size());
    EXPECT_EQ(result.stop_epochs, epochs.size());
    EXPECT_DOUBLE_EQ(result.acceleration_min_std_mps2, 0.0);
    EXPECT_DOUBLE_EQ(result.gyro_min_std_radps, 0.0);
    EXPECT_DOUBLE_EQ(result.acceleration_std_threshold_mps2, 0.08);
    EXPECT_DOUBLE_EQ(result.gyro_std_threshold_radps, 0.005);
}

TEST(UpstreamStopConstraints, GyroNormGateRejectsMovingSamples) {
    auto samples = stationarySamples();
    samples[300].gyro_raw_radps.x() = 0.050001;
    const auto result = libgnss::upstream_stop::detect(
        samples, {{2200, 103.00}});
    ASSERT_TRUE(result.ok) << result.error;
    EXPECT_FALSE(result.imu_stop[300]);
    EXPECT_TRUE(result.imu_stop[299]);
    EXPECT_TRUE(result.imu_stop[301]);
}

TEST(UpstreamStopConstraints, NearestMappingUsesPreviousAtExactTie) {
    std::vector<libgnss::ImuSample> altered = {
        sample(100.0, 9.80665, 0.050001),
        sample(101.0, 9.80665, 0.0)};
    libgnss::upstream_stop::Config config;
    config.window_samples = 2;
    const auto result = libgnss::upstream_stop::detect(
        altered, {{2200, 100.5}}, config);
    ASSERT_TRUE(result.ok) << result.error;
    EXPECT_FALSE(result.epoch_stop[0]);
}

TEST(UpstreamStopConstraints, EvenWindowsUsePreviousSideAndSingletonIsZero) {
    // These asymmetric impulses distinguish MATLAB's even centered windows
    // from a one-sample-right implementation at the pinned production sizes.
    expectCenteredImpulseWindow(2, 3, 2, 5);
    expectCenteredImpulseWindow(4, 4, 2, 8);
    expectCenteredImpulseWindow(500, 500, 250, 600);

    // k=2 at the first endpoint is a singleton.  MATLAB's sample-normalized
    // movstd returns zero there, so a stationary sample remains a stop.
    auto samples = stationarySamples(2);
    libgnss::upstream_stop::Config config;
    config.window_samples = 2;
    const auto result = libgnss::upstream_stop::detect(
        samples, {{2200, 100.0}}, config);
    ASSERT_TRUE(result.ok) << result.error;
    ASSERT_EQ(result.imu_stop.size(), 2U);
    EXPECT_TRUE(result.imu_stop.front());
}

TEST(UpstreamStopConstraints, OddWindowRemainsSymmetric) {
    // k=3 is unchanged by the even-window correction: sample 4 is outside
    // the centered window around sample 2.
    expectCenteredImpulseWindow(3, 4, 2, 8);
}

TEST(UpstreamStopConstraints, RejectsNonFiniteAndUnorderedInputs) {
    auto samples = stationarySamples();
    samples[10].accel_raw.x() = std::numeric_limits<double>::quiet_NaN();
    auto result = libgnss::upstream_stop::detect(samples, {{2200, 100.0}});
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.error.find("non-finite"), std::string::npos);

    samples = stationarySamples();
    std::swap(samples[10], samples[11]);
    result = libgnss::upstream_stop::detect(samples, {{2200, 100.0}});
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.error.find("strictly increasing"), std::string::npos);
}

TEST(UpstreamStopConstraints, RejectsInvalidConfiguration) {
    auto samples = stationarySamples();
    libgnss::upstream_stop::Config config;
    config.window_samples = 1;
    const auto result = libgnss::upstream_stop::detect(
        samples, {{2200, 100.0}}, config);
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.error.find("configuration"), std::string::npos);
}
