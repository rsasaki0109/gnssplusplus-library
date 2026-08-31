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
