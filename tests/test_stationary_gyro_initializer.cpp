#include <gtest/gtest.h>
#include <libgnss++/algorithms/stationary_gyro_initializer.hpp>
#include <limits>

namespace {
std::vector<libgnss::ImuSample> fixture(std::size_t n) {
    std::vector<libgnss::ImuSample> samples(n);
    for (std::size_t i = 0; i < n; ++i) {
        samples[i].time = {2200, 100.0 + 0.02 * i};
        samples[i].accel_raw = Eigen::Vector3d(0, 0, 9.80665);
        samples[i].gyro_raw_radps = Eigen::Vector3d(.001, -.002, .003);
    }
    return samples;
}
}

TEST(StationaryGyroInitializer, RecoversBiasAfterInitiallyTurningInterval) {
    auto samples = fixture(2000);
    for (std::size_t i = 0; i < 500; ++i) samples[i].gyro_raw_radps.z() += .2;
    const auto result = libgnss::stationary_gyro::estimate(samples);
    EXPECT_GE(result.blocks, 2U);
    EXPECT_LT((result.bias_radps - Eigen::Vector3d(.001, -.002, .003)).norm(), 1e-14);
    EXPECT_LT(result.block_scatter_rms_radps, 1e-14);
    EXPECT_DOUBLE_EQ(samples.front().gyro_raw_radps.z(), .203);
}

TEST(StationaryGyroInitializer, RejectsInsufficientSupportAndAllTurning) {
    EXPECT_THROW(libgnss::stationary_gyro::estimate({}), std::invalid_argument);
    EXPECT_THROW(libgnss::stationary_gyro::estimate(fixture(499)), std::invalid_argument);
    auto samples = fixture(2000);
    for (auto& sample : samples) sample.gyro_raw_radps.z() += .2;
    EXPECT_THROW(libgnss::stationary_gyro::estimate(samples), std::invalid_argument);
}

TEST(StationaryGyroInitializer, DoesNotBridgeGapsToFabricateBlocks) {
    auto samples = fixture(1000);
    for (std::size_t i = 0; i < samples.size(); ++i)
        samples[i].time.tow += static_cast<double>(i / 200);
    EXPECT_THROW(libgnss::stationary_gyro::estimate(samples), std::invalid_argument);
}

TEST(StationaryGyroInitializer, RejectsNonfiniteAndNonmonotonicSamples) {
    auto samples = fixture(1000);
    samples[400].gyro_raw_radps.x() = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(libgnss::stationary_gyro::estimate(samples), std::invalid_argument);
    samples = fixture(1000);
    samples[400].time = samples[399].time;
    EXPECT_THROW(libgnss::stationary_gyro::estimate(samples), std::invalid_argument);
}

TEST(StationaryGyroInitializer, FixedRotationRotatesEstimate) {
    auto samples = fixture(1000);
    const Eigen::Matrix3d rotation = Eigen::AngleAxisd(.7, Eigen::Vector3d::UnitX()).toRotationMatrix();
    const auto before = libgnss::stationary_gyro::estimate(samples);
    for (auto& sample : samples) {
        sample.accel_raw = rotation * sample.accel_raw;
        sample.gyro_raw_radps = rotation * sample.gyro_raw_radps;
    }
    const auto after = libgnss::stationary_gyro::estimate(samples);
    EXPECT_EQ(before.blocks, after.blocks);
    EXPECT_LT((after.bias_radps - rotation * before.bias_radps).norm(), 1e-14);
}
