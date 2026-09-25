#include <gtest/gtest.h>
#include <libgnss++/algorithms/source_phone_imu_noise.hpp>

TEST(SourcePhoneImuNoise, SourcePhoneBranchesAndElapsedClockScaling) {
    struct Case { const char* phone; double accel; double gyro; };
    const Case cases[] = {
        {"pixel4", .05, .001}, {"pixel4xl", .05, .001},
        {"pixel5", .05, .001}, {"pixel6pro", .05, .001},
        {"pixel7pro", .05, .001}, {"sm-g988b", .05, .001},
        {"sm-s908b", .05, .001}, {"samsunga32", .05, .001},
        {"samsunga325g", .05, .001}, {"sm-a325f", .1, .001},
        {"sm-a205u", .1, .001}, {"sm-a505u", .1, .001},
        {"mi8", .05, .001}, {"xiaomimi8", .05, .001},
        {"sm-a217m", .1, .005}, {"sm-g325f", .05, .001},
    };
    for (const auto& item : cases) {
        SCOPED_TRACE(item.phone);
        for (const bool fallback : {false, true}) {
            const auto result = libgnss::source_phone_imu_noise::select(item.phone, fallback);
            ASSERT_TRUE(result);
            const double coefficient = fallback ? 1.0 : .5;
            EXPECT_DOUBLE_EQ(result->accel_noise_sigma, item.accel * coefficient);
            EXPECT_DOUBLE_EQ(result->gyro_noise_sigma, item.gyro * coefficient);
            EXPECT_DOUBLE_EQ(result->sync_coefficient, coefficient);
        }
    }
}

TEST(SourcePhoneImuNoise, UnknownPhoneHasNoSilentPixelPreset) {
    EXPECT_FALSE(libgnss::source_phone_imu_noise::select("", true));
    EXPECT_FALSE(libgnss::source_phone_imu_noise::select("unknown", false));
}
