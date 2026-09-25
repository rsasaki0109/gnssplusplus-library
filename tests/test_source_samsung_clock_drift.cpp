#include <gtest/gtest.h>
#include <libgnss++/algorithms/source_samsung_clock_drift.hpp>

using namespace libgnss;
using namespace libgnss::smartphone_temporal;

TEST(SourceSamsungDrift, OnlySourceListedPhones) {
    EXPECT_TRUE(samsungClockRecipe("sm-a205u"));
    EXPECT_TRUE(samsungClockRecipe("samsunga325g"));
    EXPECT_FALSE(samsungClockRecipe("sm-g988b"));
    EXPECT_FALSE(samsungClockRecipe("pixel5"));
    EXPECT_FALSE(samsungClockRecipe("mi8"));
}

TEST(SourceSamsungDrift, MissingWithoutJumpsUsesNearestIncludingTieAndEdges) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const auto r = finishSamsungDrift({nan, 10, nan, 20, nan});
    ASSERT_TRUE(r.ok);
    EXPECT_EQ(r.values, (std::vector<double>{10, 10, 20, 20, 20}));
    EXPECT_EQ(r.jump_masks, 0u);
    EXPECT_EQ(r.filled_values, 3u);
    EXPECT_TRUE(std::isnan(r.medians[2]));
}

TEST(SourceSamsungDrift, JumpMasksBothSidesThenLinearFillsAndExtrapolates) {
    const auto r = finishSamsungDrift({0, 1, 200, 3, 4});
    ASSERT_TRUE(r.ok);
    EXPECT_EQ(r.values, (std::vector<double>{0, 1, 2, 3, 4}));
    EXPECT_EQ(r.jump_masks, 3u);
    const auto edge = finishSamsungDrift({200, 1, 2, 3});
    ASSERT_TRUE(edge.ok);
    EXPECT_EQ(edge.values, (std::vector<double>{0, 1, 2, 3}));
    EXPECT_EQ(edge.jump_masks, 2u);
}

TEST(SourceSamsungDrift, ThresholdIsStrictAndNoAnchorFailsClosed) {
    const auto boundary = finishSamsungDrift({0, 50, 100});
    ASSERT_TRUE(boundary.ok);
    EXPECT_EQ(boundary.jump_masks, 0u);
    EXPECT_FALSE(finishSamsungDrift({0, 100}).ok);
    EXPECT_FALSE(finishSamsungDrift({}).ok);
    EXPECT_FALSE(finishSamsungDrift({std::numeric_limits<double>::quiet_NaN()}).ok);
}

TEST(SourceSamsungDrift, RawPhoneLeadingMissingnessRecordsActualDonor) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double drift = 15.0 * constants::SPEED_OF_LIGHT / 1e9;
    const auto r = finishSamsungDrift({nan, nan, nan, nan, nan, drift, drift, 3.9});
    ASSERT_TRUE(r.ok);
    EXPECT_EQ(r.jump_masks, 0u);
    EXPECT_EQ(r.filled_values, 5u);
    for (std::size_t i = 0; i < 5; ++i) {
        EXPECT_TRUE(std::isnan(r.medians[i]));
        EXPECT_DOUBLE_EQ(r.values[i], drift);
        EXPECT_EQ(r.left_sources[i], 5u);
        EXPECT_EQ(r.right_sources[i], 5u);
        EXPECT_DOUBLE_EQ(r.right_weights[i], 0.0);
    }
    for (std::size_t i = 5; i < 8; ++i) {
        EXPECT_DOUBLE_EQ(r.values[i], r.medians[i]);
        EXPECT_EQ(r.left_sources[i], i);
        EXPECT_EQ(r.right_sources[i], i);
    }
    const auto linear = finishSamsungDrift({0, 1, 200, 3, 4});
    EXPECT_EQ(linear.left_sources[2], 0u);
    EXPECT_EQ(linear.right_sources[2], 4u);
    EXPECT_DOUBLE_EQ(linear.right_weights[2], .5);
}

TEST(SourceSamsungDrift, RemovingSatelliteAndReceiverMotionRecoversDriftSignUnits) {
    const Vector3d los(0.6, 0.8, 0.0), velocity(12, -3, 0);
    const double satellite_rate = -450, clock_seconds_per_second = 2e-9;
    const double drift = 206;
    const double measured = satellite_rate - clock_seconds_per_second * constants::SPEED_OF_LIGHT
                            - los.dot(velocity) + drift;
    EXPECT_NEAR(doppler_contract::receiverOnlyResidual(
        measured, satellite_rate, clock_seconds_per_second) + los.dot(velocity), drift, 1e-12);
}
