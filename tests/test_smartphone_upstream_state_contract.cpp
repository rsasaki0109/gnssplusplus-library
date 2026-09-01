#include <gtest/gtest.h>

#include "../apps/native/upstream_state_contract.hpp"

namespace {

using libgnss_apps::upstream_state_contract::clockResidual;
using libgnss_apps::upstream_state_contract::kTimeDifferenceThresholdSeconds;
using libgnss_apps::upstream_state_contract::legacyUnitIntervalClockResidual;
using libgnss_apps::upstream_state_contract::legacyUnitIntervalMotionResidual;
using libgnss_apps::upstream_state_contract::motionResidual;
using libgnss_apps::upstream_state_contract::validTemporalInterval;

TEST(SmartphoneUpstreamStateContractTest, AcceptsOnlyFinitePositiveIntervalsBelowOnePointFive) {
    EXPECT_TRUE(validTemporalInterval(0.001));
    EXPECT_TRUE(validTemporalInterval(1.499999));
    EXPECT_FALSE(validTemporalInterval(kTimeDifferenceThresholdSeconds));
    EXPECT_FALSE(validTemporalInterval(1.500001));
    EXPECT_FALSE(validTemporalInterval(0.0));
    EXPECT_FALSE(validTemporalInterval(-0.1));
    EXPECT_FALSE(validTemporalInterval(std::numeric_limits<double>::quiet_NaN()));
    EXPECT_FALSE(validTemporalInterval(std::numeric_limits<double>::infinity()));
}

TEST(SmartphoneUpstreamStateContractTest, MotionMatchesPublishedMidpointEquation) {
    // x2-x1 = 10 m, midpoint velocity = 4 m/s, dt = 2.5 s.
    EXPECT_DOUBLE_EQ(motionResidual(10.0, 3.0, 5.0, 2.5), 0.0);
    EXPECT_DOUBLE_EQ(motionResidual(10.0, 3.0, 5.0, 1.0), 6.0);
    EXPECT_DOUBLE_EQ(legacyUnitIntervalMotionResidual(10.0, 3.0, 5.0), 6.0);
}

TEST(SmartphoneUpstreamStateContractTest, ClockMatchesPublishedMidpointEquation) {
    // c2-c1 = 12 m, midpoint drift = 4 m/s, dt = 3 s.
    EXPECT_DOUBLE_EQ(clockResidual(12.0, 3.0, 5.0, 3.0), 0.0);
    EXPECT_DOUBLE_EQ(clockResidual(12.0, 3.0, 5.0, 1.0), 8.0);
    EXPECT_DOUBLE_EQ(legacyUnitIntervalClockResidual(12.0, 3.0, 5.0), 8.0);
}

TEST(SmartphoneUpstreamStateContractTest, LargeCoordinateScaleDoesNotChangeEquation) {
    // Use receiver-scale ECEF offsets and clock magnitudes, not a near-zero
    // toy origin.  The contract is translation invariant in the correction.
    constexpr double seed_delta = 6'400'000.0 + 18.0 - 6'400'000.0;
    EXPECT_DOUBLE_EQ(motionResidual(seed_delta, 7.0, 11.0, 1.25), 6.75);
    EXPECT_DOUBLE_EQ(clockResidual(2'000'000.0, 400.0, 600.0, 1.5),
                      1'999'250.0);
}

}  // namespace
