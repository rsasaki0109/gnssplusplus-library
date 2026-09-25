#include <gtest/gtest.h>
#include <libgnss++/algorithms/source_mi8_clock_drift.hpp>

using libgnss::smartphone_temporal::mi8ClockDrift;

TEST(SourceMi8ClockDrift, UsesUnitSampleGradientWithOneSidedEndpoints) {
    const auto r = mi8ClockDrift({0, 1, 4, 9});
    ASSERT_TRUE(r.ok);
    EXPECT_EQ(r.values, (std::vector<double>{1, 2, 4, 5}));
    EXPECT_EQ(r.filled_values, 0U);
}

TEST(SourceMi8ClockDrift, MasksLargeGradientAndFillsFromFiniteNeighbours) {
    const auto r = mi8ClockDrift({0, 2, 5004, 5006, 5008});
    ASSERT_TRUE(r.ok);
    EXPECT_EQ(r.magnitude_masks, 2U);
    EXPECT_EQ(r.values, (std::vector<double>{2, 2, 2, 2, 2}));
}

TEST(SourceMi8ClockDrift, MasksBothSidesOfDiscontinuityBeforeFilling) {
    const auto r = mi8ClockDrift({0, 0, 0, 200, 400, 600, 600, 600});
    ASSERT_TRUE(r.ok);
    EXPECT_EQ(r.jump_masks, 6U);
    EXPECT_EQ(r.values, std::vector<double>(8, 0));
}

TEST(SourceMi8ClockDrift, LinearExtrapolationAtEdgesAndNoInventedAllMissingDrift) {
    const auto r = mi8ClockDrift({0, 10000, 10004, 10010, 10018});
    ASSERT_TRUE(r.ok);
    EXPECT_EQ(r.magnitude_masks, 2U);
    EXPECT_NEAR(r.values[0], 1, 1e-12);
    EXPECT_NEAR(r.values[1], 3, 1e-12);
    EXPECT_FALSE(mi8ClockDrift({0, 10000, 20000}).ok);
    EXPECT_FALSE(mi8ClockDrift({0}).ok);
    EXPECT_FALSE(mi8ClockDrift({0, std::numeric_limits<double>::quiet_NaN()}).ok);
}
