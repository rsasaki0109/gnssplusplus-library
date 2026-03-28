#include <gtest/gtest.h>

#include <libgnss++/algorithms/rtk_validation.hpp>

#include <limits>

using namespace libgnss;

TEST(RTKValidationTest, NormalizesNonFiniteOrShortDt) {
    EXPECT_DOUBLE_EQ(rtk_validation::normalizedDt(std::numeric_limits<double>::quiet_NaN()), 1.0);
    EXPECT_DOUBLE_EQ(rtk_validation::normalizedDt(0.1), 1.0);
    EXPECT_DOUBLE_EQ(rtk_validation::normalizedDt(2.5), 2.5);
}

TEST(RTKValidationTest, AdaptiveJumpUsesMinimumOrRateScaledThreshold) {
    const Eigen::Vector3d origin = Eigen::Vector3d::Zero();

    EXPECT_FALSE(rtk_validation::exceedsAdaptiveJump(
        Eigen::Vector3d(7.5, 0.0, 0.0), origin, 0.2, 8.0, 12.0));
    EXPECT_TRUE(rtk_validation::exceedsAdaptiveJump(
        Eigen::Vector3d(13.0, 0.0, 0.0), origin, 0.2, 8.0, 12.0));
    EXPECT_FALSE(rtk_validation::exceedsAdaptiveJump(
        Eigen::Vector3d(20.0, 0.0, 0.0), origin, 2.0, 8.0, 12.0));
    EXPECT_TRUE(rtk_validation::exceedsAdaptiveJump(
        Eigen::Vector3d(30.0, 0.0, 0.0), origin, 2.0, 8.0, 12.0));
}

TEST(RTKValidationTest, FixHistoryJumpAllowsEarlyReacquisitionButRejectsLaterJumps) {
    const Eigen::Vector3d previous_fix = Eigen::Vector3d::Zero();
    const Eigen::Vector3d jumped_fix(0.25, 0.0, 0.0);

    EXPECT_FALSE(rtk_validation::exceedsFixHistoryJump(
        jumped_fix, previous_fix, true, false, 2));
    EXPECT_TRUE(rtk_validation::exceedsFixHistoryJump(
        jumped_fix, previous_fix, true, false, 3));
    EXPECT_TRUE(rtk_validation::exceedsFixHistoryJump(
        Eigen::Vector3d(0.11, 0.0, 0.0), previous_fix, true, true, 3));
}

TEST(RTKValidationTest, HoldAttemptRequiresStableHistoryAndHeldIntegers) {
    EXPECT_FALSE(rtk_validation::canAttemptHoldFix(4, 5, true, true));
    EXPECT_FALSE(rtk_validation::canAttemptHoldFix(5, 5, false, true));
    EXPECT_FALSE(rtk_validation::canAttemptHoldFix(5, 5, true, false));
    EXPECT_TRUE(rtk_validation::canAttemptHoldFix(5, 5, true, true));
}
