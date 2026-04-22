#include <gtest/gtest.h>

#include <libgnss++/algorithms/rtk_validation.hpp>

#include <limits>
#include <vector>

using namespace libgnss;

namespace {

PositionSolution makeValidationSolution(double tow,
                                        SolutionStatus status,
                                        const Eigen::Vector3d& position,
                                        double height_m = 0.0) {
    PositionSolution solution;
    solution.time = GNSSTime(2300, tow);
    solution.status = status;
    solution.position_ecef = position;
    solution.position_geodetic.height = height_m;
    solution.num_satellites = 8;
    return solution;
}

}  // namespace

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

TEST(RTKValidationTest, NonFixedDriftGuardRejectsLowSpeedBoundedOutliers) {
    std::vector<PositionSolution> solutions = {
        makeValidationSolution(0.0, SolutionStatus::FIXED, Eigen::Vector3d(0.0, 0.0, 0.0)),
        makeValidationSolution(1.0, SolutionStatus::FLOAT, Eigen::Vector3d(0.5, 0.0, 0.0)),
        makeValidationSolution(2.0, SolutionStatus::FLOAT, Eigen::Vector3d(1.0, 40.0, 0.0)),
        makeValidationSolution(3.0, SolutionStatus::SPP, Eigen::Vector3d(1.5, 0.0, 0.0)),
        makeValidationSolution(4.0, SolutionStatus::FIXED, Eigen::Vector3d(2.0, 0.0, 0.0)),
    };

    rtk_validation::NonFixedDriftGuardConfig config;
    config.max_anchor_gap_s = 10.0;
    config.max_anchor_speed_mps = 1.0;
    config.max_residual_m = 10.0;
    config.min_segment_epochs = 2;

    const auto result = rtk_validation::filterNonFixedStationaryDrift(solutions, config);

    EXPECT_EQ(result.inspected_segments, 1);
    EXPECT_EQ(result.rejected_segments, 1);
    EXPECT_EQ(result.rejected_epochs, 1);
    ASSERT_EQ(result.solutions.size(), 4U);
    EXPECT_DOUBLE_EQ(result.solutions[0].time.tow, 0.0);
    EXPECT_DOUBLE_EQ(result.solutions[1].time.tow, 1.0);
    EXPECT_DOUBLE_EQ(result.solutions[2].time.tow, 3.0);
    EXPECT_DOUBLE_EQ(result.solutions[3].time.tow, 4.0);
}

TEST(RTKValidationTest, NonFixedDriftGuardIgnoresMovingOrShortSegments) {
    rtk_validation::NonFixedDriftGuardConfig config;
    config.max_anchor_gap_s = 10.0;
    config.max_anchor_speed_mps = 1.0;
    config.max_residual_m = 10.0;
    config.min_segment_epochs = 3;

    std::vector<PositionSolution> moving_segment = {
        makeValidationSolution(0.0, SolutionStatus::FIXED, Eigen::Vector3d(0.0, 0.0, 0.0)),
        makeValidationSolution(1.0, SolutionStatus::FLOAT, Eigen::Vector3d(25.0, 40.0, 0.0)),
        makeValidationSolution(2.0, SolutionStatus::FLOAT, Eigen::Vector3d(50.0, 40.0, 0.0)),
        makeValidationSolution(3.0, SolutionStatus::FLOAT, Eigen::Vector3d(75.0, 40.0, 0.0)),
        makeValidationSolution(4.0, SolutionStatus::FIXED, Eigen::Vector3d(100.0, 0.0, 0.0)),
    };

    const auto moving_result =
        rtk_validation::filterNonFixedStationaryDrift(moving_segment, config);
    EXPECT_EQ(moving_result.inspected_segments, 0);
    EXPECT_EQ(moving_result.rejected_epochs, 0);
    EXPECT_EQ(moving_result.solutions.size(), moving_segment.size());

    std::vector<PositionSolution> short_segment = {
        makeValidationSolution(0.0, SolutionStatus::FIXED, Eigen::Vector3d(0.0, 0.0, 0.0)),
        makeValidationSolution(1.0, SolutionStatus::FLOAT, Eigen::Vector3d(0.5, 40.0, 0.0)),
        makeValidationSolution(2.0, SolutionStatus::FLOAT, Eigen::Vector3d(1.0, 40.0, 0.0)),
        makeValidationSolution(3.0, SolutionStatus::FIXED, Eigen::Vector3d(1.5, 0.0, 0.0)),
    };

    const auto short_result =
        rtk_validation::filterNonFixedStationaryDrift(short_segment, config);
    EXPECT_EQ(short_result.inspected_segments, 0);
    EXPECT_EQ(short_result.rejected_epochs, 0);
    EXPECT_EQ(short_result.solutions.size(), short_segment.size());
}

TEST(RTKValidationTest, SppHeightStepGuardRejectsSppSpikeCluster) {
    std::vector<PositionSolution> solutions = {
        makeValidationSolution(0.0, SolutionStatus::FIXED, Eigen::Vector3d::Zero(), 10.0),
        makeValidationSolution(1.0, SolutionStatus::SPP, Eigen::Vector3d::Zero(), 80.0),
        makeValidationSolution(2.0, SolutionStatus::SPP, Eigen::Vector3d::Zero(), 82.0),
        makeValidationSolution(3.0, SolutionStatus::FLOAT, Eigen::Vector3d::Zero(), 85.0),
        makeValidationSolution(4.0, SolutionStatus::SPP, Eigen::Vector3d::Zero(), 88.0),
    };

    rtk_validation::SppHeightStepGuardConfig config;
    config.min_step_m = 30.0;
    config.max_rate_mps = 4.0;

    const auto result = rtk_validation::filterSppHeightSteps(solutions, config);

    EXPECT_EQ(result.rejected_epochs, 2);
    ASSERT_EQ(result.solutions.size(), 3U);
    EXPECT_DOUBLE_EQ(result.solutions[0].time.tow, 0.0);
    EXPECT_DOUBLE_EQ(result.solutions[1].time.tow, 3.0);
    EXPECT_DOUBLE_EQ(result.solutions[2].time.tow, 4.0);
}

TEST(RTKValidationTest, SppHeightStepGuardKeepsRateScaledSppStep) {
    std::vector<PositionSolution> solutions = {
        makeValidationSolution(0.0, SolutionStatus::FIXED, Eigen::Vector3d::Zero(), 10.0),
        makeValidationSolution(10.0, SolutionStatus::SPP, Eigen::Vector3d::Zero(), 45.0),
    };

    rtk_validation::SppHeightStepGuardConfig config;
    config.min_step_m = 30.0;
    config.max_rate_mps = 4.0;

    const auto result = rtk_validation::filterSppHeightSteps(solutions, config);

    EXPECT_EQ(result.rejected_epochs, 0);
    EXPECT_EQ(result.solutions.size(), solutions.size());
}

TEST(RTKValidationTest, FloatBridgeTailGuardRejectsSlowBoundedFloatResiduals) {
    std::vector<PositionSolution> solutions = {
        makeValidationSolution(0.0, SolutionStatus::FIXED, Eigen::Vector3d(0.0, 0.0, 0.0)),
        makeValidationSolution(10.0, SolutionStatus::FLOAT, Eigen::Vector3d(0.0, 10.0, 0.0)),
        makeValidationSolution(20.0, SolutionStatus::FLOAT, Eigen::Vector3d(0.0, 20.0, 15.0)),
        makeValidationSolution(30.0, SolutionStatus::SPP, Eigen::Vector3d(0.0, 30.0, 15.0)),
        makeValidationSolution(100.0, SolutionStatus::FIXED, Eigen::Vector3d(0.0, 100.0, 0.0)),
    };

    rtk_validation::FloatBridgeTailGuardConfig config;
    config.max_anchor_gap_s = 120.0;
    config.min_anchor_speed_mps = 0.4;
    config.max_anchor_speed_mps = 1.2;
    config.max_residual_m = 10.0;
    config.min_segment_epochs = 2;

    const auto result = rtk_validation::filterFloatBridgeTail(solutions, config);

    EXPECT_EQ(result.inspected_segments, 1);
    EXPECT_EQ(result.rejected_segments, 1);
    EXPECT_EQ(result.rejected_epochs, 1);
    ASSERT_EQ(result.solutions.size(), 4U);
    EXPECT_DOUBLE_EQ(result.solutions[0].time.tow, 0.0);
    EXPECT_DOUBLE_EQ(result.solutions[1].time.tow, 10.0);
    EXPECT_DOUBLE_EQ(result.solutions[2].time.tow, 30.0);
    EXPECT_DOUBLE_EQ(result.solutions[3].time.tow, 100.0);
}

TEST(RTKValidationTest, FloatBridgeTailGuardIgnoresFastAnchors) {
    std::vector<PositionSolution> solutions = {
        makeValidationSolution(0.0, SolutionStatus::FIXED, Eigen::Vector3d(0.0, 0.0, 0.0)),
        makeValidationSolution(10.0, SolutionStatus::FLOAT, Eigen::Vector3d(0.0, 10.0, 50.0)),
        makeValidationSolution(20.0, SolutionStatus::FLOAT, Eigen::Vector3d(0.0, 20.0, 50.0)),
        makeValidationSolution(100.0, SolutionStatus::FIXED, Eigen::Vector3d(0.0, 300.0, 0.0)),
    };

    rtk_validation::FloatBridgeTailGuardConfig config;
    config.max_anchor_gap_s = 120.0;
    config.min_anchor_speed_mps = 0.4;
    config.max_anchor_speed_mps = 1.2;
    config.max_residual_m = 10.0;
    config.min_segment_epochs = 2;

    const auto result = rtk_validation::filterFloatBridgeTail(solutions, config);

    EXPECT_EQ(result.inspected_segments, 0);
    EXPECT_EQ(result.rejected_epochs, 0);
    EXPECT_EQ(result.solutions.size(), solutions.size());
}

TEST(RTKValidationTest, FloatBridgeTailGuardUsesHorizontalAnchorSpeed) {
    std::vector<PositionSolution> solutions = {
        makeValidationSolution(0.0, SolutionStatus::FIXED, Eigen::Vector3d(0.0, 0.0, 0.0)),
        makeValidationSolution(10.0, SolutionStatus::FLOAT, Eigen::Vector3d(0.0, 5.0, 40.0)),
        makeValidationSolution(20.0, SolutionStatus::FLOAT, Eigen::Vector3d(0.0, 5.0, 45.0)),
        makeValidationSolution(50.0, SolutionStatus::FIXED, Eigen::Vector3d(50.0, 5.0, 0.0)),
    };

    rtk_validation::FloatBridgeTailGuardConfig config;
    config.max_anchor_gap_s = 120.0;
    config.min_anchor_speed_mps = 0.4;
    config.max_anchor_speed_mps = 1.2;
    config.max_residual_m = 10.0;
    config.min_segment_epochs = 2;

    const auto result = rtk_validation::filterFloatBridgeTail(solutions, config);

    EXPECT_EQ(result.inspected_segments, 0);
    EXPECT_EQ(result.rejected_epochs, 0);
    EXPECT_EQ(result.solutions.size(), solutions.size());
}

TEST(RTKValidationTest, FixedBridgeBurstGuardRejectsShortBoundedFixedOutliers) {
    std::vector<PositionSolution> solutions = {
        makeValidationSolution(0.0, SolutionStatus::FIXED, Eigen::Vector3d(0.0, 0.0, 0.0)),
        makeValidationSolution(3.0, SolutionStatus::FIXED, Eigen::Vector3d(30.0, 40.0, 0.0)),
        makeValidationSolution(3.2, SolutionStatus::FIXED, Eigen::Vector3d(32.0, 42.0, 0.0)),
        makeValidationSolution(6.0, SolutionStatus::FIXED, Eigen::Vector3d(60.0, 0.0, 0.0)),
    };

    rtk_validation::FixedBridgeBurstGuardConfig config;
    config.max_anchor_gap_s = 10.0;
    config.min_boundary_gap_s = 1.0;
    config.max_residual_m = 10.0;
    config.max_segment_epochs = 4;

    const auto result = rtk_validation::filterFixedBridgeBursts(solutions, config);

    EXPECT_EQ(result.inspected_segments, 1);
    EXPECT_EQ(result.rejected_segments, 1);
    EXPECT_EQ(result.rejected_epochs, 2);
    ASSERT_EQ(result.solutions.size(), 2U);
    EXPECT_DOUBLE_EQ(result.solutions[0].time.tow, 0.0);
    EXPECT_DOUBLE_EQ(result.solutions[1].time.tow, 6.0);
}

TEST(RTKValidationTest, FixedBridgeBurstGuardKeepsLongOrContinuousFixedRuns) {
    rtk_validation::FixedBridgeBurstGuardConfig config;
    config.max_anchor_gap_s = 10.0;
    config.min_boundary_gap_s = 1.0;
    config.max_residual_m = 10.0;
    config.max_segment_epochs = 2;

    std::vector<PositionSolution> continuous = {
        makeValidationSolution(0.0, SolutionStatus::FIXED, Eigen::Vector3d(0.0, 0.0, 0.0)),
        makeValidationSolution(0.2, SolutionStatus::FIXED, Eigen::Vector3d(0.2, 20.0, 0.0)),
        makeValidationSolution(0.4, SolutionStatus::FIXED, Eigen::Vector3d(0.4, 20.0, 0.0)),
        makeValidationSolution(0.6, SolutionStatus::FIXED, Eigen::Vector3d(0.6, 0.0, 0.0)),
    };
    const auto continuous_result = rtk_validation::filterFixedBridgeBursts(continuous, config);
    EXPECT_EQ(continuous_result.inspected_segments, 0);
    EXPECT_EQ(continuous_result.rejected_epochs, 0);
    EXPECT_EQ(continuous_result.solutions.size(), continuous.size());

    std::vector<PositionSolution> long_segment = {
        makeValidationSolution(0.0, SolutionStatus::FIXED, Eigen::Vector3d(0.0, 0.0, 0.0)),
        makeValidationSolution(2.0, SolutionStatus::FIXED, Eigen::Vector3d(20.0, 20.0, 0.0)),
        makeValidationSolution(2.2, SolutionStatus::FIXED, Eigen::Vector3d(22.0, 20.0, 0.0)),
        makeValidationSolution(2.4, SolutionStatus::FIXED, Eigen::Vector3d(24.0, 20.0, 0.0)),
        makeValidationSolution(5.0, SolutionStatus::FIXED, Eigen::Vector3d(50.0, 0.0, 0.0)),
    };
    const auto long_result = rtk_validation::filterFixedBridgeBursts(long_segment, config);
    EXPECT_EQ(long_result.inspected_segments, 0);
    EXPECT_EQ(long_result.rejected_epochs, 0);
    EXPECT_EQ(long_result.solutions.size(), long_segment.size());
}
