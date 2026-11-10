#include <gtest/gtest.h>

#include <libgnss++/algorithms/forward_backward.hpp>

#include <vector>

using namespace libgnss;

namespace {

const Eigen::Vector3d kOrigin(-3961904.0, 3348993.0, 3698211.0);

PositionSolution makeSolution(double tow, SolutionStatus status, const Eigen::Vector3d& offset,
                              double variance_m2) {
    PositionSolution solution;
    solution.time = GNSSTime(2300, tow);
    solution.status = status;
    solution.num_satellites = 10;
    solution.position_ecef = kOrigin + offset;
    solution.position_covariance = Eigen::Matrix3d::Identity() * variance_m2;
    return solution;
}

}  // namespace

TEST(ForwardBackwardTest, BetterStatusWinsAndTheBackwardPassMayBeInReverseOrder) {
    forward_backward::Pass forward;
    forward.solutions = {
        makeSolution(0.0, SolutionStatus::FLOAT, Eigen::Vector3d(0.3, 0.0, 0.0), 0.04),
        makeSolution(1.0, SolutionStatus::FIXED, Eigen::Vector3d(0.0, 0.0, 0.0), 0.01),
    };
    forward_backward::Pass backward;  // processing order: latest first
    backward.solutions = {
        makeSolution(1.0, SolutionStatus::FLOAT, Eigen::Vector3d(0.5, 0.0, 0.0), 0.04),
        makeSolution(0.0, SolutionStatus::FIXED, Eigen::Vector3d(0.01, 0.0, 0.0), 0.01),
    };
    forward_backward::CombineStats stats;
    const auto combined = forward_backward::combine(forward, backward, {}, &stats);
    ASSERT_EQ(combined.size(), 2U);
    EXPECT_DOUBLE_EQ(combined[0].time.tow, 0.0);
    EXPECT_EQ(combined[0].status, SolutionStatus::FIXED);  // from the backward pass
    EXPECT_NEAR((combined[0].position_ecef - kOrigin).x(), 0.01, 1e-9);
    EXPECT_EQ(combined[1].status, SolutionStatus::FIXED);  // from the forward pass
    EXPECT_NEAR((combined[1].position_ecef - kOrigin).norm(), 0.0, 1e-9);
    EXPECT_EQ(stats.both, 2U);
    EXPECT_EQ(stats.from_backward, 1U);
    EXPECT_EQ(stats.from_forward, 1U);
    EXPECT_EQ(stats.fix_upgrades, 1U);
    EXPECT_EQ(stats.averaged, 0U);
}

TEST(ForwardBackwardTest, EqualStatusIsAveragedWithTheWeights) {
    forward_backward::Pass forward;
    forward.solutions = {makeSolution(0.0, SolutionStatus::FLOAT, Eigen::Vector3d(1.0, 0.0, 0.0), 9.0)};
    // The weights replace the solutions' own covariance: 1 m^2 forward,
    // 3 m^2 backward.
    forward.weights = {Eigen::Matrix3d::Identity()};
    forward_backward::Pass backward;
    backward.solutions = {makeSolution(0.0, SolutionStatus::FLOAT, Eigen::Vector3d(-1.0, 0.0, 2.0), 9.0)};
    backward.weights = {Eigen::Matrix3d::Identity() * 3.0};
    forward_backward::CombineStats stats;
    const auto combined = forward_backward::combine(forward, backward, {}, &stats);
    ASSERT_EQ(combined.size(), 1U);
    EXPECT_EQ(combined[0].status, SolutionStatus::FLOAT);
    // x = 0.75 xf + 0.25 xb, variance 1 * 3 / 4.
    const Eigen::Vector3d offset = combined[0].position_ecef - kOrigin;
    EXPECT_NEAR(offset.x(), 0.5, 1e-9);
    EXPECT_NEAR(offset.z(), 0.5, 1e-9);
    EXPECT_NEAR(combined[0].position_covariance(0, 0), 0.75, 1e-12);
    EXPECT_NEAR(combined[0].position_covariance(0, 1), 0.0, 1e-12);
    EXPECT_EQ(stats.averaged, 1U);
}

TEST(ForwardBackwardTest, TwoFixesFurtherApartThanTheToleranceBecomeFloat) {
    forward_backward::Pass forward;
    forward.solutions = {
        makeSolution(0.0, SolutionStatus::FIXED, Eigen::Vector3d(0.0, 0.0, 0.0), 0.01),
        makeSolution(1.0, SolutionStatus::FIXED, Eigen::Vector3d(0.0, 0.0, 0.0), 0.01),
    };
    forward_backward::Pass backward;
    backward.solutions = {
        makeSolution(1.0, SolutionStatus::FIXED, Eigen::Vector3d(0.0, 0.19, 0.0), 0.01),
        makeSolution(0.0, SolutionStatus::FIXED, Eigen::Vector3d(0.02, 0.0, 0.0), 0.01),
    };
    forward_backward::CombineConfig config;
    config.max_fix_difference_m = 0.05;
    forward_backward::CombineStats stats;
    const auto combined = forward_backward::combine(forward, backward, config, &stats);
    ASSERT_EQ(combined.size(), 2U);
    EXPECT_EQ(combined[0].status, SolutionStatus::FIXED);
    EXPECT_NEAR((combined[0].position_ecef - kOrigin).x(), 0.01, 1e-9);
    EXPECT_EQ(combined[1].status, SolutionStatus::FLOAT);
    EXPECT_NEAR((combined[1].position_ecef - kOrigin).y(), 0.095, 1e-9);
    EXPECT_EQ(stats.fix_inconsistent, 1U);
    EXPECT_EQ(stats.averaged, 2U);
}

TEST(ForwardBackwardTest, AnEpochOfOnePassIsKeptAndInvalidOnesAreIgnored) {
    forward_backward::Pass forward;
    forward.solutions = {
        makeSolution(0.0, SolutionStatus::SPP, Eigen::Vector3d(2.0, 0.0, 0.0), 25.0),
        makeSolution(1.0, SolutionStatus::NONE, Eigen::Vector3d::Zero(), 0.01),
    };
    forward_backward::Pass backward;
    backward.solutions = {
        makeSolution(2.0, SolutionStatus::FIXED, Eigen::Vector3d::Zero(), 0.01),
        makeSolution(1.0, SolutionStatus::FLOAT, Eigen::Vector3d(0.1, 0.0, 0.0), 0.04),
    };
    forward_backward::CombineStats stats;
    const auto combined = forward_backward::combine(forward, backward, {}, &stats);
    ASSERT_EQ(combined.size(), 3U);
    EXPECT_EQ(combined[0].status, SolutionStatus::SPP);
    EXPECT_EQ(combined[1].status, SolutionStatus::FLOAT);
    // A FIX the other pass has no RTK solution for is reported as FLOAT.
    EXPECT_EQ(combined[2].status, SolutionStatus::FLOAT);
    EXPECT_NEAR((combined[2].position_ecef - kOrigin).norm(), 0.0, 1e-9);
    EXPECT_EQ(stats.forward_only, 1U);
    EXPECT_EQ(stats.backward_only, 2U);
    EXPECT_EQ(stats.both, 0U);
    EXPECT_EQ(stats.fix_upgrades, 0U);
    EXPECT_EQ(stats.fix_unconfirmed, 1U);
    EXPECT_EQ(forward_backward::statusRank(forward.solutions[1]), -1);

    forward_backward::CombineConfig keep;
    keep.require_rtk_confirmation = false;
    EXPECT_EQ(forward_backward::combine(forward, backward, keep)[2].status,
              SolutionStatus::FIXED);
}

TEST(ForwardBackwardTest, AFixOverAnSppSolutionIsReportedAsFloat) {
    forward_backward::Pass forward;
    forward.solutions = {makeSolution(0.0, SolutionStatus::FIXED, Eigen::Vector3d::Zero(), 0.01)};
    forward_backward::Pass backward;
    backward.solutions = {makeSolution(0.0, SolutionStatus::SPP, Eigen::Vector3d(3.0, 0.0, 0.0), 25.0)};
    forward_backward::CombineStats stats;
    const auto combined = forward_backward::combine(forward, backward, {}, &stats);
    ASSERT_EQ(combined.size(), 1U);
    EXPECT_EQ(combined[0].status, SolutionStatus::FLOAT);
    EXPECT_NEAR((combined[0].position_ecef - kOrigin).norm(), 0.0, 1e-9);
    EXPECT_EQ(stats.from_forward, 1U);
    EXPECT_EQ(stats.fix_unconfirmed, 1U);
}

namespace {

ObservationData epochWith(const std::vector<std::pair<SignalType, bool>>& signals) {
    ObservationData epoch;
    for (const auto& [signal, slip] : signals) {
        Observation observation;
        observation.satellite = SatelliteId(GNSSSystem::GPS, 5);
        observation.signal = signal;
        observation.lli = slip ? 0x01 : 0x00;
        epoch.observations.push_back(observation);
    }
    return epoch;
}

bool slipOf(const ObservationData& epoch, SignalType signal) {
    for (const auto& observation : epoch.observations) {
        if (observation.signal == signal) {
            return observation.loss_of_lock && (observation.lli & 0x01) != 0;
        }
    }
    ADD_FAILURE() << "signal not in the epoch";
    return false;
}

}  // namespace

TEST(ForwardBackwardTest, BackwardLossOfLockMovesEachSlipToTheEarlierEpoch) {
    // Times t0 < t1 < t2, applied backward: t2, t1, t0. L1 slips at t2 (since
    // t1): going backward the slip lies before t1.
    auto t2 = epochWith({{SignalType::GPS_L1CA, true}, {SignalType::GPS_L2C, true}});
    auto t1 = epochWith({{SignalType::GPS_L1CA, false}});  // no L2 at t1
    auto t0 = epochWith({{SignalType::GPS_L1CA, false}, {SignalType::GPS_L2C, false}});
    forward_backward::BackwardLossOfLock lol;
    lol.apply(t2);
    lol.apply(t1);
    lol.apply(t0);
    EXPECT_FALSE(slipOf(t2, SignalType::GPS_L1CA));  // nothing after t2
    EXPECT_FALSE(slipOf(t2, SignalType::GPS_L2C));
    EXPECT_TRUE(slipOf(t1, SignalType::GPS_L1CA));
    EXPECT_FALSE(slipOf(t0, SignalType::GPS_L1CA));
    // L2 was missing at t1: its slip is kept for its next earlier epoch.
    EXPECT_TRUE(slipOf(t0, SignalType::GPS_L2C));
}
