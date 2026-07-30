#include <gtest/gtest.h>

#include <limits>

#include <libgnss++/algorithms/safe_float_continuity.hpp>
#include <libgnss++/algorithms/safe_fix_state_machine.hpp>

namespace {

using libgnss::safe_fix::Candidate;
using libgnss::safe_fix::Config;
using libgnss::safe_fix::State;
using libgnss::safe_fix::StateMachine;

Candidate candidate(double time_s, double x_m = 0.0) {
    Candidate value;
    value.time_s = time_s;
    value.acquisition_eligible = true;
    value.correction_m = Eigen::Vector3d(x_m, 0.0, 0.0);
    value.nis_per_observation = 1.0;
    value.prefit_residual_rms_m = 0.5;
    value.pair_count = 16;
    return value;
}

TEST(SafeFixStateMachineTest, DisabledIsAStatelessNoOp) {
    StateMachine machine;
    Config config;
    const auto decision = machine.update(config, candidate(1.0));
    EXPECT_EQ(decision.state, State::DISABLED);
    EXPECT_FALSE(decision.declared_fixed);
}

TEST(SafeFixStateMachineTest, RequiresContiguousConsistentAcquisition) {
    StateMachine machine;
    Config config;
    config.enabled = true;
    config.acquisition_streak_epochs = 2;
    EXPECT_EQ(machine.update(config, candidate(1.0)).state, State::ACQUIRING);
    const auto fixed = machine.update(config, candidate(1.2, 0.01));
    EXPECT_EQ(fixed.state, State::FIXED);
    EXPECT_TRUE(fixed.declared_fixed);
    EXPECT_TRUE(fixed.candidate_accepted);
}

TEST(SafeFixStateMachineTest, GapOrCorrectionJumpRestartsAcquisition) {
    StateMachine machine;
    Config config;
    config.enabled = true;
    config.acquisition_streak_epochs = 2;
    machine.update(config, candidate(1.0));
    EXPECT_EQ(machine.update(config, candidate(1.4)).acquisition_streak, 1);
    EXPECT_EQ(machine.update(config, candidate(1.6, 0.10)).acquisition_streak, 1);
}

TEST(SafeFixStateMachineTest, IndependentFailureBudgetFailsClosed) {
    StateMachine machine;
    Config config;
    config.enabled = true;
    config.acquisition_streak_epochs = 1;
    config.allow_strong_instant_acquisition = true;
    config.require_independent_failure_budget = true;
    auto sample = candidate(1.0);
    sample.strong_acquisition_eligible = true;

    const auto rejected = machine.update(config, sample);
    EXPECT_FALSE(rejected.declared_fixed);
    EXPECT_FALSE(rejected.independent_failure_budget_passed);

    sample.time_s = 1.2;
    sample.independent_failure_budget_passed = true;
    const auto accepted = machine.update(config, sample);
    EXPECT_TRUE(accepted.declared_fixed);
    EXPECT_TRUE(accepted.independent_failure_budget_passed);
}

TEST(SafeFixStateMachineTest, HoldsOnlyWithFiniteBoundedQualityThenRevokes) {
    StateMachine machine;
    Config config;
    config.enabled = true;
    config.acquisition_streak_epochs = 2;
    machine.update(config, candidate(1.0));
    machine.update(config, candidate(1.2));

    auto hold = candidate(1.4, 0.02);
    hold.acquisition_eligible = false;
    const auto held = machine.update(config, hold);
    EXPECT_EQ(held.state, State::HELD);
    EXPECT_TRUE(held.declared_fixed);
    EXPECT_TRUE(held.held);

    hold.time_s = 1.6;
    hold.nis_per_observation = 100.0;
    const auto revoked = machine.update(config, hold);
    EXPECT_EQ(revoked.state, State::REVOKED);
    EXPECT_TRUE(revoked.revoked);
    EXPECT_FALSE(revoked.declared_fixed);
}

TEST(SafeFixStateMachineTest, NeverHoldsWithoutCandidateCorrection) {
    StateMachine machine;
    Config config;
    config.enabled = true;
    config.acquisition_streak_epochs = 2;
    machine.update(config, candidate(1.0));
    machine.update(config, candidate(1.2));
    Candidate missing;
    missing.time_s = 1.4;
    missing.pair_count = 16;
    missing.nis_per_observation = 1.0;
    EXPECT_TRUE(machine.update(config, missing).revoked);
}

TEST(SafeFixStateMachineTest, NeverHoldsAcrossAnEpochGap) {
    StateMachine machine;
    Config config;
    config.enabled = true;
    config.acquisition_streak_epochs = 2;
    machine.update(config, candidate(1.0));
    machine.update(config, candidate(1.2));
    auto late = candidate(2.0);
    late.acquisition_eligible = false;
    EXPECT_TRUE(machine.update(config, late).revoked);
}

TEST(SafeFixStateMachineTest, InsufficientPairGeometryRevokesDespiteLowNis) {
    StateMachine machine;
    Config config;
    config.enabled = true;
    config.acquisition_streak_epochs = 2;
    machine.update(config, candidate(1.0));
    machine.update(config, candidate(1.2));
    auto nlos = candidate(1.4);
    nlos.acquisition_eligible = false;
    nlos.nis_per_observation = 0.2;
    nlos.prefit_residual_rms_m = 22.0;
    nlos.pair_count = 6;
    EXPECT_TRUE(machine.update(config, nlos).revoked);
}

TEST(SafeFixStateMachineTest, ExtremePrefitResidualFailsClosed) {
    StateMachine machine;
    Config config;
    config.enabled = true;
    config.acquisition_streak_epochs = 2;
    machine.update(config, candidate(1.0));
    machine.update(config, candidate(1.2));
    auto corrupt = candidate(1.4);
    corrupt.acquisition_eligible = false;
    corrupt.nis_per_observation = 0.2;
    corrupt.prefit_residual_rms_m = 51.0;
    EXPECT_TRUE(machine.update(config, corrupt).revoked);
}

TEST(SafeFixStateMachineTest, AbsoluteRatioFloorFailsClosed) {
    StateMachine machine;
    Config config;
    config.enabled = true;
    config.acquisition_streak_epochs = 1;
    config.minimum_absolute_ratio = 1.5;
    auto weak = candidate(1.0);
    weak.ambiguity_ratio = 1.49;
    EXPECT_FALSE(machine.update(config, weak).declared_fixed);
    weak.time_s = 1.2;
    weak.ambiguity_ratio = 1.5;
    EXPECT_TRUE(machine.update(config, weak).declared_fixed);
}

TEST(SafeFixStateMachineTest, IndependentConsensusGateFailsClosed) {
    StateMachine machine;
    Config config;
    config.enabled = true;
    config.acquisition_streak_epochs = 1;
    config.maximum_independent_consensus_delta_m = 0.10;
    auto disagreeing = candidate(1.0);
    disagreeing.independent_consensus_delta_m =
        std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(machine.update(config, disagreeing).declared_fixed);
    disagreeing.time_s = 1.2;
    disagreeing.independent_consensus_delta_m = 0.11;
    EXPECT_FALSE(machine.update(config, disagreeing).declared_fixed);
    disagreeing.time_s = 1.4;
    disagreeing.independent_consensus_delta_m = 0.10;
    EXPECT_TRUE(machine.update(config, disagreeing).declared_fixed);
}

TEST(SafeFixStateMachineTest, StrongInstantAcquisitionIsExplicitAndGated) {
    StateMachine machine;
    Config config;
    config.enabled = true;
    config.acquisition_streak_epochs = 3;
    config.minimum_absolute_ratio = 1.4;
    config.maximum_independent_consensus_delta_m = 0.10;
    config.allow_strong_instant_acquisition = true;
    auto strong = candidate(1.0);
    strong.ambiguity_ratio = 10.0;
    strong.independent_consensus_delta_m = 0.01;
    strong.strong_acquisition_eligible = true;
    const auto accepted = machine.update(config, strong);
    EXPECT_TRUE(accepted.declared_fixed);
    EXPECT_TRUE(accepted.strong_acquisition);

    machine.reset();
    strong.time_s = 1.2;
    strong.ambiguity_ratio = 1.39;
    const auto rejected = machine.update(config, strong);
    EXPECT_FALSE(rejected.declared_fixed);
    EXPECT_FALSE(rejected.strong_acquisition);
}

TEST(SafeFixStateMachineTest, ChangePointRequiresLargeJumpThenStableStreak) {
    StateMachine machine;
    Config config;
    config.enabled = true;
    config.acquisition_streak_epochs = 10;
    config.allow_change_point_acquisition = true;
    auto sample = candidate(1.0, 0.0);
    sample.acquisition_eligible = false;
    EXPECT_FALSE(machine.update(config, sample).declared_fixed);

    sample.time_s = 1.2;
    sample.correction_m.x() = 0.5;
    EXPECT_FALSE(machine.update(config, sample).declared_fixed);
    sample.time_s = 1.4;
    sample.correction_m.x() = 0.505;
    sample.change_point_acquisition_eligible = true;
    EXPECT_FALSE(machine.update(config, sample).declared_fixed);
    sample.time_s = 1.6;
    sample.correction_m.x() = 0.510;
    EXPECT_FALSE(machine.update(config, sample).declared_fixed);
    sample.time_s = 1.8;
    sample.correction_m.x() = 0.515;
    const auto accepted = machine.update(config, sample);
    EXPECT_TRUE(accepted.declared_fixed);
    EXPECT_TRUE(accepted.change_point_acquisition);
}

TEST(SafeFixStateMachineTest, ChangePointFailsClosedAcrossGapAndNonfiniteSample) {
    StateMachine machine;
    Config config;
    config.enabled = true;
    config.acquisition_streak_epochs = 10;
    config.allow_change_point_acquisition = true;
    auto sample = candidate(1.0, 0.0);
    sample.acquisition_eligible = false;
    EXPECT_FALSE(machine.update(config, sample).declared_fixed);

    sample.time_s = 1.2;
    sample.correction_m.x() = 0.5;
    EXPECT_FALSE(machine.update(config, sample).declared_fixed);
    sample.change_point_acquisition_eligible = true;
    sample.time_s = 1.4;
    sample.correction_m.x() = 0.505;
    EXPECT_FALSE(machine.update(config, sample).declared_fixed);

    sample.time_s = 2.0;
    sample.correction_m.x() = 0.510;
    EXPECT_FALSE(machine.update(config, sample).declared_fixed);
    sample.time_s = 2.2;
    sample.correction_m.x() =
        std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(machine.update(config, sample).declared_fixed);

    sample.time_s = 2.4;
    sample.correction_m.x() = 0.515;
    EXPECT_FALSE(machine.update(config, sample).declared_fixed);
    sample.time_s = 2.6;
    sample.correction_m.x() = 0.520;
    EXPECT_FALSE(machine.update(config, sample).declared_fixed);
    sample.time_s = 2.8;
    sample.correction_m.x() = 0.525;
    EXPECT_FALSE(machine.update(config, sample).declared_fixed);
}

TEST(SafeFixStateMachineTest, FixedStateRevokesOnCorrectionJumpOrEpochGap) {
    StateMachine machine;
    Config config;
    config.enabled = true;
    config.acquisition_streak_epochs = 1;
    config.maximum_acquisition_correction_jump_m = 0.02;
    config.maximum_hold_epochs = 0;
    auto sample = candidate(1.0, 0.0);
    EXPECT_TRUE(machine.update(config, sample).declared_fixed);

    sample.time_s = 1.2;
    sample.correction_m.x() = 0.03;
    const auto jumped = machine.update(config, sample);
    EXPECT_FALSE(jumped.declared_fixed);
    EXPECT_TRUE(jumped.revoked);

    machine.reset();
    sample = candidate(1.0, 0.0);
    EXPECT_TRUE(machine.update(config, sample).declared_fixed);
    sample.time_s = 1.4;
    const auto gapped = machine.update(config, sample);
    EXPECT_FALSE(gapped.declared_fixed);
    EXPECT_TRUE(gapped.revoked);
}

TEST(SafeFloatContinuityTest, DisabledFailsClosed) {
    libgnss::safe_float_continuity::Config config;
    EXPECT_FALSE(
        libgnss::safe_float_continuity::propagate(
            config,
            Eigen::Vector3d(1.0, 2.0, 3.0),
            1.0,
            Eigen::Vector3d(4.0, 0.0, 0.0),
            0.2)
            .valid);
}

TEST(SafeFloatContinuityTest, PropagatesTrustedAnchorWithBoundedVelocity) {
    libgnss::safe_float_continuity::Config config;
    config.enabled = true;
    const auto result =
        libgnss::safe_float_continuity::propagate(
            config,
            Eigen::Vector3d(100.0, 200.0, 300.0),
            2.0,
            Eigen::Vector3d(5.0, -1.0, 0.5),
            1.0);
    ASSERT_TRUE(result.valid);
    EXPECT_TRUE(
        result.position_ecef.isApprox(
            Eigen::Vector3d(110.0, 198.0, 301.0)));
    EXPECT_DOUBLE_EQ(result.position_variance_m2, 61.0);
}

TEST(SafeFloatContinuityTest, StaleOrImplausibleInputsFailClosed) {
    libgnss::safe_float_continuity::Config config;
    config.enabled = true;
    EXPECT_FALSE(
        libgnss::safe_float_continuity::propagate(
            config,
            Eigen::Vector3d::Zero(),
            6.1,
            Eigen::Vector3d::Zero(),
            0.0)
            .valid);
    EXPECT_FALSE(
        libgnss::safe_float_continuity::propagate(
            config,
            Eigen::Vector3d::Zero(),
            1.0,
            Eigen::Vector3d(81.0, 0.0, 0.0),
            0.0)
            .valid);
    EXPECT_FALSE(
        libgnss::safe_float_continuity::propagate(
            config,
            Eigen::Vector3d::Zero(),
            1.0,
            Eigen::Vector3d(
                std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0),
            0.0)
            .valid);
}

}  // namespace
