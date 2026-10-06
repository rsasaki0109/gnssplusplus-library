#include <gtest/gtest.h>
#include <libgnss++/algorithms/fix_recovery_guard.hpp>

using namespace libgnss;
namespace {
PositionSolution fix() {
    PositionSolution solution;
    solution.status = SolutionStatus::FIXED;
    solution.num_satellites = 12;
    solution.position_ecef = Vector3d(6378137, 0, 0);
    solution.ratio = 10;
    solution.rtk_update_observations = 20;
    solution.rtk_update_prefit_residual_rms_m = 1;
    solution.rtk_update_post_suppression_residual_rms_m = 0.1;
    solution.rtk_update_normalized_innovation_squared_per_observation = 1;
    return solution;
}
GNSSTime t(double tow) { return GNSSTime(2200, tow); }
FixRecoveryGuard enabled() {
    FixRecoveryGuard::Config config;
    config.enabled = true;
    config.clean_epochs = 3;
    return FixRecoveryGuard(config);
}
TEST(FixRecoveryGuardTest, DisabledPassesThroughEvenSuspiciousOrRepeatedEpochs) {
    FixRecoveryGuard guard;
    auto solution = fix();
    solution.rtk_update_post_suppression_residual_rms_m = 100;
    for (int i = 0; i < 10; ++i) {
        const auto decision = guard.update(solution, t(10));
        EXPECT_FALSE(decision.demote_fixed);
        EXPECT_FALSE(decision.request_primary_reset);
        EXPECT_EQ(decision.state, FixRecoveryGuard::State::NORMAL);
        EXPECT_EQ(solution.status, SolutionStatus::FIXED);
    }
}
TEST(FixRecoveryGuardTest, HardResidualQuarantinesImmediatelyButOnlyRequestsOneReset) {
    auto guard = enabled();
    auto solution = fix();
    solution.rtk_update_post_suppression_residual_rms_m = 5;
    auto first = guard.update(solution, t(10));
    EXPECT_TRUE(first.demote_fixed);
    EXPECT_TRUE(first.request_primary_reset);
    EXPECT_EQ(first.state, FixRecoveryGuard::State::QUARANTINE);
    const auto next = guard.update(solution, t(10.2));
    EXPECT_TRUE(next.demote_fixed);
    EXPECT_FALSE(next.request_primary_reset);
}
TEST(FixRecoveryGuardTest, JointPrefitEvidenceRequiresAContiguousStreak) {
    auto guard = enabled();
    auto suspicious = fix();
    suspicious.rtk_update_prefit_residual_rms_m = 11;
    suspicious.rtk_update_suppressed_outliers = 10;
    suspicious.ratio = 5;
    auto first = guard.update(suspicious, t(10));
    EXPECT_EQ(first.state, FixRecoveryGuard::State::SUSPECT);
    EXPECT_TRUE(first.demote_fixed);
    EXPECT_FALSE(first.request_primary_reset);
    EXPECT_EQ(guard.update(fix(), t(10.2)).state, FixRecoveryGuard::State::NORMAL);
    guard.update(suspicious, t(10.4));
    EXPECT_TRUE(guard.update(suspicious, t(10.6)).request_primary_reset);
}
TEST(FixRecoveryGuardTest, LargePrefitAloneCannotTriggerTheJointRule) {
    auto guard = enabled();
    auto solution = fix();
    solution.rtk_update_prefit_residual_rms_m = 100;
    for (int i = 0; i < 10; ++i)
        EXPECT_FALSE(guard.update(solution, t(10 + i * 0.2)).demote_fixed);
}
TEST(FixRecoveryGuardTest, RecoveryRequiresConsecutiveCleanCandidatesAndEmitsWithoutHolding) {
    auto guard = enabled();
    auto bad = fix();
    bad.rtk_update_post_suppression_residual_rms_m = 10;
    guard.update(bad, t(10));
    auto clean = guard.update(fix(), t(10.2));
    EXPECT_TRUE(clean.demote_fixed);
    EXPECT_EQ(clean.clean_streak, 1);
    EXPECT_EQ(clean.state, FixRecoveryGuard::State::RECOVERY);
    auto missing = PositionSolution{};
    EXPECT_EQ(guard.update(missing, t(10.4)).clean_streak, 0);
    EXPECT_TRUE(guard.update(fix(), t(10.6)).demote_fixed);
    EXPECT_TRUE(guard.update(fix(), t(10.8)).demote_fixed);
    const auto recovered = guard.update(fix(), t(11));
    EXPECT_TRUE(recovered.recovered);
    EXPECT_FALSE(recovered.demote_fixed);
    EXPECT_FALSE(recovered.request_primary_reset);
}
TEST(FixRecoveryGuardTest, AGapOrMissingDiagnosticsCannotCompleteRecovery) {
    auto guard = enabled();
    auto bad = fix();
    bad.rtk_update_normalized_innovation_squared_per_observation = 51;
    guard.update(bad, t(10));
    guard.update(fix(), t(10.2));
    const auto gap = guard.update(fix(), t(12));
    EXPECT_EQ(gap.clean_streak, 1);
    EXPECT_TRUE(gap.reasons & FixRecoveryGuard::INPUT_GAP);
    auto absent = fix();
    absent.rtk_update_observations = 0;
    EXPECT_EQ(guard.update(absent, t(12.2)).clean_streak, 0);
}
TEST(FixRecoveryGuardTest, InvalidConfigurationAndNonMonotoneInputAreRejected) {
    FixRecoveryGuard::Config config;
    config.clean_epochs = 0;
    EXPECT_THROW(FixRecoveryGuard{config}, std::invalid_argument);
    auto guard = enabled();
    guard.update(fix(), t(10));
    EXPECT_THROW(guard.update(fix(), t(10)), std::invalid_argument);
    EXPECT_THROW(guard.update(fix(), t(9)), std::invalid_argument);
}
}
