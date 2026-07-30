#include <gtest/gtest.h>

#include <libgnss++/algorithms/fixed_quality_gate.hpp>

namespace {

using libgnss::fixed_quality_gate::Config;
using libgnss::fixed_quality_gate::Evidence;
using libgnss::fixed_quality_gate::evaluate;

TEST(FixedQualityGate, FailsClosedWithoutEvidence) {
    const auto decision = evaluate(Config{}, Evidence{});
    EXPECT_FALSE(decision.passed);
    EXPECT_FALSE(decision.safe_shadow_branch);
    EXPECT_FALSE(decision.covariance_branch);
    EXPECT_FALSE(decision.strong_innovation_branch);
}

TEST(FixedQualityGate, AcceptsSafeShadowBranch) {
    Evidence evidence;
    evidence.safe_fix_shadow_declared_fixed = true;
    const auto decision = evaluate(Config{}, evidence);
    EXPECT_TRUE(decision.passed);
    EXPECT_TRUE(decision.safe_shadow_branch);
}

TEST(FixedQualityGate, AcceptsCovarianceBranchAtBoundary) {
    Evidence evidence;
    evidence.float_position_covariance_trace_m2 = 0.00025;
    evidence.update_nis_per_observation = 10.0;
    const auto decision = evaluate(Config{}, evidence);
    EXPECT_TRUE(decision.passed);
    EXPECT_TRUE(decision.covariance_branch);
}

TEST(FixedQualityGate, AcceptsStrongInnovationBranchAtBoundary) {
    Evidence evidence;
    evidence.update_observations = 28;
    evidence.update_nis_per_observation = 1.0;
    const auto decision = evaluate(Config{}, evidence);
    EXPECT_TRUE(decision.passed);
    EXPECT_TRUE(decision.strong_innovation_branch);
}

TEST(FixedQualityGate, RejectsWeakEvidenceOutsideBoundaries) {
    Evidence evidence;
    evidence.float_position_covariance_trace_m2 = 0.000251;
    evidence.update_observations = 27;
    evidence.update_nis_per_observation = 1.001;
    EXPECT_FALSE(evaluate(Config{}, evidence).passed);
}

TEST(FixedQualityGate, StrongInnovationRejectsMajorityOutlierSuppression) {
    Evidence evidence;
    evidence.update_observations = 60;
    evidence.suppressed_outliers = 31;
    evidence.update_nis_per_observation = 0.1;
    EXPECT_FALSE(evaluate(Config{}, evidence).strong_innovation_branch);

    evidence.suppressed_outliers = 30;
    EXPECT_TRUE(evaluate(Config{}, evidence).strong_innovation_branch);
}

TEST(FixedQualityGate, IndependentBudgetIsFailClosedWhenRequired) {
    Config config;
    config.require_independent_failure_budget = true;
    Evidence evidence;
    evidence.safe_fix_shadow_declared_fixed = true;

    const auto rejected = evaluate(config, evidence);
    EXPECT_TRUE(rejected.safe_shadow_branch);
    EXPECT_FALSE(rejected.independent_failure_budget_passed);
    EXPECT_FALSE(rejected.passed);

    evidence.independent_failure_budget_passed = true;
    const auto accepted = evaluate(config, evidence);
    EXPECT_TRUE(accepted.independent_failure_budget_passed);
    EXPECT_TRUE(accepted.passed);
}

}  // namespace
