#include <gtest/gtest.h>

#include <libgnss++/algorithms/fix_failure_budget.hpp>

namespace {

namespace budget = libgnss::fix_failure_budget;

TEST(FixFailureBudget, RejectsTwoAlgorithmsInOneFaultDomain) {
    const std::array<budget::Evidence, 2> evidence{{
        {budget::SourceFamily::PRIMARY_CARRIER_AR, true, 1e-3},
        {budget::SourceFamily::PRIMARY_CARRIER_AR, true, 1e-3},
    }};
    const auto decision = budget::evaluate(budget::Config{}, evidence);
    EXPECT_FALSE(decision.passed);
    EXPECT_EQ(decision.independent_families, 1);
    EXPECT_DOUBLE_EQ(decision.joint_failure_probability, 1e-3);
}

TEST(FixFailureBudget, AcceptsTwoFamiliesAtJointBoundary) {
    const std::array<budget::Evidence, 2> evidence{{
        {budget::SourceFamily::PRIMARY_CARRIER_AR, true, 1e-3},
        {budget::SourceFamily::MULTIFREQUENCY_CASCADE, true, 2e-3},
    }};
    const auto decision = budget::evaluate(budget::Config{}, evidence);
    EXPECT_TRUE(decision.passed);
    EXPECT_EQ(decision.independent_families, 2);
    EXPECT_NEAR(decision.joint_failure_probability, 2e-6, 1e-18);
}

TEST(FixFailureBudget, RejectsMissingInvalidOrOverBudgetEvidence) {
    const std::array<budget::Evidence, 3> evidence{{
        {budget::SourceFamily::PRIMARY_CARRIER_AR, true, 3e-3},
        {budget::SourceFamily::MULTIFREQUENCY_CASCADE, true, 1e-3},
        {budget::SourceFamily::INERTIAL_SOLUTION_SEPARATION, false, 1e-4},
    }};
    const auto decision = budget::evaluate(budget::Config{}, evidence);
    EXPECT_FALSE(decision.passed);
    EXPECT_EQ(decision.independent_families, 2);
    EXPECT_NEAR(decision.joint_failure_probability, 3e-6, 1e-18);
}

}  // namespace
