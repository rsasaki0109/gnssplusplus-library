#include <gtest/gtest.h>

#include <libgnss++/algorithms/inertial_fix_evidence.hpp>

namespace {

using libgnss::inertial_fix_evidence::Config;
using libgnss::inertial_fix_evidence::Evidence;
using libgnss::inertial_fix_evidence::evaluate;

Evidence nominalEvidence() {
    Evidence evidence;
    evidence.available = true;
    evidence.healthy_independent_anchor = true;
    evidence.time_error_s = 0.05;
    evidence.predicted_position_covariance_ecef =
        Eigen::Matrix3d::Identity() * 0.01;
    evidence.primary_candidate_ecef =
        Eigen::Vector3d(0.8, 0.0, 0.0);
    return evidence;
}

TEST(InertialFixEvidence, FailsClosedWithoutIndependentAnchor) {
    auto evidence = nominalEvidence();
    evidence.healthy_independent_anchor = false;
    EXPECT_FALSE(evaluate(Config{}, evidence).passed);
}

TEST(InertialFixEvidence, AcceptsAtCausalAndStatisticalBoundaries) {
    const auto decision = evaluate(Config{}, nominalEvidence());
    EXPECT_TRUE(decision.passed);
    EXPECT_NEAR(decision.position_delta_m, 0.8, 1e-12);
    EXPECT_LT(decision.nis_per_dimension, 5.422);
}

TEST(InertialFixEvidence, RejectsStaleGrossOrSingularEvidence) {
    auto stale = nominalEvidence();
    stale.time_error_s = 0.051;
    EXPECT_FALSE(evaluate(Config{}, stale).passed);

    auto gross = nominalEvidence();
    gross.primary_candidate_ecef = Eigen::Vector3d(2.01, 0.0, 0.0);
    EXPECT_FALSE(evaluate(Config{}, gross).passed);

    auto invalid = nominalEvidence();
    invalid.predicted_position_covariance_ecef(0, 0) =
        std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(evaluate(Config{}, invalid).passed);
}

}  // namespace
