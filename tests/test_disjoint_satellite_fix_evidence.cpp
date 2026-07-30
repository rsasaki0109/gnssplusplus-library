#include <gtest/gtest.h>

#include <libgnss++/algorithms/disjoint_satellite_fix_evidence.hpp>

namespace {

namespace evidence =
    libgnss::disjoint_satellite_fix_evidence;

TEST(DisjointSatelliteFixEvidence, FailsClosedWithoutDisjointInputs) {
    evidence::Evidence input;
    input.available = true;
    input.primary_ffrt_passed = true;
    input.partition_a_ffrt_passed = true;
    input.partition_b_ffrt_passed = true;
    input.partition_a_candidate_ecef = Eigen::Vector3d::Zero();
    input.partition_b_candidate_ecef = Eigen::Vector3d::Zero();
    input.primary_candidate_ecef = Eigen::Vector3d::Zero();

    EXPECT_FALSE(evidence::evaluate({}, input).passed);
}

TEST(DisjointSatelliteFixEvidence, AcceptsAtSeparationBoundaries) {
    evidence::Evidence input;
    input.available = true;
    input.inputs_verified_disjoint = true;
    input.primary_ffrt_passed = true;
    input.partition_a_ffrt_passed = true;
    input.partition_b_ffrt_passed = true;
    input.partition_a_candidate_ecef =
        Eigen::Vector3d(-0.125, 0.0, 0.0);
    input.partition_b_candidate_ecef =
        Eigen::Vector3d(0.125, 0.0, 0.0);
    input.primary_candidate_ecef = Eigen::Vector3d::Zero();

    const auto decision = evidence::evaluate({}, input);
    EXPECT_TRUE(decision.passed);
    EXPECT_DOUBLE_EQ(decision.partition_separation_m, 0.25);
}

TEST(DisjointSatelliteFixEvidence, RejectsFfrtOrSeparationFailure) {
    evidence::Evidence input;
    input.available = true;
    input.inputs_verified_disjoint = true;
    input.primary_ffrt_passed = true;
    input.partition_a_ffrt_passed = true;
    input.partition_b_ffrt_passed = false;
    input.partition_a_candidate_ecef = Eigen::Vector3d::Zero();
    input.partition_b_candidate_ecef =
        Eigen::Vector3d(0.01, 0.0, 0.0);
    input.primary_candidate_ecef = Eigen::Vector3d::Zero();
    EXPECT_FALSE(evidence::evaluate({}, input).passed);

    input.partition_b_ffrt_passed = true;
    input.partition_b_candidate_ecef =
        Eigen::Vector3d(0.26, 0.0, 0.0);
    EXPECT_FALSE(evidence::evaluate({}, input).passed);
}

TEST(DisjointSatelliteFixEvidence, AcceptsCovarianceNormalizedSeparation) {
    evidence::Evidence input;
    input.available = true;
    input.inputs_verified_disjoint = true;
    input.primary_ffrt_passed = true;
    input.partition_a_ffrt_passed = true;
    input.partition_b_ffrt_passed = true;
    input.partition_a_candidate_ecef =
        Eigen::Vector3d(-0.4, 0.0, 0.0);
    input.partition_b_candidate_ecef =
        Eigen::Vector3d(0.4, 0.0, 0.0);
    input.primary_candidate_ecef = Eigen::Vector3d::Zero();
    input.partition_a_covariance_ecef =
        Eigen::Matrix3d::Identity() * 0.04;
    input.partition_b_covariance_ecef =
        Eigen::Matrix3d::Identity() * 0.04;
    input.primary_covariance_ecef =
        Eigen::Matrix3d::Identity() * 0.04;

    const auto decision = evidence::evaluate({}, input);
    EXPECT_FALSE(decision.hard_separation_passed);
    EXPECT_TRUE(decision.statistical_separation_passed);
    EXPECT_TRUE(decision.passed);

    input.partition_b_candidate_ecef =
        Eigen::Vector3d(2.1, 0.0, 0.0);
    EXPECT_FALSE(evidence::evaluate({}, input).passed);
}

TEST(DisjointSatelliteFixEvidence, ConsensusRejectsPrimaryOutlier) {
    const auto result = evidence::closestPairConsensus(
        Eigen::Vector3d(1.5, 0.0, 0.0),
        Eigen::Vector3d(0.01, 0.0, 0.0),
        Eigen::Vector3d(-0.01, 0.0, 0.0));
    ASSERT_TRUE(result.valid);
    EXPECT_EQ(
        result.selected_pair,
        evidence::SelectedPair::A_B);
    EXPECT_NEAR(result.position_ecef.x(), 0.0, 1e-12);
    EXPECT_NEAR(result.selected_pair_separation_m, 0.02, 1e-12);
}

TEST(DisjointSatelliteFixEvidence, ConsensusRejectsPartitionOutlier) {
    const auto result = evidence::closestPairConsensus(
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(0.04, 0.0, 0.0),
        Eigen::Vector3d(1.34, 0.0, 0.0));
    ASSERT_TRUE(result.valid);
    EXPECT_EQ(
        result.selected_pair,
        evidence::SelectedPair::PRIMARY_A);
    EXPECT_NEAR(result.position_ecef.x(), 0.02, 1e-12);
    EXPECT_NEAR(result.selected_pair_separation_m, 0.04, 1e-12);
}

}  // namespace
