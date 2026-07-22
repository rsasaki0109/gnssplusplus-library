#include <gtest/gtest.h>

#include "libgnss++/algorithms/integrity_consensus.hpp"

namespace {

using Manager = libgnss::IntegrityConsensusManager;
using PositionConsensus = libgnss::MultiShadowPositionConsensus;

Manager::Input input(double separation_m = 0.0) {
    Manager::Input value;
    value.primary.valid = true;
    value.primary.position_ecef = Eigen::Vector3d::Zero();
    value.primary.covariance_trace_m2 = 0.25;
    value.independent.valid = true;
    value.independent.position_ecef = Eigen::Vector3d(separation_m, 0.0, 0.0);
    value.independent.covariance_trace_m2 = 0.25;
    value.independent_age_s = 0.2;
    value.fixed_candidate = true;
    return value;
}

TEST(IntegrityConsensusTest, NormalAgreementAllowsFixed) {
    Manager manager;
    const auto decision = manager.update(input(0.5));
    EXPECT_EQ(decision.state, Manager::State::NORMAL);
    EXPECT_TRUE(decision.estimators_agree);
    EXPECT_TRUE(decision.allow_fixed);
}

TEST(IntegrityConsensusTest, HardSuspectEntersFailClosedQuarantine) {
    Manager manager;
    auto value = input(0.2);
    value.hard_primary_suspect = true;
    const auto decision = manager.update(value);
    EXPECT_EQ(decision.state, Manager::State::QUARANTINE);
    EXPECT_FALSE(decision.allow_fixed);
    EXPECT_TRUE(decision.request_primary_reset);
}

TEST(IntegrityConsensusTest, MissingIndependentSoftSuspectStaysNormal) {
    Manager manager;
    auto value = input(0.2);
    value.independent.valid = false;
    value.primary_suspect = true;
    for (int i = 0; i < 5; ++i) {
        const auto decision = manager.update(value);
        EXPECT_EQ(decision.state, Manager::State::NORMAL);
        EXPECT_TRUE(decision.allow_fixed);
    }
}

TEST(IntegrityConsensusTest, JointEvidenceModeIgnoresCleanDisagreement) {
    Manager::Config config;
    config.disagreement_requires_primary_suspect = true;
    Manager manager(config);
    for (int i = 0; i < 5; ++i) {
        const auto decision = manager.update(input(100.0));
        EXPECT_EQ(decision.state, Manager::State::NORMAL);
        EXPECT_TRUE(decision.allow_fixed);
    }
}

TEST(IntegrityConsensusTest, RecoveryRequiresConsecutiveAgreement) {
    Manager::Config config;
    config.recovery_streak = 3;
    Manager manager(config);
    auto value = input(0.2);
    value.hard_primary_suspect = true;
    manager.update(value);
    value.hard_primary_suspect = false;
    EXPECT_EQ(manager.update(value).state, Manager::State::RECOVERY);
    EXPECT_EQ(manager.update(value).state, Manager::State::RECOVERY);
    const auto promoted = manager.update(value);
    EXPECT_EQ(promoted.state, Manager::State::NORMAL);
    EXPECT_TRUE(promoted.promote_joint_anchor);
    EXPECT_TRUE(promoted.allow_fixed);
}

TEST(IntegrityConsensusTest, HealthyRecoveryCandidateAllowsProvisionalFixedOnly) {
    Manager::Config config;
    config.recovery_streak = 3;
    Manager manager(config);
    auto value = input(0.2);
    value.hard_primary_suspect = true;
    manager.update(value);
    value.hard_primary_suspect = false;
    value.recovery_candidate_healthy = true;
    const auto provisional = manager.update(value);
    EXPECT_EQ(provisional.state, Manager::State::RECOVERY);
    EXPECT_TRUE(provisional.allow_fixed);
    EXPECT_FALSE(provisional.promote_joint_anchor);
    EXPECT_EQ(provisional.recovery_streak, 1);
}

TEST(IntegrityConsensusTest, MissingIndependentCannotAuthorizeRecovery) {
    Manager manager;
    auto value = input(0.2);
    value.hard_primary_suspect = true;
    manager.update(value);
    value.hard_primary_suspect = false;
    value.independent.valid = false;
    const auto decision = manager.update(value);
    EXPECT_EQ(decision.state, Manager::State::QUARANTINE);
    EXPECT_FALSE(decision.allow_fixed);
    EXPECT_NE(decision.reasons & Manager::INDEPENDENT_UNAVAILABLE, 0U);
}

TEST(IntegrityConsensusTest, ResetGenerationRestartsRecovery) {
    Manager::Config config;
    config.recovery_streak = 3;
    Manager manager(config);
    auto value = input(0.2);
    value.hard_primary_suspect = true;
    manager.update(value);
    value.hard_primary_suspect = false;
    EXPECT_EQ(manager.update(value).state, Manager::State::RECOVERY);
    value.independent_reset_generation = 1;
    const auto decision = manager.update(value);
    EXPECT_EQ(decision.state, Manager::State::QUARANTINE);
    EXPECT_FALSE(decision.promote_joint_anchor);
}

PositionConsensus::Shadow shadow(
    std::uint64_t source_id, double x, std::uint64_t age_epochs = 0) {
    PositionConsensus::Shadow value;
    value.source_id = source_id;
    value.valid = true;
    value.fixed = true;
    value.healthy = true;
    value.position_ecef = Eigen::Vector3d(x, 0.0, 0.0);
    value.age_epochs = age_epochs;
    return value;
}

PositionConsensus::Input position_input() {
    PositionConsensus::Input value;
    value.primary_valid = true;
    value.primary_fixed = true;
    value.primary_position_ecef = Eigen::Vector3d::Zero();
    value.prediction_valid = true;
    value.predicted_position_ecef = Eigen::Vector3d(1.05, 0.0, 0.0);
    value.shadows = {shadow(1, 1.0), shadow(2, 1.1)};
    return value;
}

TEST(MultiShadowPositionConsensusTest, TwoIndependentFixedShadowsAuthorizeMedian) {
    PositionConsensus consensus;
    const auto decision = consensus.evaluate(position_input());
    EXPECT_TRUE(decision.replace_primary_position);
    EXPECT_NEAR(decision.consensus_position_ecef.x(), 1.05, 1e-12);
    EXPECT_NEAR(decision.cluster_diameter_m, 0.1, 1e-12);
    EXPECT_EQ(decision.source_ids, (std::vector<std::uint64_t>{1, 2}));
}

TEST(MultiShadowPositionConsensusTest, DuplicateSourceCannotCreateAuthority) {
    PositionConsensus consensus;
    auto value = position_input();
    value.shadows[1].source_id = value.shadows[0].source_id;
    const auto decision = consensus.evaluate(value);
    EXPECT_FALSE(decision.replace_primary_position);
    EXPECT_NE(decision.reasons & PositionConsensus::DUPLICATE_SHADOW_SOURCE, 0U);
}

TEST(MultiShadowPositionConsensusTest, EqualSizeCompetingClustersAreAmbiguous) {
    PositionConsensus consensus;
    auto value = position_input();
    value.shadows = {
        shadow(1, 1.0), shadow(2, 1.1), shadow(3, 4.0), shadow(4, 4.1)};
    const auto decision = consensus.evaluate(value);
    EXPECT_FALSE(decision.replace_primary_position);
    EXPECT_NE(decision.reasons & PositionConsensus::AMBIGUOUS_CLUSTERS, 0U);
}

TEST(MultiShadowPositionConsensusTest, ClusterNeedsFreshMember) {
    PositionConsensus consensus;
    auto value = position_input();
    value.shadows = {shadow(1, 1.0, 1000), shadow(2, 1.1, 1200)};
    const auto decision = consensus.evaluate(value);
    EXPECT_FALSE(decision.replace_primary_position);
    EXPECT_NE(decision.reasons & PositionConsensus::NO_UNIQUE_CLUSTER, 0U);
}

TEST(MultiShadowPositionConsensusTest, PredictionGateRejectsCorrelatedWrongBasin) {
    PositionConsensus consensus;
    auto value = position_input();
    value.predicted_position_ecef = Eigen::Vector3d(4.0, 0.0, 0.0);
    const auto decision = consensus.evaluate(value);
    EXPECT_FALSE(decision.replace_primary_position);
    EXPECT_NE(decision.reasons & PositionConsensus::PREDICTION_DISAGREEMENT, 0U);
}

TEST(MultiShadowPositionConsensusTest, PredictionMustBeAvailableWhenConfigured) {
    PositionConsensus consensus;
    auto value = position_input();
    value.prediction_valid = false;
    const auto decision = consensus.evaluate(value);
    EXPECT_FALSE(decision.replace_primary_position);
    EXPECT_NE(decision.reasons & PositionConsensus::PREDICTION_UNAVAILABLE, 0U);
}

}  // namespace
