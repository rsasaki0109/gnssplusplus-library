#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "libgnss++/algorithms/integrity_consensus.hpp"

namespace {

using Manager = libgnss::IntegrityConsensusManager;
using PositionConsensus = libgnss::MultiShadowPositionConsensus;
using RealtimeGate = libgnss::RealtimeFixIntegrityGate;

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

libgnss::PositionSolution fixed_solution(double x = 0.0) {
    libgnss::PositionSolution solution;
    solution.status = libgnss::SolutionStatus::FIXED;
    solution.num_satellites = 20;
    solution.position_ecef = Eigen::Vector3d(x, 0.0, 0.0);
    solution.position_covariance = Eigen::Matrix3d::Identity() * 0.01;
    solution.ratio = 10.0;
    solution.rtk_update_observations = 24;
    solution.rtk_update_suppressed_outliers = 12;
    solution.rtk_update_prefit_residual_rms_m = 41.0;
    return solution;
}

RealtimeGate::IndependentEstimate independent(double x) {
    RealtimeGate::IndependentEstimate estimate;
    estimate.present = true;
    estimate.estimate.valid = true;
    estimate.estimate.position_ecef = Eigen::Vector3d(x, 0.0, 0.0);
    estimate.estimate.covariance_trace_m2 = 0.25;
    estimate.age_s = 0.0;
    return estimate;
}

// Mirrors ShadowEstimateHealthGate::Result for an unhealthy sample: present
// and position-valid, but covariance_trace_m2 == +infinity (e.g. an
// unpopulated/placeholder shadow trace, per the PPC tokyo1 regression this
// guards). Must behave exactly like an absent independent for every
// consensus evidence path.
RealtimeGate::IndependentEstimate unhealthy_independent(double x) {
    RealtimeGate::IndependentEstimate estimate;
    estimate.present = true;
    estimate.estimate.valid = true;
    estimate.estimate.position_ecef = Eigen::Vector3d(x, 0.0, 0.0);
    estimate.estimate.covariance_trace_m2 =
        std::numeric_limits<double>::infinity();
    estimate.age_s = 0.0;
    return estimate;
}

libgnss::PositionSolution suspicious_primary_solution(double x = 0.0) {
    auto solution = fixed_solution(x);
    solution.rtk_update_prefit_residual_rms_m = 50.0;   // > primary_max_prefit_rms_m (10.0)
    solution.rtk_update_suppressed_outliers = 40;        // >= primary_min_suppressed_outliers (35)
    return solution;
}

TEST(RealtimeFixIntegrityGateTest, ConfirmedResidualStreakDemotesBufferedPrefix) {
    RealtimeGate::Config config;
    config.enable_consensus = false;
    RealtimeGate gate(config);

    for (int epoch = 0; epoch < 7; ++epoch) {
        RealtimeGate::EpochInput input;
        input.primary = fixed_solution(static_cast<double>(epoch));
        EXPECT_TRUE(gate.push(std::move(input)).emitted.empty());
    }
    RealtimeGate::EpochInput eighth;
    eighth.primary = fixed_solution(7.0);
    auto update = gate.push(std::move(eighth));
    ASSERT_EQ(update.emitted.size(), 1U);
    EXPECT_EQ(update.emitted.front().solution.status, libgnss::SolutionStatus::FLOAT);
    EXPECT_TRUE(update.emitted.front().telemetry.residual_streak_demoted);
    EXPECT_EQ(update.emitted.front().telemetry.output_latency_epochs, 7);

    const auto tail = gate.flush();
    ASSERT_EQ(tail.size(), 7U);
    for (const auto& emission : tail) {
        EXPECT_EQ(emission.solution.status, libgnss::SolutionStatus::FLOAT);
        EXPECT_TRUE(emission.telemetry.residual_streak_demoted);
    }
}

TEST(RealtimeFixIntegrityGateTest, UnconfirmedResidualPrefixFailsOpenOnFlush) {
    RealtimeGate::Config config;
    config.enable_consensus = false;
    RealtimeGate gate(config);
    for (int epoch = 0; epoch < 7; ++epoch) {
        RealtimeGate::EpochInput input;
        input.primary = fixed_solution(static_cast<double>(epoch));
        gate.push(std::move(input));
    }
    const auto output = gate.flush();
    ASSERT_EQ(output.size(), 7U);
    for (const auto& emission : output) {
        EXPECT_EQ(emission.solution.status, libgnss::SolutionStatus::FIXED);
        EXPECT_FALSE(emission.telemetry.output_demoted);
    }
}

TEST(RealtimeFixIntegrityGateTest, MissingIndependentDoesNotLatchSoftSuspect) {
    RealtimeGate::Config config;
    config.enable_residual_policy = false;
    RealtimeGate gate(config);
    for (int epoch = 0; epoch < 5; ++epoch) {
        RealtimeGate::EpochInput input;
        input.primary = fixed_solution();
        input.primary.rtk_update_prefit_residual_rms_m = 20.0;
        input.primary.rtk_update_suppressed_outliers = 35;
        const auto update = gate.push(std::move(input));
        ASSERT_EQ(update.emitted.size(), 1U);
        EXPECT_EQ(update.current.consensus.state, Manager::State::NORMAL);
        EXPECT_EQ(update.emitted.front().solution.status,
                  libgnss::SolutionStatus::FIXED);
    }
}

// --- Base confidence gate (ported from scripts/apply_ppc_status_demotion.py
// should_demote()/should_exonerate() with the frozen audited defaults). ---

libgnss::PositionSolution base_confidence_solution(
    int nsat, double ratio, double prefit_rms_m = 1.0,
    double nis_per_obs = 0.0, int outliers = 0) {
    auto solution = fixed_solution();
    solution.num_satellites = nsat;
    solution.ratio = ratio;
    // Keep prefit/outliers away from the residual streak/spike thresholds
    // (prefit > 40, outliers >= 12) so these tests isolate the base gate.
    solution.rtk_update_prefit_residual_rms_m = prefit_rms_m;
    solution.rtk_update_suppressed_outliers = outliers;
    solution.rtk_update_normalized_innovation_squared_per_observation =
        nis_per_obs;
    return solution;
}

TEST(RealtimeFixIntegrityGateTest, BaseConfidenceDisabledByDefaultNeverDemotes) {
    // Fix 1: enable_base_confidence_policy now defaults to false (opt-in via
    // gnss_solve's --integrity-base-gate). Verify the default alone -- no
    // explicit config.enable_base_confidence_policy write -- never demotes,
    // even for a solution that would otherwise clearly fail the base gate.
    RealtimeGate::Config config;
    config.enable_consensus = false;
    config.enable_residual_policy = false;
    RealtimeGate gate(config);
    RealtimeGate::EpochInput input;
    input.primary = base_confidence_solution(/*nsat=*/3, /*ratio=*/1.0);
    auto update = gate.push(std::move(input));
    ASSERT_EQ(update.emitted.size(), 1U);
    EXPECT_EQ(update.emitted.front().solution.status,
              libgnss::SolutionStatus::FIXED);
    EXPECT_FALSE(update.emitted.front().telemetry.base_confidence_demoted);
}

TEST(RealtimeFixIntegrityGateTest, BaseConfidenceDemotesBelowMinSatellites) {
    RealtimeGate::Config config;
    config.enable_consensus = false;
    config.enable_residual_policy = false;
    config.enable_base_confidence_policy = true;
    RealtimeGate gate(config);
    RealtimeGate::EpochInput input;
    input.primary = base_confidence_solution(/*nsat=*/5, /*ratio=*/50.0);
    auto update = gate.push(std::move(input));
    ASSERT_EQ(update.emitted.size(), 1U);
    EXPECT_EQ(update.emitted.front().solution.status,
              libgnss::SolutionStatus::FLOAT);
    EXPECT_TRUE(update.emitted.front().telemetry.base_confidence_demoted);
    EXPECT_TRUE(update.emitted.front().telemetry.output_demoted);
}

TEST(RealtimeFixIntegrityGateTest, BaseConfidenceDemotesLowSatelliteWeakRatio) {
    RealtimeGate::Config config;
    config.enable_consensus = false;
    config.enable_residual_policy = false;
    config.enable_base_confidence_policy = true;
    RealtimeGate gate(config);
    RealtimeGate::EpochInput input;
    // nsat=10 is >= min_satellites(8) but <= low_satellite_ceiling(11), and
    // ratio=5 <= low_satellite_max_ratio(15).
    input.primary = base_confidence_solution(/*nsat=*/10, /*ratio=*/5.0);
    auto update = gate.push(std::move(input));
    ASSERT_EQ(update.emitted.size(), 1U);
    EXPECT_EQ(update.emitted.front().solution.status,
              libgnss::SolutionStatus::FLOAT);
    EXPECT_TRUE(update.emitted.front().telemetry.base_confidence_demoted);
}

TEST(RealtimeFixIntegrityGateTest, BaseConfidenceIgnoresHealthySatelliteCount) {
    RealtimeGate::Config config;
    config.enable_consensus = false;
    config.enable_residual_policy = false;
    config.enable_base_confidence_policy = true;
    RealtimeGate gate(config);
    RealtimeGate::EpochInput input;
    // nsat=20 clears both the hard floor and the low-satellite ceiling.
    input.primary = base_confidence_solution(/*nsat=*/20, /*ratio=*/1.0);
    auto update = gate.push(std::move(input));
    ASSERT_EQ(update.emitted.size(), 1U);
    EXPECT_EQ(update.emitted.front().solution.status,
              libgnss::SolutionStatus::FIXED);
    EXPECT_FALSE(update.emitted.front().telemetry.base_confidence_demoted);
}

TEST(RealtimeFixIntegrityGateTest, BaseConfidenceExplicitlyDisabledNeverDemotes) {
    // Explicit false (same as the default -- see
    // BaseConfidenceDisabledByDefaultNeverDemotes above) is still respected
    // when set alongside an otherwise-enabled config.
    RealtimeGate::Config config;
    config.enable_consensus = false;
    config.enable_residual_policy = false;
    config.enable_base_confidence_policy = false;
    RealtimeGate gate(config);
    RealtimeGate::EpochInput input;
    input.primary = base_confidence_solution(/*nsat=*/3, /*ratio=*/1.0);
    auto update = gate.push(std::move(input));
    ASSERT_EQ(update.emitted.size(), 1U);
    EXPECT_EQ(update.emitted.front().solution.status,
              libgnss::SolutionStatus::FIXED);
    EXPECT_FALSE(update.emitted.front().telemetry.base_confidence_demoted);
}

TEST(RealtimeFixIntegrityGateTest, BaseConfidenceExonerationRequiresSatelliteFloor) {
    RealtimeGate::Config config;
    config.enable_consensus = false;
    config.enable_residual_policy = false;
    config.enable_base_confidence_policy = true;
    RealtimeGate gate(config);
    RealtimeGate::EpochInput input;
    // Gate fires (nsat=10 <= ceiling, ratio <= max), but nsat=10 is below
    // exonerate_min_satellites(11), so exoneration cannot apply even with
    // otherwise-perfect telemetry.
    input.primary = base_confidence_solution(
        /*nsat=*/10, /*ratio=*/10.0, /*prefit_rms_m=*/0.1, /*nis_per_obs=*/0.1);
    auto update = gate.push(std::move(input));
    ASSERT_EQ(update.emitted.size(), 1U);
    EXPECT_EQ(update.emitted.front().solution.status,
              libgnss::SolutionStatus::FLOAT);
    EXPECT_TRUE(update.emitted.front().telemetry.base_confidence_demoted);
}

TEST(RealtimeFixIntegrityGateTest, BaseConfidenceExonerationRequiresTightPrefit) {
    RealtimeGate::Config config;
    config.enable_consensus = false;
    config.enable_residual_policy = false;
    config.enable_base_confidence_policy = true;
    RealtimeGate gate(config);
    RealtimeGate::EpochInput input;
    input.primary = base_confidence_solution(
        /*nsat=*/11, /*ratio=*/10.0, /*prefit_rms_m=*/1.0, /*nis_per_obs=*/0.1);
    auto update = gate.push(std::move(input));
    ASSERT_EQ(update.emitted.size(), 1U);
    EXPECT_EQ(update.emitted.front().solution.status,
              libgnss::SolutionStatus::FLOAT);
    EXPECT_TRUE(update.emitted.front().telemetry.base_confidence_demoted);
}

TEST(RealtimeFixIntegrityGateTest, BaseConfidenceExonerationRequiresTightNis) {
    RealtimeGate::Config config;
    config.enable_consensus = false;
    config.enable_residual_policy = false;
    config.enable_base_confidence_policy = true;
    RealtimeGate gate(config);
    RealtimeGate::EpochInput input;
    input.primary = base_confidence_solution(
        /*nsat=*/11, /*ratio=*/10.0, /*prefit_rms_m=*/0.1, /*nis_per_obs=*/0.5);
    auto update = gate.push(std::move(input));
    ASSERT_EQ(update.emitted.size(), 1U);
    EXPECT_EQ(update.emitted.front().solution.status,
              libgnss::SolutionStatus::FLOAT);
    EXPECT_TRUE(update.emitted.front().telemetry.base_confidence_demoted);
}

TEST(RealtimeFixIntegrityGateTest, BaseConfidenceExonerationOverridesDemotion) {
    RealtimeGate::Config config;
    config.enable_consensus = false;
    config.enable_residual_policy = false;
    config.enable_base_confidence_policy = true;
    RealtimeGate gate(config);
    RealtimeGate::EpochInput input;
    input.primary = base_confidence_solution(
        /*nsat=*/11, /*ratio=*/10.0, /*prefit_rms_m=*/0.1, /*nis_per_obs=*/0.1);
    auto update = gate.push(std::move(input));
    ASSERT_EQ(update.emitted.size(), 1U);
    EXPECT_EQ(update.emitted.front().solution.status,
              libgnss::SolutionStatus::FIXED);
    EXPECT_FALSE(update.emitted.front().telemetry.base_confidence_demoted);
}

TEST(RealtimeFixIntegrityGateTest, BaseConfidenceInteractsWithSevenEpochBuffer) {
    RealtimeGate::Config config;
    config.enable_consensus = false;
    config.enable_base_confidence_policy = true;
    RealtimeGate gate(config);  // enable_residual_policy default true -> 7-epoch buffer.

    RealtimeGate::EpochInput first;
    first.primary = base_confidence_solution(/*nsat=*/5, /*ratio=*/50.0);
    EXPECT_TRUE(gate.push(std::move(first)).emitted.empty());

    for (int epoch = 0; epoch < 6; ++epoch) {
        RealtimeGate::EpochInput input;
        input.primary = fixed_solution(static_cast<double>(epoch));
        input.primary.rtk_update_prefit_residual_rms_m = 1.0;
        input.primary.rtk_update_suppressed_outliers = 0;
        EXPECT_TRUE(gate.push(std::move(input)).emitted.empty());
    }

    RealtimeGate::EpochInput eighth;
    eighth.primary = fixed_solution(6.0);
    eighth.primary.rtk_update_prefit_residual_rms_m = 1.0;
    eighth.primary.rtk_update_suppressed_outliers = 0;
    auto update = gate.push(std::move(eighth));
    ASSERT_EQ(update.emitted.size(), 1U);
    EXPECT_EQ(update.emitted.front().solution.status,
              libgnss::SolutionStatus::FLOAT);
    EXPECT_TRUE(update.emitted.front().telemetry.base_confidence_demoted);
    EXPECT_EQ(update.emitted.front().telemetry.output_latency_epochs, 7);

    const auto tail = gate.flush();
    ASSERT_EQ(tail.size(), 7U);
    for (const auto& emission : tail) {
        EXPECT_EQ(emission.solution.status, libgnss::SolutionStatus::FIXED);
        EXPECT_FALSE(emission.telemetry.base_confidence_demoted);
    }
}

TEST(RealtimeFixIntegrityGateTest, IndependentDisagreementControlsEmittedStatus) {
    RealtimeGate::Config config;
    config.enable_residual_policy = false;
    config.consensus.aperture_min_m = 1.0;
    config.consensus.aperture_max_m = 1.0;
    RealtimeGate gate(config);

    for (int epoch = 0; epoch < 3; ++epoch) {
        RealtimeGate::EpochInput input;
        input.primary = fixed_solution();
        input.primary.rtk_update_prefit_residual_rms_m = 0.1;
        input.primary.rtk_update_suppressed_outliers = 0;
        input.independent = independent(10.0);
        const auto update = gate.push(std::move(input));
        ASSERT_EQ(update.emitted.size(), 1U);
        if (epoch == 0) {
            EXPECT_EQ(update.current.consensus.state, Manager::State::NORMAL);
            EXPECT_EQ(update.emitted.front().solution.status,
                      libgnss::SolutionStatus::FIXED);
        } else {
            EXPECT_TRUE(update.current.consensus_demoted);
            EXPECT_EQ(update.emitted.front().solution.status,
                      libgnss::SolutionStatus::FLOAT);
        }
    }
}

// --- hard_primary_suspect must require a HEALTHY independent, not merely a
// present one (Fix 2: an unhealthy independent must contribute nothing to
// consensus evidence, entering OR keeping QUARANTINE). ---

TEST(RealtimeFixIntegrityGateTest, HealthyIndependentPreservesHardSuspectQuarantine) {
    RealtimeGate::Config config;
    config.enable_residual_policy = false;
    RealtimeGate gate(config);
    RealtimeGate::EpochInput input;
    input.primary = suspicious_primary_solution();
    input.independent = independent(0.0);  // healthy: trace 0.25 <= default ceiling 25.0
    const auto update = gate.push(std::move(input));
    EXPECT_TRUE(update.current.hard_primary_suspect);
    EXPECT_EQ(update.current.consensus.state, Manager::State::QUARANTINE);
    EXPECT_TRUE(update.current.consensus_demoted);
}

TEST(RealtimeFixIntegrityGateTest, UnhealthyIndependentCannotArmHardSuspect) {
    RealtimeGate::Config config;
    config.enable_residual_policy = false;
    RealtimeGate gate(config);
    RealtimeGate::EpochInput input;
    input.primary = suspicious_primary_solution();
    input.independent = unhealthy_independent(0.0);  // present, valid, but trace = +inf
    const auto update = gate.push(std::move(input));
    EXPECT_FALSE(update.current.hard_primary_suspect);
    EXPECT_EQ(update.current.consensus.state, Manager::State::NORMAL);
    EXPECT_TRUE(update.current.consensus.allow_fixed);
}

TEST(RealtimeFixIntegrityGateTest,
     UnhealthyIndependentIdenticalToAbsentForHardSuspectEntry) {
    RealtimeGate::Config config;
    config.enable_residual_policy = false;

    RealtimeGate gate_absent(config);
    RealtimeGate::EpochInput absent_input;
    absent_input.primary = suspicious_primary_solution();
    // independent left default-constructed: present == false.
    const auto absent_update = gate_absent.push(std::move(absent_input));

    RealtimeGate gate_unhealthy(config);
    RealtimeGate::EpochInput unhealthy_input;
    unhealthy_input.primary = suspicious_primary_solution();
    unhealthy_input.independent = unhealthy_independent(0.0);
    const auto unhealthy_update = gate_unhealthy.push(std::move(unhealthy_input));

    EXPECT_EQ(absent_update.current.hard_primary_suspect,
              unhealthy_update.current.hard_primary_suspect);
    EXPECT_FALSE(unhealthy_update.current.hard_primary_suspect);
    EXPECT_EQ(absent_update.current.consensus.state,
              unhealthy_update.current.consensus.state);
    EXPECT_EQ(absent_update.current.consensus.allow_fixed,
              unhealthy_update.current.consensus.allow_fixed);
}

TEST(RealtimeFixIntegrityGateTest, UnhealthyIndependentCannotClearActiveQuarantine) {
    RealtimeGate::Config config;
    config.enable_residual_policy = false;
    RealtimeGate gate(config);

    // Enter QUARANTINE via a healthy independent + hard primary suspect.
    RealtimeGate::EpochInput trigger;
    trigger.primary = suspicious_primary_solution();
    trigger.independent = independent(0.0);
    const auto triggered = gate.push(std::move(trigger));
    ASSERT_EQ(triggered.current.consensus.state, Manager::State::QUARANTINE);
    ASSERT_TRUE(triggered.current.hard_primary_suspect);

    // From here on the independent is present but permanently unhealthy
    // (e.g. an unpopulated shadow covariance trace, per the PPC tokyo1
    // regression). This must behave exactly like an absent independent:
    // estimators_agree can never become true, so QUARANTINE must never
    // clear via RECOVERY -- not "exonerated", simply inert.
    for (int i = 0; i < 10; ++i) {
        RealtimeGate::EpochInput input;
        input.primary = fixed_solution(static_cast<double>(i));
        input.independent = unhealthy_independent(static_cast<double>(i));
        const auto update = gate.push(std::move(input));
        EXPECT_EQ(update.current.consensus.state, Manager::State::QUARANTINE);
        EXPECT_FALSE(update.current.consensus.allow_fixed);
    }
}

// --- ShadowEstimateHealthGate: truth-free accuracy-bounding health gate for
// an externally produced independent position shadow (e.g. an FGO CSV dump)
// before it may act as RealtimeFixIntegrityGate consensus/demotion
// authority. ---

using HealthGate = libgnss::ShadowEstimateHealthGate;

HealthGate::Sample healthy_shadow_sample() {
    HealthGate::Sample sample;
    sample.status_fixed = true;
    sample.status_present = true;
    sample.gdop = 2.0;
    sample.ddpr_rms_m = 1.0;
    sample.num_satellites = 20;
    sample.covariance_trace_m2 = 0.5;
    sample.age_s = 0.1;
    return sample;
}

TEST(ShadowEstimateHealthGateTest, HealthySampleIsAuthorizedWithItsOwnTrace) {
    HealthGate gate;
    const auto result = gate.evaluate(healthy_shadow_sample());
    EXPECT_TRUE(result.healthy);
    EXPECT_NEAR(result.covariance_trace_m2, 0.5, 1e-12);
}

TEST(ShadowEstimateHealthGateTest, FloatStatusUnhealthyByDefault) {
    HealthGate gate;  // require_fixed_status defaults to true.
    auto sample = healthy_shadow_sample();
    sample.status_fixed = false;  // FLOAT, but status_present stays true.
    const auto result = gate.evaluate(sample);
    EXPECT_FALSE(result.healthy);
    EXPECT_TRUE(std::isinf(result.covariance_trace_m2));
}

TEST(ShadowEstimateHealthGateTest, FloatStatusAllowedWhenNotRequiredFixed) {
    HealthGate::Config config;
    config.require_fixed_status = false;
    HealthGate gate(config);
    auto sample = healthy_shadow_sample();
    sample.status_fixed = false;
    const auto result = gate.evaluate(sample);
    EXPECT_TRUE(result.healthy);
}

TEST(ShadowEstimateHealthGateTest, DroppedStatusNeverHealthy) {
    HealthGate::Config config;
    config.require_fixed_status = false;
    HealthGate gate(config);
    auto sample = healthy_shadow_sample();
    sample.status_present = false;
    const auto result = gate.evaluate(sample);
    EXPECT_FALSE(result.healthy);
}

TEST(ShadowEstimateHealthGateTest, MissingTraceDisablesAuthorityByDefault) {
    HealthGate gate;  // assume_default_covariance_trace defaults to false.
    auto sample = healthy_shadow_sample();
    sample.covariance_trace_m2.reset();
    const auto result = gate.evaluate(sample);
    EXPECT_FALSE(result.healthy);
    EXPECT_TRUE(std::isinf(result.covariance_trace_m2));
}

TEST(ShadowEstimateHealthGateTest, ZeroTraceTreatedAsUnpopulatedNotSubMillimeter) {
    // Regression: the PPC tokyo1 shipping FGO shadow CSV reports
    // position_covariance_trace_m2 == 0.000 on literally every one of its
    // 11905 rows (a placeholder, not a genuine zero-uncertainty estimate).
    // A present-but-non-positive trace must be treated the same as a
    // missing one, not as a trivially-passing "perfect" covariance.
    HealthGate gate;
    auto sample = healthy_shadow_sample();
    sample.covariance_trace_m2 = 0.0;
    const auto result = gate.evaluate(sample);
    EXPECT_FALSE(result.healthy);
}

TEST(ShadowEstimateHealthGateTest, MissingTraceAllowedWithExplicitOptIn) {
    HealthGate::Config config;
    config.assume_default_covariance_trace = true;
    config.default_covariance_trace_m2 = 2.0;
    config.max_covariance_trace_m2 = 4.0;
    HealthGate gate(config);
    auto sample = healthy_shadow_sample();
    sample.covariance_trace_m2.reset();
    const auto result = gate.evaluate(sample);
    EXPECT_TRUE(result.healthy);
    EXPECT_NEAR(result.covariance_trace_m2, 2.0, 1e-12);
}

TEST(ShadowEstimateHealthGateTest, TraceAboveCeilingUnhealthy) {
    HealthGate::Config config;
    config.max_covariance_trace_m2 = 1.0;
    HealthGate gate(config);
    auto sample = healthy_shadow_sample();
    sample.covariance_trace_m2 = 5.0;
    const auto result = gate.evaluate(sample);
    EXPECT_FALSE(result.healthy);
}

TEST(ShadowEstimateHealthGateTest, IncompleteTelemetryUnhealthy) {
    HealthGate gate;
    auto sample = healthy_shadow_sample();
    sample.gdop.reset();
    const auto result = gate.evaluate(sample);
    EXPECT_FALSE(result.healthy);
}

TEST(ShadowEstimateHealthGateTest, StaleAgeUnhealthy) {
    HealthGate::Config config;
    config.max_age_s = 0.5;
    HealthGate gate(config);
    auto sample = healthy_shadow_sample();
    sample.age_s = 0.6;
    const auto result = gate.evaluate(sample);
    EXPECT_FALSE(result.healthy);
}

}  // namespace
