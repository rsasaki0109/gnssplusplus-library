#include <gtest/gtest.h>

#include <libgnss++/fusion/dd_imu_bridge.hpp>

namespace {
using namespace libgnss;
using namespace libgnss::dd_imu_bridge;

FusionState initialState() {
    FusionState state;
    state.covariance.setIdentity();
    state.covariance *= 4.0;
    return state;
}

DDObservation carrier(int prn, const Eigen::RowVector3d& geometry,
                      double ambiguity, int lock = 100, int system = 1,
                      int reference_prn = 20) {
    DDObservation observation;
    observation.key = {prn, 0, 0};
    observation.key.satellite_system = system;
    observation.key.reference_satellite_prn = reference_prn;
    observation.key.reference_satellite_system = system;
    observation.geometry_enu = geometry;
    observation.carrier_residual_m = 0.19 * ambiguity;
    observation.carrier_variance_m2 = 0.0025;
    observation.wavelength_m = 0.19;
    observation.elevation_rad = 0.8;
    observation.lock_count = lock;
    return observation;
}

TEST(DDIMUBridge, AmbiguityIdentitySeparatesConstellationsAndReferenceArcs) {
    DDIMUBridge bridge(initialState());
    const auto gps = carrier(1, Eigen::RowVector3d::UnitX(), 4.0, 100, 1, 20);
    const auto galileo = carrier(1, Eigen::RowVector3d::UnitY(), 5.0, 100, 2, 20);
    const auto changed_reference = carrier(1, Eigen::RowVector3d::UnitZ(), 6.0, 100, 1, 21);
    auto changed_signal = gps;
    changed_signal.key.signal_type = 3;
    changed_signal.carrier_residual_m = 0.19 * 7.0;

    ASSERT_TRUE(bridge.update({gps, galileo, changed_reference, changed_signal}).ok);
    ASSERT_EQ(bridge.state().ambiguities.size(), 4u);
    EXPECT_EQ(bridge.state().augmented_covariance.rows(), 19);

    auto slipped_gps = gps;
    slipped_gps.cycle_slip = true;
    ASSERT_TRUE(bridge.update({slipped_gps}).ok);
    // The other three arcs are not active in this epoch and are retired.
    ASSERT_EQ(bridge.state().ambiguities.size(), 1u);
    EXPECT_EQ(bridge.state().ambiguities.back().key.generation, 1);
}
}  // namespace

TEST(DDIMUBridge, JointCodeCarrierUpdateAugmentsStateAndMovesPosition) {
    DDIMUBridge bridge(initialState());
    DDObservation observation = carrier(3, Eigen::RowVector3d(1.0, 0.0, 0.0), 12.2);
    observation.code_residual_m = 2.0;
    observation.code_variance_m2 = 0.25;
    const auto result = bridge.update({observation});
    EXPECT_TRUE(result.ok);
    EXPECT_EQ(result.observation_count, 2);
    ASSERT_EQ(bridge.state().ambiguities.size(), 1u);
    EXPECT_EQ(bridge.state().augmented_covariance.rows(), 16);
    EXPECT_GT(bridge.state().eskf.nominal.position_enu.x(), 0.0);
    EXPECT_NE(bridge.state().augmented_covariance(0, 15), 0.0);
}

TEST(DDIMUBridge, RejectedCarrierFallsBackToHealthyCodeWithoutPartialARAuthority) {
    BridgeConfig config;
    config.max_nis_per_observation = 4.0;
    DDIMUBridge bridge(initialState(), config);
    DDObservation observation = carrier(
        3, Eigen::RowVector3d(1.0, 0.0, 0.0), 0.0);
    observation.code_residual_m = 0.1;
    observation.code_variance_m2 = 1.0;
    ASSERT_TRUE(bridge.update({observation}).ok);

    observation.carrier_residual_m = 100.0;
    const auto result = bridge.update({observation});
    EXPECT_TRUE(result.ok);
    EXPECT_TRUE(result.carrier_fallback_used);
    EXPECT_FALSE(result.carrier_update_accepted);
    EXPECT_FALSE(result.rejected_by_innovation_gate);
    EXPECT_EQ(result.observation_count, 1);
    EXPECT_GT(result.joint_nis_per_observation, config.max_nis_per_observation);
    EXPECT_LT(result.nis_per_observation, config.max_nis_per_observation);
}

TEST(DDIMUBridge, ShadowCarrierNeverCommitsEvenWhenJointGatePasses) {
    BridgeConfig config;
    config.commit_carrier_updates = false;
    DDIMUBridge bridge(initialState(), config);
    DDObservation observation = carrier(
        3, Eigen::RowVector3d(1.0, 0.0, 0.0), 0.0);
    observation.code_residual_m = 0.2;
    observation.code_variance_m2 = 1.0;

    const auto result = bridge.update({observation});
    EXPECT_TRUE(result.ok);
    EXPECT_TRUE(result.carrier_fallback_used);
    EXPECT_FALSE(result.carrier_update_accepted);
    EXPECT_EQ(result.observation_count, 1);
    EXPECT_NEAR(bridge.state().augmented_covariance(0, 15), 0.0, 1e-12);
}

TEST(DDIMUBridge, SSEPartialARUsesShadowPosteriorWithoutHoldingIntegers) {
    BridgeConfig config;
    config.commit_carrier_updates = false;
    config.partial_ar_min_ambiguities = 3;
    config.sse_min_fixed_ambiguities = 3;
    config.partial_ar_min_lock_count = 100;
    config.lambda_ratio_threshold = 2.0;
    DDIMUBridge bridge(initialState(), config);
    std::vector<DDObservation> observations{
        carrier(1, Eigen::RowVector3d::UnitX(), 10.01, 400),
        carrier(2, Eigen::RowVector3d::UnitY(), -3.01, 400),
        carrier(3, Eigen::RowVector3d::UnitZ(), 6.01, 400)};
    for (auto& observation : observations) {
        observation.code_residual_m = 0.0;
        observation.code_variance_m2 = 1e-8;
        observation.carrier_variance_m2 = 1e-6;
    }

    const auto update = bridge.update(observations);
    ASSERT_TRUE(update.ok);
    EXPECT_FALSE(update.carrier_update_accepted);
    const auto result =
        bridge.evaluateSSEPartialAmbiguities(observations);
    EXPECT_TRUE(result.available);
    EXPECT_TRUE(result.passed);
    EXPECT_EQ(result.fixed_count, 3);
    EXPECT_GE(result.ratio, result.ffrt_minimum_ratio);
    EXPECT_GT(result.bootstrapped_success_rate, 0.8);
    for (const auto& ambiguity : bridge.state().ambiguities) {
        EXPECT_FALSE(ambiguity.held);
    }
}

TEST(DDIMUBridge, SSEPartialARFailsClosedWhenSeparationLimitRejects) {
    BridgeConfig config;
    config.commit_carrier_updates = false;
    config.partial_ar_min_ambiguities = 3;
    config.sse_min_fixed_ambiguities = 3;
    config.partial_ar_min_lock_count = 100;
    config.lambda_ratio_threshold = 2.0;
    config.sse_max_statistic_per_dof = -1.0;
    DDIMUBridge bridge(initialState(), config);
    std::vector<DDObservation> observations{
        carrier(1, Eigen::RowVector3d::UnitX(), 10.01, 400),
        carrier(2, Eigen::RowVector3d::UnitY(), -3.01, 400),
        carrier(3, Eigen::RowVector3d::UnitZ(), 6.01, 400)};
    for (auto& observation : observations) {
        observation.code_residual_m = 0.0;
        observation.code_variance_m2 = 1e-8;
        observation.carrier_variance_m2 = 1e-6;
    }

    ASSERT_TRUE(bridge.update(observations).ok);
    const auto result =
        bridge.evaluateSSEPartialAmbiguities(observations);
    EXPECT_TRUE(result.available);
    EXPECT_FALSE(result.passed);
    EXPECT_GT(result.subsets_evaluated, 0);
}

TEST(DDIMUBridge, SSEPartialARDefaultRejectsSmallSubsets) {
    BridgeConfig config;
    config.commit_carrier_updates = false;
    config.partial_ar_min_lock_count = 100;
    DDIMUBridge bridge(initialState(), config);
    std::vector<DDObservation> observations{
        carrier(1, Eigen::RowVector3d::UnitX(), 10.01, 400),
        carrier(2, Eigen::RowVector3d::UnitY(), -3.01, 400),
        carrier(3, Eigen::RowVector3d::UnitZ(), 6.01, 400)};
    for (auto& observation : observations) {
        observation.code_residual_m = 0.0;
        observation.code_variance_m2 = 0.01;
    }

    ASSERT_TRUE(bridge.update(observations).ok);
    const auto result =
        bridge.evaluateSSEPartialAmbiguities(observations);
    EXPECT_TRUE(result.available);
    EXPECT_FALSE(result.passed);
    EXPECT_EQ(result.subsets_evaluated, 0);
}

TEST(DDIMUBridge, SlipRetiresOldAmbiguityAndCreatesFreshGeneration) {
    DDIMUBridge bridge(initialState());
    auto observation = carrier(7, Eigen::RowVector3d::UnitX(), 4.0);
    ASSERT_TRUE(bridge.update({observation}).ok);
    observation.cycle_slip = true;
    ASSERT_TRUE(bridge.update({observation}).ok);
    ASSERT_EQ(bridge.state().ambiguities.size(), 1u);
    EXPECT_EQ(bridge.state().ambiguities.back().key.generation, 1);
    EXPECT_EQ(bridge.state().augmented_covariance.rows(), 16);
    observation.cycle_slip = false;  // caller may keep the original external generation tag
    ASSERT_TRUE(bridge.update({observation}).ok);
    EXPECT_EQ(bridge.state().ambiguities.size(), 1u);
    EXPECT_EQ(bridge.state().ambiguities.back().key.generation, 1);
}

TEST(DDIMUBridge, InactiveAndChangedReferenceArcsAreRetired) {
    DDIMUBridge bridge(initialState());
    auto first = carrier(1, Eigen::RowVector3d::UnitX(), 4.0, 100, 1, 20);
    ASSERT_TRUE(bridge.update({first}).ok);
    ASSERT_EQ(bridge.state().ambiguities.size(), 1u);

    auto changed_reference = first;
    changed_reference.key.reference_satellite_prn = 21;
    ASSERT_TRUE(bridge.update({changed_reference}).ok);
    ASSERT_EQ(bridge.state().ambiguities.size(), 1u);
    EXPECT_EQ(bridge.state().ambiguities.front().key.reference_satellite_prn, 21);

    auto different_satellite = carrier(
        2, Eigen::RowVector3d::UnitY(), -2.0, 100, 1, 21);
    ASSERT_TRUE(bridge.update({different_satellite}).ok);
    ASSERT_EQ(bridge.state().ambiguities.size(), 1u);
    EXPECT_EQ(bridge.state().ambiguities.front().key.satellite_prn, 2);
}

TEST(DDIMUBridge, PropagationPreservesAmbiguityCrossCovariance) {
    DDIMUBridge bridge(initialState());
    auto observation = carrier(8, Eigen::RowVector3d::UnitX(), 5.0);
    observation.code_residual_m = 1.0;
    observation.code_variance_m2 = 0.2;
    ASSERT_TRUE(bridge.update({observation}).ok);
    const double before = bridge.state().augmented_covariance(0, 15);
    FusionState propagated = bridge.state().eskf;
    propagated.covariance *= 1.1;
    Eigen::Matrix<double, 15, 15> phi = Eigen::Matrix<double, 15, 15>::Identity();
    phi(0, 0) = 2.0;
    bridge.acceptPropagatedINS(propagated, phi);
    EXPECT_NEAR(bridge.state().augmented_covariance(0, 15), 2.0 * before, 1e-12);
}

TEST(DDIMUBridge, PartialARFixesOnlyRatioValidatedSubset) {
    BridgeConfig config;
    config.partial_ar_min_ambiguities = 3;
    config.partial_ar_min_lock_count = 100;
    config.lambda_ratio_threshold = 2.0;
    DDIMUBridge bridge(initialState(), config);
    std::vector<DDObservation> observations;
    observations.push_back(carrier(1, Eigen::RowVector3d::UnitX(), 10.02, 200));
    observations.push_back(carrier(2, Eigen::RowVector3d::UnitY(), -3.01, 190));
    observations.push_back(carrier(3, Eigen::RowVector3d::UnitZ(), 6.03, 180));
    ASSERT_TRUE(bridge.update(observations).ok);
    const auto result = bridge.resolvePartialAmbiguities(observations);
    EXPECT_TRUE(result.fixed);
    EXPECT_EQ(result.fixed_count, 3);
    EXPECT_GE(result.ratio, 2.0);
    for (const auto& ambiguity : bridge.state().ambiguities) EXPECT_TRUE(ambiguity.held);
}

TEST(DDIMUBridge, PartialARWaitsForStableLockHistory) {
    BridgeConfig config;
    config.partial_ar_min_ambiguities = 3;
    config.partial_ar_min_lock_count = 201;
    DDIMUBridge bridge(initialState(), config);
    std::vector<DDObservation> observations;
    observations.push_back(carrier(1, Eigen::RowVector3d::UnitX(), 10.02, 200));
    observations.push_back(carrier(2, Eigen::RowVector3d::UnitY(), -3.01, 200));
    observations.push_back(carrier(3, Eigen::RowVector3d::UnitZ(), 6.03, 200));
    ASSERT_TRUE(bridge.update(observations).ok);
    const auto result = bridge.resolvePartialAmbiguities(observations);
    EXPECT_FALSE(result.fixed);
    EXPECT_EQ(result.attempted, 0);
}

TEST(DDIMUBridge, SoftResetInflatesInsteadOfOverwritingValidPropagation) {
    BridgeConfig config;
    config.soft_reset_max_innovation_m = 10.0;
    config.rejected_reset_covariance_scale = 3.0;
    config.soft_reset_rejection_patience = 1;
    config.soft_reset_max_position_variance_m2 = 1.0e6;
    DDIMUBridge bridge(initialState(), config);
    const Eigen::Vector3d before = bridge.state().eskf.nominal.position_enu;
    const double variance = bridge.state().eskf.covariance(0, 0);
    EXPECT_EQ(bridge.softResetPosition(Eigen::Vector3d(100.0, 0.0, 0.0), true),
              SoftResetAction::COVARIANCE_INFLATION);
    EXPECT_TRUE(bridge.state().eskf.nominal.position_enu.isApprox(before));
    EXPECT_NEAR(bridge.state().eskf.covariance(0, 0), 3.0 * variance, 1e-12);
}

TEST(DDIMUBridge, SoftResetUsesInnovationUpdateWhenClose) {
    BridgeConfig config;
    config.soft_reset_rejection_patience = 1;
    DDIMUBridge bridge(initialState(), config);
    EXPECT_EQ(bridge.softResetPosition(Eigen::Vector3d(2.0, 0.0, 0.0), true),
              SoftResetAction::MEASUREMENT_UPDATE);
    EXPECT_GT(bridge.state().eskf.nominal.position_enu.x(), 0.0);
    EXPECT_LT(bridge.state().eskf.nominal.position_enu.x(), 2.0);
}

TEST(DDIMUBridge, SoftResetWaitsAndBoundsRepeatedCovarianceInflation) {
    BridgeConfig config;
    config.soft_reset_max_innovation_m = 10.0;
    config.soft_reset_rejection_patience = 3;
    config.rejected_reset_covariance_scale = 4.0;
    config.soft_reset_max_position_variance_m2 = 8.0;
    DDIMUBridge bridge(initialState(), config);
    const Eigen::Vector3d far(100.0, 0.0, 0.0);

    EXPECT_EQ(bridge.softResetPosition(far, true), SoftResetAction::REJECTED);
    EXPECT_EQ(bridge.softResetPosition(far, true), SoftResetAction::REJECTED);
    EXPECT_EQ(bridge.softResetPosition(far, true),
              SoftResetAction::COVARIANCE_INFLATION);
    for (int attempt = 0; attempt < 30; ++attempt) {
        bridge.softResetPosition(far, true);
    }
    EXPECT_LE(bridge.state().eskf.covariance(0, 0), 8.0);
    EXPECT_TRUE(bridge.state().eskf.nominal.position_enu.isZero());
}
