#include <gtest/gtest.h>

#include <libgnss++/algorithms/rtk_update.hpp>

#include <cmath>

namespace libgnss {
namespace {

TEST(RTKUpdateTest, RejectsMeasurementSystemsWithTooFewObservations) {
    Eigen::VectorXd state = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(4, 4);
    rtk_measurement::MeasurementSystem system;
    system.design_matrix = Eigen::MatrixXd::Zero(5, 4);
    system.residuals = Eigen::VectorXd::Zero(5);
    system.covariance = Eigen::MatrixXd::Identity(5, 5);

    const auto result =
        rtk_update::applyMeasurementUpdate(state, covariance, system, 30.0, 6);

    EXPECT_FALSE(result.ok);
    EXPECT_EQ(result.observation_count, 5);
    EXPECT_EQ(result.suppressed_outliers, 0);
    EXPECT_DOUBLE_EQ(result.prefit_residual_rms_m, 0.0);
    EXPECT_DOUBLE_EQ(result.prefit_residual_max_abs_m, 0.0);
}

TEST(RTKUpdateTest, AppliesKalmanUpdateAndSuppressesOutliers) {
    Eigen::VectorXd state = Eigen::VectorXd::Constant(3, 0.05);
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(3, 3);
    rtk_measurement::MeasurementSystem system;
    system.design_matrix = Eigen::MatrixXd::Zero(6, 3);
    system.residuals = Eigen::VectorXd::Zero(6);
    system.covariance = Eigen::MatrixXd::Identity(6, 6);

    system.design_matrix.row(0) << 1.0, 0.0, 0.0;
    system.design_matrix.row(1) << 0.0, 1.0, 0.0;
    system.design_matrix.row(2) << 0.0, 0.0, 1.0;
    system.design_matrix.row(3) << 1.0, 1.0, 0.0;
    system.design_matrix.row(4) << 0.0, 1.0, 1.0;
    system.design_matrix.row(5) << 1.0, 0.0, 1.0;
    system.residuals.setConstant(0.2);
    system.residuals(0) = 100.0;

    const auto result =
        rtk_update::applyMeasurementUpdate(state, covariance, system, 30.0, 6);

    ASSERT_TRUE(result.ok);
    EXPECT_EQ(result.observation_count, 6);
    EXPECT_EQ(result.suppressed_outliers, 1);
    EXPECT_NEAR(result.prefit_residual_rms_m, std::sqrt((10000.0 + 5.0 * 0.04) / 6.0), 1e-12);
    EXPECT_DOUBLE_EQ(result.prefit_residual_max_abs_m, 100.0);
    EXPECT_NEAR(result.post_suppression_residual_rms_m, std::sqrt((5.0 * 0.04) / 6.0), 1e-12);
    EXPECT_DOUBLE_EQ(result.post_suppression_residual_max_abs_m, 0.2);
    EXPECT_NEAR(system.residuals(0), 0.0, 1e-12);
    EXPECT_TRUE(system.design_matrix.row(0).isZero(0.0));
    EXPECT_GT(state(0), 0.05);
    EXPECT_LT(covariance(0, 0), 1.0);
}

TEST(RTKUpdateTest, RejectsLargeNormalizedInnovationBeforeKalmanUpdate) {
    Eigen::VectorXd state = Eigen::VectorXd::Constant(2, 1.0);
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(2, 2);
    const Eigen::VectorXd original_state = state;
    const Eigen::MatrixXd original_covariance = covariance;

    rtk_measurement::MeasurementSystem system;
    system.design_matrix = Eigen::MatrixXd::Identity(2, 2);
    system.residuals = Eigen::VectorXd::Zero(2);
    system.residuals(0) = 10.0;
    system.covariance = Eigen::MatrixXd::Identity(2, 2);

    const auto result =
        rtk_update::applyMeasurementUpdate(state, covariance, system, 30.0, 2, 10.0);

    EXPECT_FALSE(result.ok);
    EXPECT_TRUE(result.rejected_by_innovation_gate);
    EXPECT_EQ(result.innovation_observation_count, 2);
    EXPECT_NEAR(result.normalized_innovation_squared, 50.0, 1e-12);
    EXPECT_NEAR(result.normalized_innovation_squared_per_observation, 25.0, 1e-12);
    EXPECT_TRUE(state.isApprox(original_state, 0.0));
    EXPECT_TRUE(covariance.isApprox(original_covariance, 0.0));
}

TEST(RTKUpdateTest, ForceActiveMaskUpdatesAZeroValuedState) {
    Eigen::VectorXd state = Eigen::VectorXd::Zero(2);
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(2, 2);
    rtk_measurement::MeasurementSystem system;
    system.design_matrix = Eigen::MatrixXd::Identity(2, 2);
    system.residuals = Eigen::VectorXd::Constant(2, 1.0);
    system.covariance = Eigen::MatrixXd::Identity(2, 2);

    const std::vector<bool> force_active{true, true};
    const auto result = rtk_update::applyMeasurementUpdate(
        state, covariance, system, 30.0, 2, 0.0, force_active);

    ASSERT_TRUE(result.ok);
    EXPECT_TRUE(state.isApprox(Eigen::VectorXd::Constant(2, 0.5), 1e-12));
    EXPECT_TRUE(covariance.isApprox(0.5 * Eigen::MatrixXd::Identity(2, 2), 1e-12));
}

TEST(RTKUpdateTest, ApplyMeasurementUpdateExportsRowStatsOnlyWhenRequested) {
    Eigen::VectorXd state = Eigen::VectorXd::Constant(2, 1.0);
    Eigen::MatrixXd covariance = 2.0 * Eigen::MatrixXd::Identity(2, 2);
    rtk_measurement::MeasurementSystem system;
    system.design_matrix = Eigen::MatrixXd::Identity(2, 2);
    system.residuals = Eigen::VectorXd::Constant(2, 0.5);
    system.covariance = Eigen::MatrixXd::Identity(2, 2);

    // Default path: no row stats allocated.
    {
        Eigen::VectorXd s = state;
        Eigen::MatrixXd p = covariance;
        auto sys = system;
        const auto result = rtk_update::applyMeasurementUpdate(s, p, sys, 30.0, 2);
        ASSERT_TRUE(result.ok);
        EXPECT_EQ(result.row_innovations.size(), 0);
        EXPECT_EQ(result.row_hph_diagonal.size(), 0);
    }

    // Opt-in path: prefit innovations and diag(H P- H') exported.
    {
        Eigen::VectorXd s = state;
        Eigen::MatrixXd p = covariance;
        auto sys = system;
        const auto result = rtk_update::applyMeasurementUpdate(
            s, p, sys, 30.0, 2, 0.0, {}, /*compute_row_stats=*/true);
        ASSERT_TRUE(result.ok);
        ASSERT_EQ(result.row_innovations.size(), 2);
        ASSERT_EQ(result.row_hph_diagonal.size(), 2);
        EXPECT_DOUBLE_EQ(result.row_innovations(0), 0.5);
        EXPECT_DOUBLE_EQ(result.row_innovations(1), 0.5);
        // H = I, P- = 2I -> diag(H P- H') = 2 (prior covariance, not posterior).
        EXPECT_DOUBLE_EQ(result.row_hph_diagonal(0), 2.0);
        EXPECT_DOUBLE_EQ(result.row_hph_diagonal(1), 2.0);
    }
}

TEST(RTKUpdateTest, RejectsWrongSizedForceActiveMaskWithoutMutation) {
    Eigen::VectorXd state = Eigen::VectorXd::Zero(2);
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(2, 2);
    rtk_measurement::MeasurementSystem system;
    system.design_matrix = Eigen::MatrixXd::Identity(2, 2);
    system.residuals = Eigen::VectorXd::Ones(2);
    system.covariance = Eigen::MatrixXd::Identity(2, 2);
    const Eigen::VectorXd state_before = state;
    const Eigen::MatrixXd covariance_before = covariance;

    const auto result = rtk_update::applyMeasurementUpdate(
        state, covariance, system, 30.0, 2, 0.0, std::vector<bool>{true});

    EXPECT_FALSE(result.ok);
    EXPECT_TRUE(state.isApprox(state_before, 0.0));
    EXPECT_TRUE(covariance.isApprox(covariance_before, 0.0));
}

TEST(RTKUpdateTest, ReusedKalmanFactorizationPreservesUpdateAndRowStats) {
    Eigen::VectorXd legacy_state(3);
    legacy_state << 0.4, -0.2, 0.0;
    Eigen::MatrixXd legacy_covariance(3, 3);
    legacy_covariance <<
        2.0, 0.2, 0.0,
        0.2, 1.5, 0.0,
        0.0, 0.0, 1.0;
    rtk_measurement::MeasurementSystem legacy_system;
    legacy_system.design_matrix.resize(4, 3);
    legacy_system.design_matrix <<
        1.0, 0.2,  0.5,
       -0.3, 1.0, -0.2,
        0.7, 0.4,  0.1,
       -0.2, 0.8,  0.6;
    legacy_system.residuals.resize(4);
    legacy_system.residuals << 0.3, -0.1, 0.2, 0.05;
    legacy_system.covariance =
        0.5 * Eigen::MatrixXd::Identity(4, 4);
    legacy_system.covariance(0, 1) = legacy_system.covariance(1, 0) = 0.04;
    const std::vector<bool> force_active{false, false, true};

    auto reused_state = legacy_state;
    auto reused_covariance = legacy_covariance;
    auto reused_system = legacy_system;
    const auto legacy = rtk_update::applyMeasurementUpdate(
        legacy_state, legacy_covariance, legacy_system, 30.0, 4, 0.0,
        force_active, /*compute_row_stats=*/true);
    const auto reused = rtk_update::applyMeasurementUpdate(
        reused_state, reused_covariance, reused_system, 30.0, 4, 0.0,
        force_active, /*compute_row_stats=*/true,
        /*reuse_kalman_factorization_for_nis=*/true);

    ASSERT_TRUE(legacy.ok);
    ASSERT_TRUE(reused.ok);
    EXPECT_TRUE(reused_state.isApprox(legacy_state, 0.0));
    EXPECT_TRUE(reused_covariance.isApprox(legacy_covariance, 0.0));
    EXPECT_TRUE(reused.row_innovations.isApprox(legacy.row_innovations, 0.0));
    EXPECT_TRUE(reused.row_hph_diagonal.isApprox(legacy.row_hph_diagonal, 0.0));
    EXPECT_NEAR(reused.normalized_innovation_squared,
                legacy.normalized_innovation_squared, 1e-12);
}

TEST(RTKUpdateTest, SequentialIndependentBlocksMatchJointUpdate) {
    Eigen::VectorXd joint_state(4);
    joint_state << 0.5, -0.3, 0.2, 0.1;
    Eigen::MatrixXd joint_covariance(4, 4);
    joint_covariance <<
        2.0, 0.2, 0.1, 0.0,
        0.2, 1.8, 0.0, 0.1,
        0.1, 0.0, 1.5, 0.2,
        0.0, 0.1, 0.2, 1.2;
    rtk_measurement::MeasurementSystem first;
    first.design_matrix.resize(4, 4);
    first.design_matrix <<
        1.0,  0.2, 0.0, -0.1,
       -0.3,  1.0, 0.2,  0.0,
        0.4, -0.2, 1.0,  0.1,
        0.1,  0.3, 0.0,  1.0;
    first.residuals.resize(4);
    first.residuals << 0.2, -0.1, 0.3, 0.05;
    first.covariance = 0.4 * Eigen::MatrixXd::Identity(4, 4);
    first.covariance(0, 1) = first.covariance(1, 0) = 0.03;

    rtk_measurement::MeasurementSystem second;
    second.design_matrix.resize(3, 4);
    second.design_matrix <<
        0.0, 0.0,  1.0,  0.3,
        0.0, 0.0, -0.2,  1.0,
        0.0, 0.0,  0.7, -0.4;
    second.residuals.resize(3);
    second.residuals << 0.12, -0.08, 0.04;
    second.covariance = 0.25 * Eigen::MatrixXd::Identity(3, 3);
    second.covariance(0, 2) = second.covariance(2, 0) = 0.02;

    rtk_measurement::MeasurementSystem joint;
    joint.design_matrix.resize(7, 4);
    joint.design_matrix.topRows(4) = first.design_matrix;
    joint.design_matrix.bottomRows(3) = second.design_matrix;
    joint.residuals.resize(7);
    joint.residuals.head(4) = first.residuals;
    joint.residuals.tail(3) = second.residuals;
    joint.covariance = Eigen::MatrixXd::Zero(7, 7);
    joint.covariance.topLeftCorner(4, 4) = first.covariance;
    joint.covariance.bottomRightCorner(3, 3) = second.covariance;

    auto sequential_state = joint_state;
    auto sequential_covariance = joint_covariance;
    const Eigen::VectorXd prior_state = sequential_state;
    ASSERT_TRUE(rtk_update::applyMeasurementUpdate(
        joint_state, joint_covariance, joint, 30.0, 7).ok);
    ASSERT_TRUE(rtk_update::applyMeasurementUpdate(
        sequential_state, sequential_covariance, first, 30.0, 4).ok);
    second.residuals -=
        second.design_matrix * (sequential_state - prior_state);
    ASSERT_TRUE(rtk_update::applyMeasurementUpdate(
        sequential_state, sequential_covariance, second, 30.0, 3).ok);

    EXPECT_TRUE(sequential_state.isApprox(joint_state, 1e-12));
    EXPECT_TRUE(sequential_covariance.isApprox(joint_covariance, 1e-12));
}

TEST(RTKUpdateTest, StudentTInflationPreservesMeasurementCorrelation) {
    Eigen::VectorXd residuals(2);
    residuals << 1.0, 0.1;
    Eigen::MatrixXd covariance(2, 2);
    covariance << 0.04, 0.01,
                  0.01, 0.04;
    const double correlation_before =
        covariance(0, 1) /
        std::sqrt(covariance(0, 0) * covariance(1, 1));
    rtk_update::StudentTFrontEndConfig config;
    config.enabled = true;
    const auto result =
        rtk_update::applyStudentTMeasurementWeights(
            residuals, covariance, config);
    const double correlation_after =
        covariance(0, 1) /
        std::sqrt(covariance(0, 0) * covariance(1, 1));

    EXPECT_TRUE(result.applied);
    EXPECT_EQ(result.downweighted_rows, 1);
    EXPECT_NEAR(result.minimum_weight, 5.0 / 29.0, 1e-12);
    EXPECT_NEAR(correlation_after, correlation_before, 1e-12);
    EXPECT_GT(covariance(0, 0), 0.04);
    EXPECT_NEAR(covariance(1, 1), 0.04, 1e-12);
}

TEST(RTKUpdateTest, StudentTDefaultOffIsExactNoOp) {
    Eigen::VectorXd residuals(1);
    residuals << 100.0;
    Eigen::MatrixXd covariance(1, 1);
    covariance << 2.0;
    const Eigen::MatrixXd before = covariance;

    const auto result =
        rtk_update::applyStudentTMeasurementWeights(
            residuals, covariance, {});

    EXPECT_FALSE(result.applied);
    EXPECT_TRUE(covariance.isApprox(before, 0.0));
}

TEST(RTKUpdateTest, StudentTUsesInnovationNotOnlyMeasurementVariance) {
    Eigen::VectorXd residuals(1);
    residuals << 1.0;
    Eigen::VectorXd innovation_variances(1);
    innovation_variances << 4.0;
    Eigen::MatrixXd covariance(1, 1);
    covariance << 0.04;
    rtk_update::StudentTFrontEndConfig config;
    config.enabled = true;

    const auto result =
        rtk_update::applyStudentTMeasurementWeights(
            residuals, innovation_variances, covariance, config);

    EXPECT_TRUE(result.applied);
    EXPECT_EQ(result.downweighted_rows, 0);
    EXPECT_DOUBLE_EQ(result.minimum_weight, 1.0);
    EXPECT_DOUBLE_EQ(covariance(0, 0), 0.04);
}

TEST(RTKUpdateTest, StudentTLeavesCentralInnovationExactlyUntouched) {
    Eigen::VectorXd residuals(2);
    residuals << 2.49, -2.5;
    Eigen::VectorXd innovation_variances =
        Eigen::VectorXd::Ones(2);
    Eigen::MatrixXd covariance(2, 2);
    covariance << 1.0, 0.25,
                  0.25, 1.0;
    const Eigen::MatrixXd before = covariance;
    rtk_update::StudentTFrontEndConfig config;
    config.enabled = true;

    const auto result =
        rtk_update::applyStudentTMeasurementWeights(
            residuals, innovation_variances, covariance, config);

    EXPECT_TRUE(result.applied);
    EXPECT_EQ(result.downweighted_rows, 1);
    EXPECT_DOUBLE_EQ(covariance(0, 0), before(0, 0));
    EXPECT_GT(covariance(1, 1), before(1, 1));
}

TEST(RTKUpdateTest, StudentTDegreesOneUsesCauchyTailWeight) {
    Eigen::VectorXd residuals(1);
    residuals << 4.0;
    Eigen::VectorXd innovation_variances =
        Eigen::VectorXd::Ones(1);
    Eigen::MatrixXd covariance =
        Eigen::MatrixXd::Identity(1, 1);
    rtk_update::StudentTFrontEndConfig config;
    config.enabled = true;
    config.degrees_of_freedom = 1.0;

    const auto result =
        rtk_update::applyStudentTMeasurementWeights(
            residuals, innovation_variances, covariance, config);

    EXPECT_TRUE(result.applied);
    EXPECT_EQ(result.downweighted_rows, 1);
    EXPECT_NEAR(result.minimum_weight, 2.0 / 17.0, 1e-12);
    EXPECT_NEAR(covariance(0, 0), 8.5, 1e-12);
}

TEST(RTKUpdateTest, LaplacianModelUsesL1IrlsWeight) {
    Eigen::VectorXd residuals(1);
    residuals << 4.0;
    Eigen::VectorXd innovation_variances =
        Eigen::VectorXd::Ones(1);
    Eigen::MatrixXd covariance =
        Eigen::MatrixXd::Identity(1, 1);
    rtk_update::StudentTFrontEndConfig config;
    config.enabled = true;
    config.weight_model =
        rtk_update::HeavyTailWeightModel::LAPLACIAN;

    const auto result =
        rtk_update::applyStudentTMeasurementWeights(
            residuals, innovation_variances, covariance, config);

    EXPECT_TRUE(result.applied);
    EXPECT_EQ(result.downweighted_rows, 1);
    EXPECT_NEAR(result.minimum_weight, 0.25, 1e-12);
    EXPECT_NEAR(covariance(0, 0), 4.0, 1e-12);
}

TEST(RTKUpdateTest, HuberModelIsContinuousAtActivationThreshold) {
    Eigen::VectorXd residuals(2);
    residuals << 2.5, 5.0;
    Eigen::VectorXd innovation_variances =
        Eigen::VectorXd::Ones(2);
    Eigen::MatrixXd covariance =
        Eigen::MatrixXd::Identity(2, 2);
    rtk_update::StudentTFrontEndConfig config;
    config.enabled = true;
    config.weight_model =
        rtk_update::HeavyTailWeightModel::HUBER;
    config.activation_threshold_sigma = 2.5;

    const auto result =
        rtk_update::applyStudentTMeasurementWeights(
            residuals, innovation_variances, covariance, config);

    EXPECT_TRUE(result.applied);
    EXPECT_EQ(result.downweighted_rows, 1);
    EXPECT_NEAR(result.minimum_weight, 0.5, 1e-12);
    EXPECT_NEAR(covariance(0, 0), 1.0, 1e-12);
    EXPECT_NEAR(covariance(1, 1), 2.0, 1e-12);
}

}  // namespace
}  // namespace libgnss
