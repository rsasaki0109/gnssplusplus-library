#include <gtest/gtest.h>

#include <libgnss++/fusion/fusion_update.hpp>

#include <cmath>

namespace libgnss {
namespace {

fusion_measurement::FusionMeasurementSystem makeIdentity3System(const Eigen::Vector3d& residual,
                                                                double sigma) {
    fusion_measurement::FusionMeasurementSystem system;
    system.design_matrix = Eigen::MatrixXd::Zero(3, fusion_index::SIZE);
    system.design_matrix.block<3, 3>(0, fusion_index::POSITION) = Eigen::Matrix3d::Identity();
    system.residuals = residual;
    system.covariance = (sigma * sigma) * Eigen::Matrix3d::Identity();
    return system;
}

TEST(FusionUpdateTest, AppliesUpdateEvenWhenErrorStateIsAllZeros) {
    // The specific regression this module exists to fix vs. reusing
    // kalman.hpp's kalmanFilter(): an all-zero error state must NOT be
    // treated as "no active states" (docs/design.md 0.4, 3.5).
    Eigen::Matrix<double, 15, 1> error_state = Eigen::Matrix<double, 15, 1>::Zero();
    Eigen::Matrix<double, 15, 15> covariance = Eigen::Matrix<double, 15, 15>::Identity();

    const auto system = makeIdentity3System(Eigen::Vector3d(1.0, -0.5, 0.25), 0.1);
    const auto result = fusion_update::applyDenseUpdate(error_state, covariance, system);

    ASSERT_TRUE(result.ok);
    EXPECT_EQ(result.observation_count, 3);
    EXPECT_FALSE(result.rejected_by_innovation_gate);
    // Error state must have moved away from zero toward the residual.
    EXPECT_GT(error_state.segment<3>(fusion_index::POSITION).norm(), 0.0);
    // Covariance for the observed (position) block should shrink.
    EXPECT_LT(covariance(fusion_index::POSITION, fusion_index::POSITION), 1.0);
    EXPECT_LT(covariance(fusion_index::POSITION + 1, fusion_index::POSITION + 1), 1.0);
    EXPECT_LT(covariance(fusion_index::POSITION + 2, fusion_index::POSITION + 2), 1.0);
}

TEST(FusionUpdateTest, ConsiderMaskLeavesMaskedStatesUncorrectedAndKeepsAValidCovariance) {
    // Position measurement; the attitude and gyro-bias states are correlated
    // with position, so a plain update would correct them.
    Eigen::Matrix<double, 15, 15> base_cov = Eigen::Matrix<double, 15, 15>::Identity();
    base_cov.block<3, 3>(fusion_index::POSITION, fusion_index::ATTITUDE) =
        0.5 * Eigen::Matrix3d::Identity();
    base_cov.block<3, 3>(fusion_index::ATTITUDE, fusion_index::POSITION) =
        0.5 * Eigen::Matrix3d::Identity();
    base_cov.block<3, 3>(fusion_index::POSITION, fusion_index::GYRO_BIAS) =
        0.3 * Eigen::Matrix3d::Identity();
    base_cov.block<3, 3>(fusion_index::GYRO_BIAS, fusion_index::POSITION) =
        0.3 * Eigen::Matrix3d::Identity();
    const auto system = makeIdentity3System(Eigen::Vector3d(1.0, -2.0, 0.5), 0.1);

    Eigen::Matrix<double, 15, 1> plain_error = Eigen::Matrix<double, 15, 1>::Zero();
    Eigen::Matrix<double, 15, 15> plain_cov = base_cov;
    ASSERT_TRUE(fusion_update::applyDenseUpdate(plain_error, plain_cov, system).ok);
    EXPECT_GT(plain_error.segment<3>(fusion_index::ATTITUDE).norm(), 0.1);

    // No mask (the default argument and the explicit constant) is the plain
    // update, bit for bit.
    Eigen::Matrix<double, 15, 1> zero_mask_error = Eigen::Matrix<double, 15, 1>::Zero();
    Eigen::Matrix<double, 15, 15> zero_mask_cov = base_cov;
    ASSERT_TRUE(fusion_update::applyDenseUpdate(zero_mask_error, zero_mask_cov, system, 0.0,
                                                fusion_update::kNoConsiderMask)
                    .ok);
    EXPECT_EQ(zero_mask_error, plain_error);
    EXPECT_EQ(zero_mask_cov, plain_cov);

    // Attitude and both bias states are consider states.
    ASSERT_EQ(fusion_update::kConsiderAttitudeAndBiasesMask, 0x7fc0u);
    Eigen::Matrix<double, 15, 1> error = Eigen::Matrix<double, 15, 1>::Zero();
    Eigen::Matrix<double, 15, 15> cov = base_cov;
    ASSERT_TRUE(fusion_update::applyDenseUpdate(error, cov, system, 0.0,
                                                fusion_update::kConsiderAttitudeAndBiasesMask)
                    .ok);
    const Eigen::Matrix<double, 9, 1> consider_error = error.tail<9>();
    EXPECT_EQ(consider_error.norm(), 0.0);
    // The position correction does not depend on the consider rows.
    EXPECT_NEAR((error.head<3>() - plain_error.head<3>()).norm(), 0.0, 1e-12);
    const double position_trace = cov.block<3, 3>(0, 0).trace();
    const double base_position_trace = base_cov.block<3, 3>(0, 0).trace();
    EXPECT_LT(position_trace, base_position_trace);
    // The consider states keep their own variance (no information gained).
    EXPECT_NEAR((cov.block<9, 9>(6, 6) - base_cov.block<9, 9>(6, 6)).norm(), 0.0, 1e-12);
    EXPECT_LT((cov - cov.transpose()).norm(), 1e-12);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> eig(cov);
    EXPECT_GT(eig.eigenvalues().minCoeff(), 0.0);
}

TEST(FusionUpdateTest, RejectsLargeNormalizedInnovationBeforeUpdate) {
    Eigen::Matrix<double, 15, 1> error_state = Eigen::Matrix<double, 15, 1>::Zero();
    Eigen::Matrix<double, 15, 15> covariance = Eigen::Matrix<double, 15, 15>::Identity();
    const Eigen::Matrix<double, 15, 1> original_error_state = error_state;
    const Eigen::Matrix<double, 15, 15> original_covariance = covariance;

    // Residual (10,0,0) with unit measurement noise and unit prior
    // covariance: S = P + R = 2*I, NIS = v^T S^-1 v = 100/2 = 50, per-obs = 50/3.
    const auto system = makeIdentity3System(Eigen::Vector3d(10.0, 0.0, 0.0), 1.0);
    const auto result = fusion_update::applyDenseUpdate(error_state, covariance, system, 1.0);

    EXPECT_FALSE(result.ok);
    EXPECT_TRUE(result.rejected_by_innovation_gate);
    EXPECT_EQ(result.observation_count, 3);
    EXPECT_NEAR(result.normalized_innovation_squared, 50.0, 1e-9);
    EXPECT_NEAR(result.normalized_innovation_squared_per_observation, 50.0 / 3.0, 1e-9);
    EXPECT_TRUE(error_state.isApprox(original_error_state, 0.0));
    EXPECT_TRUE(covariance.isApprox(original_covariance, 0.0));
}

TEST(FusionUpdateTest, RejectsEmptyMeasurementSystem) {
    Eigen::Matrix<double, 15, 1> error_state = Eigen::Matrix<double, 15, 1>::Zero();
    Eigen::Matrix<double, 15, 15> covariance = Eigen::Matrix<double, 15, 15>::Identity();

    fusion_measurement::FusionMeasurementSystem system;
    system.design_matrix = Eigen::MatrixXd::Zero(0, fusion_index::SIZE);
    system.residuals = Eigen::VectorXd::Zero(0);
    system.covariance = Eigen::MatrixXd::Zero(0, 0);

    const auto result = fusion_update::applyDenseUpdate(error_state, covariance, system);
    EXPECT_FALSE(result.ok);
    EXPECT_EQ(result.observation_count, 0);
}

TEST(FusionUpdateTest, RejectsNonPositiveInnovationCovarianceWithoutMutation) {
    Eigen::Matrix<double, 15, 1> error_state = Eigen::Matrix<double, 15, 1>::Zero();
    Eigen::Matrix<double, 15, 15> covariance = Eigen::Matrix<double, 15, 15>::Identity();
    const auto original_error_state = error_state;

    auto system = makeIdentity3System(Eigen::Vector3d(1.0, -2.0, 0.5), 0.1);
    // The first observed state has P+R < 0. LDLT used to return a finite
    // nonsense solve here, producing a tiny/negative NIS and accepting the
    // update. The dense fusion update must reject it before any mutation.
    covariance(fusion_index::POSITION, fusion_index::POSITION) = -1.0;
    const auto original_covariance = covariance;
    const auto result = fusion_update::applyDenseUpdate(error_state, covariance, system, 1.0);

    EXPECT_FALSE(result.ok);
    EXPECT_FALSE(result.rejected_by_innovation_gate);
    EXPECT_TRUE(result.rejected_by_invalid_innovation_covariance);
    EXPECT_TRUE(error_state.isApprox(original_error_state, 0.0));
    EXPECT_TRUE(covariance.isApprox(original_covariance, 0.0));
}

TEST(FusionUpdateTest, JosephFormCovarianceStaysSymmetricAndPositiveSemiDefiniteAfterManyUpdates) {
    Eigen::Matrix<double, 15, 1> error_state = Eigen::Matrix<double, 15, 1>::Zero();
    Eigen::Matrix<double, 15, 15> covariance = Eigen::Matrix<double, 15, 15>::Identity();

    for (int i = 0; i < 500; ++i) {
        const double residual_x = (i % 7 == 0) ? 0.3 : -0.1;
        const auto system =
            makeIdentity3System(Eigen::Vector3d(residual_x, 0.05, -0.02), 0.2);
        const auto result = fusion_update::applyDenseUpdate(error_state, covariance, system);
        ASSERT_TRUE(result.ok);
        // Reset the error state each iteration, mirroring
        // LooseCouplingProcessor's inject-then-reset usage pattern.
        error_state.setZero();

        ASSERT_TRUE(covariance.isApprox(covariance.transpose(), 1e-9))
            << "covariance asymmetric at iteration " << i;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> solver(covariance);
        ASSERT_EQ(solver.info(), Eigen::Success);
        ASSERT_GE(solver.eigenvalues().minCoeff(), -1e-9)
            << "covariance lost positive-semi-definiteness at iteration " << i;
    }
}

}  // namespace
}  // namespace libgnss
