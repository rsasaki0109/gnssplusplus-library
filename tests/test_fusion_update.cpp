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
