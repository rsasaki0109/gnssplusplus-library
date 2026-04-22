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

}  // namespace
}  // namespace libgnss
