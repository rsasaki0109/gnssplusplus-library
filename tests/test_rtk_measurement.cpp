#include <gtest/gtest.h>

#include <libgnss++/algorithms/rtk_measurement.hpp>

#include <cmath>

using namespace libgnss;

TEST(RTKMeasurementTest, BuildsDoubleDifferenceCovariancePerBlock) {
    const std::vector<int> block_sizes = {2, 1};
    const std::vector<double> ref_variances = {0.1, 0.2, 0.3};
    const std::vector<double> sat_variances = {1.0, 2.0, 3.0};

    const Eigen::MatrixXd covariance =
        rtk_measurement::buildDoubleDifferenceCovariance(block_sizes, ref_variances, sat_variances, 3);

    ASSERT_EQ(covariance.rows(), 3);
    ASSERT_EQ(covariance.cols(), 3);
    EXPECT_DOUBLE_EQ(covariance(0, 0), 1.1);
    EXPECT_DOUBLE_EQ(covariance(0, 1), 0.1);
    EXPECT_DOUBLE_EQ(covariance(1, 0), 0.2);
    EXPECT_DOUBLE_EQ(covariance(1, 1), 2.2);
    EXPECT_DOUBLE_EQ(covariance(2, 2), 3.3);
    EXPECT_DOUBLE_EQ(covariance(2, 0), 0.0);
    EXPECT_DOUBLE_EQ(covariance(2, 1), 0.0);
}

TEST(RTKMeasurementTest, SuppressesOutlierRowsInResidualsAndDesignMatrix) {
    Eigen::VectorXd residuals(3);
    residuals << 0.5, 35.0, -40.0;
    Eigen::MatrixXd design = Eigen::MatrixXd::Ones(3, 4);

    const int suppressed = rtk_measurement::suppressOutlierRows(residuals, design, 30.0);

    EXPECT_EQ(suppressed, 2);
    EXPECT_DOUBLE_EQ(residuals(0), 0.5);
    EXPECT_DOUBLE_EQ(residuals(1), 0.0);
    EXPECT_DOUBLE_EQ(residuals(2), 0.0);
    EXPECT_EQ(design.row(0).sum(), 4.0);
    EXPECT_EQ(design.row(1).sum(), 0.0);
    EXPECT_EQ(design.row(2).sum(), 0.0);
}

TEST(RTKMeasurementTest, AssemblesExactSizeMeasurementSystemFromBlocks) {
    rtk_measurement::MeasurementBlock phase_block;
    rtk_measurement::MeasurementRow phase_row;
    phase_row.residual = 0.25;
    phase_row.baseline_coefficients << 1.0, 2.0, 3.0;
    phase_row.state_coefficients = {{4, 0.5}, {7, -0.25}};
    phase_row.reference_variance = 0.1;
    phase_row.satellite_variance = 1.1;
    phase_block.rows.push_back(phase_row);

    rtk_measurement::MeasurementBlock code_block;
    rtk_measurement::MeasurementRow code_row0;
    code_row0.residual = -0.75;
    code_row0.baseline_coefficients << -1.0, 0.0, 1.0;
    code_row0.state_coefficients = {{5, 2.0}};
    code_row0.reference_variance = 0.2;
    code_row0.satellite_variance = 2.2;
    code_block.rows.push_back(code_row0);

    rtk_measurement::MeasurementRow code_row1;
    code_row1.residual = 1.5;
    code_row1.baseline_coefficients << 0.0, -2.0, 0.5;
    code_row1.reference_variance = 0.3;
    code_row1.satellite_variance = 3.3;
    code_block.rows.push_back(code_row1);

    const auto system = rtk_measurement::assembleMeasurementSystem({phase_block, code_block}, 9);

    ASSERT_EQ(system.design_matrix.rows(), 3);
    ASSERT_EQ(system.design_matrix.cols(), 9);
    ASSERT_EQ(system.residuals.size(), 3);
    ASSERT_EQ(system.covariance.rows(), 3);
    ASSERT_EQ(system.covariance.cols(), 3);

    EXPECT_DOUBLE_EQ(system.residuals(0), 0.25);
    EXPECT_DOUBLE_EQ(system.residuals(1), -0.75);
    EXPECT_DOUBLE_EQ(system.residuals(2), 1.5);

    EXPECT_DOUBLE_EQ(system.design_matrix(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(system.design_matrix(0, 1), 2.0);
    EXPECT_DOUBLE_EQ(system.design_matrix(0, 2), 3.0);
    EXPECT_DOUBLE_EQ(system.design_matrix(0, 4), 0.5);
    EXPECT_DOUBLE_EQ(system.design_matrix(0, 7), -0.25);
    EXPECT_DOUBLE_EQ(system.design_matrix(1, 0), -1.0);
    EXPECT_DOUBLE_EQ(system.design_matrix(1, 2), 1.0);
    EXPECT_DOUBLE_EQ(system.design_matrix(1, 5), 2.0);
    EXPECT_DOUBLE_EQ(system.design_matrix(2, 1), -2.0);
    EXPECT_DOUBLE_EQ(system.design_matrix(2, 2), 0.5);

    EXPECT_DOUBLE_EQ(system.covariance(0, 0), 1.2);
    EXPECT_DOUBLE_EQ(system.covariance(1, 1), 2.4);
    EXPECT_DOUBLE_EQ(system.covariance(2, 2), 3.6);
    EXPECT_DOUBLE_EQ(system.covariance(1, 2), 0.2);
    EXPECT_DOUBLE_EQ(system.covariance(2, 1), 0.3);
    EXPECT_DOUBLE_EQ(system.covariance(0, 1), 0.0);
}

TEST(RTKMeasurementTest, SummarizesMeasurementBlocksByType) {
    rtk_measurement::MeasurementBlock phase_block;
    phase_block.kind = rtk_measurement::MeasurementKind::PHASE;
    phase_block.rows.push_back(rtk_measurement::MeasurementRow{0.25});
    phase_block.rows.push_back(rtk_measurement::MeasurementRow{-0.75});

    rtk_measurement::MeasurementBlock code_block;
    code_block.kind = rtk_measurement::MeasurementKind::CODE;
    code_block.rows.push_back(rtk_measurement::MeasurementRow{1.5});

    const auto diagnostics =
        rtk_measurement::summarizeMeasurementBlocks({phase_block, code_block});

    EXPECT_EQ(diagnostics.observation_count, 3);
    EXPECT_EQ(diagnostics.phase_observation_count, 2);
    EXPECT_EQ(diagnostics.code_observation_count, 1);
    EXPECT_NEAR(diagnostics.residual_rms_m, std::sqrt((0.0625 + 0.5625 + 2.25) / 3.0), 1e-12);
    EXPECT_DOUBLE_EQ(diagnostics.residual_max_abs_m, 1.5);
}

TEST(RTKMeasurementTest, BuildsAmbiguityTransformWithoutDenseDMatrix) {
    Eigen::VectorXd state(8);
    state << 10.0, 20.0, 30.0, 2.5, -1.0, 0.5, 4.0, -3.5;

    Eigen::MatrixXd covariance(8, 8);
    covariance << 5.0, 0.2, 0.1, 0.3, -0.1, 0.0, 0.2, -0.2,
                  0.2, 4.0, 0.4, 0.1, 0.2, -0.3, 0.0, 0.1,
                  0.1, 0.4, 3.5, -0.2, 0.1, 0.3, -0.1, 0.0,
                  0.3, 0.1, -0.2, 2.8, 0.5, 0.2, 0.1, -0.1,
                  -0.1, 0.2, 0.1, 0.5, 2.6, -0.4, 0.3, 0.2,
                  0.0, -0.3, 0.3, 0.2, -0.4, 2.4, 0.1, -0.2,
                  0.2, 0.0, -0.1, 0.1, 0.3, 0.1, 2.2, 0.6,
                  -0.2, 0.1, 0.0, -0.1, 0.2, -0.2, 0.6, 2.0;
    covariance = (covariance + covariance.transpose()) / 2.0;

    const std::vector<rtk_measurement::AmbiguityDifference> differences = {
        {3, 5},
        {4, 6},
        {3, 7},
    };

    const auto transform =
        rtk_measurement::buildAmbiguityTransform(state, covariance, 3, differences);

    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(8, 6);
    for (int i = 0; i < 3; ++i) D(i, i) = 1.0;
    for (size_t i = 0; i < differences.size(); ++i) {
        D(differences[i].reference_state_index, 3 + static_cast<int>(i)) = 1.0;
        D(differences[i].satellite_state_index, 3 + static_cast<int>(i)) = -1.0;
    }

    const Eigen::VectorXd y = D.transpose() * state;
    const Eigen::MatrixXd qy = D.transpose() * covariance * D;
    const Eigen::VectorXd expected_head = y.head(3);
    const Eigen::VectorXd expected_dd = y.tail(3);
    const Eigen::MatrixXd expected_qab = qy.block(0, 3, 3, 3);
    const Eigen::MatrixXd expected_qb = qy.block(3, 3, 3, 3);

    EXPECT_TRUE(transform.head_state.isApprox(expected_head, 1e-12));
    EXPECT_TRUE(transform.dd_float.isApprox(expected_dd, 1e-12));
    EXPECT_TRUE(transform.head_ambiguity_covariance.isApprox(expected_qab, 1e-12));
    EXPECT_TRUE(transform.ambiguity_covariance.isApprox(expected_qb, 1e-12));
}
