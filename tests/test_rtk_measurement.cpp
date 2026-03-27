#include <gtest/gtest.h>

#include <libgnss++/algorithms/rtk_measurement.hpp>

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
