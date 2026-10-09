#include <gtest/gtest.h>

#include <limits>

#include <libgnss++/algorithms/rtk_covariance_consistency.hpp>

namespace libgnss::rtk_covariance_consistency {
namespace {

double statistic(const Eigen::Vector3d& d, const Eigen::Matrix3d& f,
                 const Eigen::Matrix3d& spp, double s) {
    return d.dot((s * f + spp).ldlt().solve(d));
}

TEST(RTKCovarianceConsistencyTest, ConsistentFloatIsNotInflated) {
    const Eigen::Matrix3d f = 0.04 * Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d spp = 36.0 * Eigen::Matrix3d::Identity();
    // 6 m disagreement against sigma 6 m: statistic 1, below the 3-dof mean.
    EXPECT_EQ(sppConsistencyScale(Eigen::Vector3d(6.0, 0.0, 0.0), f, spp), 1.0);
    EXPECT_EQ(sppConsistencyScale(Eigen::Vector3d::Zero(), f, spp), 1.0);
}

TEST(RTKCovarianceConsistencyTest, InconsistentFloatIsInflatedToTheExpectedStatistic) {
    const Eigen::Matrix3d f = 0.01 * Eigen::Matrix3d::Identity();   // the legacy 0.1 m
    const Eigen::Matrix3d spp = 36.0 * Eigen::Matrix3d::Identity();
    const Eigen::Vector3d d(100.0, -30.0, 5.0);                     // a 104 m disagreement
    ASSERT_GT(statistic(d, f, spp, 1.0), 3.0);
    const double s = sppConsistencyScale(d, f, spp);
    EXPECT_GT(s, 1.0);
    EXPECT_NEAR(statistic(d, f, spp, s), 3.0, 1e-6);
    // The reported per-axis sigma explains the disagreement: |d| ~ sqrt(3) sigma.
    const double sigma = std::sqrt(0.01 * s);
    EXPECT_GT(sigma, 0.2 * d.norm());
    EXPECT_LT(sigma, d.norm());
}

TEST(RTKCovarianceConsistencyTest, ScaleGrowsWithDisagreementAndShrinksWithSppUncertainty) {
    const Eigen::Matrix3d f = 0.01 * Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d tight = 4.0 * Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d loose = 400.0 * Eigen::Matrix3d::Identity();
    const Eigen::Vector3d small_gap(20.0, 0.0, 0.0), large_gap(80.0, 0.0, 0.0);
    EXPECT_LT(sppConsistencyScale(small_gap, f, tight), sppConsistencyScale(large_gap, f, tight));
    EXPECT_GT(sppConsistencyScale(large_gap, f, tight), sppConsistencyScale(large_gap, f, loose));
}

TEST(RTKCovarianceConsistencyTest, InvalidInputReportsNoEvidence) {
    const Eigen::Matrix3d f = 0.01 * Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d spp = 36.0 * Eigen::Matrix3d::Identity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(sppConsistencyScale(Eigen::Vector3d(nan, 0.0, 0.0), f, spp), 1.0);
    EXPECT_EQ(sppConsistencyScale(Eigen::Vector3d(50.0, 0.0, 0.0), f,
                                  Eigen::Matrix3d::Constant(nan)), 1.0);
    EXPECT_EQ(sppConsistencyScale(Eigen::Vector3d(50.0, 0.0, 0.0),
                                  -Eigen::Matrix3d::Identity(),
                                  Eigen::Matrix3d::Zero()), 1.0);
}

}  // namespace
}  // namespace libgnss::rtk_covariance_consistency
