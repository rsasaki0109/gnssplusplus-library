#include <gtest/gtest.h>

#include <libgnss++/algorithms/rtk_ar_evaluation.hpp>

namespace libgnss {
namespace {

TEST(RTKArEvaluationTest, SearchPoliciesTrackRatioAndVarianceThresholds) {
    EXPECT_TRUE(rtk_ar_evaluation::shouldSearchPreferredSubsets(false, 0.0, 3.0));
    EXPECT_TRUE(rtk_ar_evaluation::shouldSearchPreferredSubsets(true, 3.2, 3.0));
    EXPECT_FALSE(rtk_ar_evaluation::shouldSearchPreferredSubsets(true, 3.6, 3.0));

    EXPECT_TRUE(rtk_ar_evaluation::shouldSearchDropSubsets(false, 0.0, 3.0, 0.01));
    EXPECT_TRUE(rtk_ar_evaluation::shouldSearchDropSubsets(true, 3.2, 3.0, 0.01));
    EXPECT_TRUE(rtk_ar_evaluation::shouldSearchDropSubsets(true, 4.0, 3.0, 0.25));
    EXPECT_FALSE(rtk_ar_evaluation::shouldSearchDropSubsets(true, 4.0, 3.0, 0.05));
}

TEST(RTKArEvaluationTest, ArFilterRequiresExtraRatioMarginOnlyWhenEnabled) {
    EXPECT_TRUE(rtk_ar_evaluation::passesArFilter(false, 3.05, 3.0, 0.25));
    EXPECT_FALSE(rtk_ar_evaluation::passesArFilter(true, 3.05, 3.0, 0.25));
    EXPECT_TRUE(rtk_ar_evaluation::passesArFilter(true, 3.30, 3.0, 0.25));
}

TEST(RTKArEvaluationTest, ExtractSubsetSlicesFloatAndCovarianceTerms) {
    Eigen::VectorXd dd_float(4);
    dd_float << 10.0, 20.0, 30.0, 40.0;
    Eigen::MatrixXd Qb(4, 4);
    Qb << 1.0, 0.1, 0.2, 0.3,
          0.1, 2.0, 0.4, 0.5,
          0.2, 0.4, 3.0, 0.6,
          0.3, 0.5, 0.6, 4.0;
    Eigen::MatrixXd Qab(2, 4);
    Qab << 1.0, 2.0, 3.0, 4.0,
           5.0, 6.0, 7.0, 8.0;

    const auto subset = rtk_ar_evaluation::extractSubset(dd_float, Qb, Qab, {1, 3});

    ASSERT_EQ(subset.dd_float.size(), 2);
    EXPECT_DOUBLE_EQ(subset.dd_float(0), 20.0);
    EXPECT_DOUBLE_EQ(subset.dd_float(1), 40.0);
    EXPECT_DOUBLE_EQ(subset.Qb(0, 0), 2.0);
    EXPECT_DOUBLE_EQ(subset.Qb(0, 1), 0.5);
    EXPECT_DOUBLE_EQ(subset.Qb(1, 0), 0.5);
    EXPECT_DOUBLE_EQ(subset.Qb(1, 1), 4.0);
    EXPECT_DOUBLE_EQ(subset.Qab(0, 0), 2.0);
    EXPECT_DOUBLE_EQ(subset.Qab(0, 1), 4.0);
    EXPECT_DOUBLE_EQ(subset.Qab(1, 0), 6.0);
    EXPECT_DOUBLE_EQ(subset.Qab(1, 1), 8.0);
}

TEST(RTKArEvaluationTest, SolveFixedHeadStateMatchesDirectLinearSolution) {
    Eigen::VectorXd head_state(2);
    head_state << 1.0, 2.0;
    Eigen::MatrixXd Qab(2, 2);
    Qab << 0.2, 0.1,
           0.0, 0.3;
    Eigen::MatrixXd Qb(2, 2);
    Qb << 2.0, 0.2,
          0.2, 1.5;
    Eigen::VectorXd dd_float(2);
    dd_float << 10.5, 11.25;
    Eigen::VectorXd dd_fixed(2);
    dd_fixed << 10.0, 11.0;

    Eigen::VectorXd fixed_head;
    ASSERT_TRUE(rtk_ar_evaluation::solveFixedHeadState(
        head_state, Qab, Qb, dd_float, dd_fixed, fixed_head));

    const Eigen::VectorXd expected =
        head_state - Qab * Qb.ldlt().solve(dd_float - dd_fixed);
    EXPECT_TRUE(fixed_head.isApprox(expected, 1e-12));
}

TEST(RTKArEvaluationTest, AdoptCandidateReplacesStoredBestState) {
    rtk_ar_evaluation::CandidateState best;
    const rtk_ar_evaluation::SubsetMatrices matrices{
        Eigen::Vector2d(1.0, 2.0),
        Eigen::Matrix2d::Identity() * 3.0,
        (Eigen::Matrix<double, 1, 2>() << 4.0, 5.0).finished(),
    };
    const Eigen::Vector2d fixed(6.0, 7.0);

    rtk_ar_evaluation::adoptCandidate(best, {2, 4}, matrices, fixed, 3.5);

    EXPECT_TRUE(best.fixed);
    EXPECT_DOUBLE_EQ(best.ratio, 3.5);
    EXPECT_EQ(best.subset, (std::vector<int>{2, 4}));
    EXPECT_TRUE(best.dd_float.isApprox(matrices.dd_float));
    EXPECT_TRUE(best.Qb.isApprox(matrices.Qb));
    EXPECT_TRUE(best.Qab.isApprox(matrices.Qab));
    EXPECT_TRUE(best.dd_fixed.isApprox(fixed));
}

}  // namespace
}  // namespace libgnss
