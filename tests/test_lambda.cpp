// Tests for the LAMBDA solver wrapper exposed in include/libgnss++/algorithms/lambda.hpp.
// Focus: backward compatibility of lambdaSearch and the new lambdaSearchExtended
// fields (top-2 candidates, bootstrap success rate, conditional variance bounds).

#include <gtest/gtest.h>
#include <libgnss++/algorithms/lambda.hpp>

#include <Eigen/Dense>
#include <cmath>

namespace {

using libgnss::LambdaSolution;
using libgnss::lambdaSearch;
using libgnss::lambdaSearchExtended;

// Diagonal Q with given variances. Independent ambiguities, no decorrelation.
Eigen::MatrixXd diagonalQ(std::initializer_list<double> diag) {
    int n = static_cast<int>(diag.size());
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n, n);
    int i = 0;
    for (double d : diag) Q(i, i) = d, ++i;
    return Q;
}

double erfBSR(double D) { return std::erf(0.5 / std::sqrt(2.0 * D)); }

}  // namespace

TEST(LambdaSearch, BackwardCompatHighPrecision) {
    // Two ambiguities with very small variance — best candidate should round to nearest
    // integer and ratio should be very large.
    Eigen::VectorXd a(2);
    a << 3.02, -1.97;
    Eigen::MatrixXd Q = diagonalQ({1e-4, 1e-4});

    Eigen::VectorXd fixed;
    double ratio = 0.0;
    ASSERT_TRUE(lambdaSearch(a, Q, fixed, ratio));
    EXPECT_EQ(fixed.size(), 2);
    EXPECT_NEAR(fixed(0), 3.0, 1e-9);
    EXPECT_NEAR(fixed(1), -2.0, 1e-9);
    EXPECT_GT(ratio, 100.0);
}

TEST(LambdaSearchExtended, ExposesTopTwoCandidates) {
    // Float estimate halfway between two integers — top-2 should be different integers.
    Eigen::VectorXd a(2);
    a << 3.5, -2.0;
    Eigen::MatrixXd Q = diagonalQ({0.04, 0.04});  // sigma = 0.2

    LambdaSolution sol;
    ASSERT_TRUE(lambdaSearchExtended(a, Q, sol));
    EXPECT_EQ(sol.n, 2);
    EXPECT_EQ(sol.best_amb.size(), 2);
    EXPECT_EQ(sol.second_amb.size(), 2);
    // Top-2 must round to integers.
    for (int i = 0; i < 2; ++i) {
        EXPECT_NEAR(sol.best_amb(i), std::round(sol.best_amb(i)), 1e-9);
        EXPECT_NEAR(sol.second_amb(i), std::round(sol.second_amb(i)), 1e-9);
    }
    // Top-1 and top-2 must be different integer vectors.
    EXPECT_NE(sol.best_amb, sol.second_amb);
    // best_norm <= second_norm by construction of the search.
    EXPECT_LE(sol.best_norm, sol.second_norm + 1e-12);
    EXPECT_GE(sol.ratio, 1.0);
}

TEST(LambdaSearchExtended, BootstrapSuccessRateHighPrecision) {
    // sigma = 0.01 → erf(0.5/(sqrt(2)*0.01)) ≈ 1.0; product over 3 ambiguities ≈ 1.
    Eigen::VectorXd a(3);
    a << 3.0, -2.0, 5.0;
    Eigen::MatrixXd Q = diagonalQ({1e-4, 1e-4, 1e-4});

    LambdaSolution sol;
    ASSERT_TRUE(lambdaSearchExtended(a, Q, sol));
    EXPECT_GT(sol.bootstrap_sr, 0.999);
    EXPECT_LE(sol.bootstrap_sr, 1.0);
    EXPECT_GT(sol.min_cond_var, 0.0);
    EXPECT_LE(sol.min_cond_var, sol.max_cond_var + 1e-15);
}

TEST(LambdaSearchExtended, BootstrapSuccessRateLowPrecision) {
    // Variance = 1.0 → erf(0.5/sqrt(2)) ≈ 0.5205; over 3 ambiguities → ≈ 0.141.
    Eigen::VectorXd a(3);
    a << 3.0, -2.0, 5.0;
    Eigen::MatrixXd Q = diagonalQ({1.0, 1.0, 1.0});

    LambdaSolution sol;
    ASSERT_TRUE(lambdaSearchExtended(a, Q, sol));
    EXPECT_LT(sol.bootstrap_sr, 0.5);
    EXPECT_GT(sol.bootstrap_sr, 0.0);
    // Closed-form check against erf-based formula on the diagonal D[i].
    // After LAMBDA reduction on a diagonal Q the conditional variances stay
    // diagonal so we know D[i] = 1.0 for all i.
    const double expected = std::pow(erfBSR(1.0), 3);
    EXPECT_NEAR(sol.bootstrap_sr, expected, 1e-9);
}

TEST(LambdaSearchExtended, RejectsEmpty) {
    Eigen::VectorXd a;
    Eigen::MatrixXd Q;
    LambdaSolution sol;
    EXPECT_FALSE(lambdaSearchExtended(a, Q, sol));
}

TEST(LambdaSearchExtended, RatioMatchesNorms) {
    Eigen::VectorXd a(3);
    a << 1.4, 2.7, -0.3;
    Eigen::MatrixXd Q = diagonalQ({0.01, 0.04, 0.09});

    LambdaSolution sol;
    ASSERT_TRUE(lambdaSearchExtended(a, Q, sol));
    ASSERT_GT(sol.best_norm, 0.0);
    EXPECT_NEAR(sol.ratio, sol.second_norm / sol.best_norm, 1e-12);
}
