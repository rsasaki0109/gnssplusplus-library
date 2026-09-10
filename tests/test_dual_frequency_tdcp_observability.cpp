// Algebraic research controls, not a production FGO integration or accuracy test.
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>
#include <libgnss++/algorithms/tdcp_frequency_covariance.hpp>
#include <libgnss++/algorithms/residual_ionosphere_contract.hpp>

namespace {
// P-only local controls: receiver position is fixed. These do not establish
// observability in the complete position/clock/IMU graph.
TEST(CodeIonosphereClockObservability, EqualElevationAbsorbedByIndependentBandClocks) {
    Eigen::Matrix<double, 6, 2> clocks = Eigen::Matrix<double, 6, 2>::Zero();
    Eigen::Matrix<double, 6, 1> ionosphere;
    for (int row = 0; row < 6; ++row) {
        const int band = row % 2;
        clocks(row, band) = 1.;
        ionosphere[row] = libgnss::residual_ionosphere::signalCoefficient(
            .6, band ? 1176.45e6 : 1575.42e6);
    }
    const auto fit = clocks.colPivHouseholderQr().solve(ionosphere).eval();
    EXPECT_LT((ionosphere - clocks * fit).norm(), 1e-12);
    Eigen::Matrix<double, 6, 3> joint;
    joint << clocks, ionosphere;
    EXPECT_EQ(joint.colPivHouseholderQr().rank(), 2);
}

TEST(CodeIonosphereClockObservability, ElevationDiversityLeavesInformationAfterBandClocks) {
    Eigen::Matrix<double, 6, 2> clocks = Eigen::Matrix<double, 6, 2>::Zero();
    Eigen::Matrix<double, 6, 1> ionosphere;
    for (int row = 0; row < 6; ++row) {
        const int band = row % 2;
        clocks(row, band) = 1.;
        ionosphere[row] = libgnss::residual_ionosphere::signalCoefficient(
            .2 + .4 * (row / 2), band ? 1176.45e6 : 1575.42e6);
    }
    const auto fit = clocks.colPivHouseholderQr().solve(ionosphere).eval();
    const auto projected = (ionosphere - clocks * fit).eval();
    EXPECT_GT(projected.squaredNorm(), .1);
    EXPECT_LT((clocks.transpose() * projected).norm(), 1e-12);
    Eigen::Matrix<double, 6, 3> joint;
    joint << clocks, ionosphere;
    EXPECT_EQ(joint.colPivHouseholderQr().rank(), 3);
    // A group-constant code bias vanishes under the same projection, hence
    // elevation information cannot itself identify a group-constant ISC.
    const auto shifted = (ionosphere + clocks * Eigen::Vector2d(7., -4.)).eval();
    const auto shifted_fit = clocks.colPivHouseholderQr().solve(shifted).eval();
    EXPECT_LT((shifted - clocks * shifted_fit - projected).norm(), 1e-12);
}

double huber(double residual, double threshold = 4.0) {
    const double a = std::abs(residual);
    return a <= threshold ? .5*a*a : threshold*(a-.5*threshold);
}

TEST(DualFrequencyTdcpRobustModel, JointRadialLossChangesEvenZeroNuisanceBaseline) {
    const Eigen::Vector2d residual(4., 4.);
    const double independent = huber(residual[0])+huber(residual[1]);
    const double joint = huber(residual.norm());
    EXPECT_DOUBLE_EQ(independent, 16.0);
    EXPECT_LT(joint, independent);
    // Therefore q=0 in a joint robust two-row replacement does not recover
    // the existing independent scalar-Huber TDCP objective.
}

TEST(DualFrequencyTdcpRobustModel, GaussianEliminationIsNotRobustNuisanceElimination) {
    const Eigen::Vector2d residual(10., 0.), coefficient(1., 2.);
    const double gaussian_nuisance = coefficient.dot(residual)/6.0;
    const double robust_nuisance = .8;
    const auto objective = [&](double u) {
        return huber(residual[0]-coefficient[0]*u)+
               huber(residual[1]-coefficient[1]*u)+.5*u*u;
    };
    EXPECT_NEAR(objective(robust_nuisance), 30.4, 1e-12);
    EXPECT_LT(objective(robust_nuisance), objective(gaussian_nuisance));
    const double step = 1e-5;
    EXPECT_NEAR((objective(robust_nuisance+step)-objective(robust_nuisance-step))/(2*step),
                0., 1e-8);
}

TEST(DualFrequencyTdcpRobustModel, CorrectedCarrierRequiresResidualNotTotalIonosphere) {
    const double alpha = std::pow(1575.42/1176.45, 2);
    const double range_clock = 12., true_l1_change = .04, model_l1_change = .03;
    for (double frequency_scale : {1., alpha}) {
        const double raw_without_trop_satclock = range_clock-frequency_scale*true_l1_change;
        const double corrected = raw_without_trop_satclock+frequency_scale*model_l1_change;
        const double residual_change = true_l1_change-model_l1_change;
        EXPECT_NEAR(range_clock-frequency_scale*residual_change-corrected, 0., 1e-12);
        EXPECT_GT(std::abs(range_clock-frequency_scale*true_l1_change-corrected), .02);
    }
}

TEST(DualFrequencyTdcpCovariance, ZeroPriorVariancePreservesIndependentNoise) {
    const Eigen::Vector2d sigma(.002, .003);
    const auto n = libgnss::tdcp_frequency::marginalNoise(sigma, {1., 1.8}, 0.0);
    EXPECT_DOUBLE_EQ(n.covariance_m2(0,1), 0.0);
    EXPECT_NEAR(n.sqrt_information_per_m(0,0), 500.0, 1e-10);
    EXPECT_NEAR(n.sqrt_information_per_m(1,1), 1/.003, 1e-10);
}

TEST(DualFrequencyTdcpCovariance, MatchesExplicitGaussianNuisanceElimination) {
    const Eigen::Vector2d sigma(.002, .003), a(-1., -1.79327), r(.015, -.007);
    const double q = .0001;  // synthetic fixture, not a production tuning value
    const auto n = libgnss::tdcp_frequency::marginalNoise(sigma, a, q);
    const Eigen::Matrix2d precision = sigma.array().square().inverse().matrix().asDiagonal();
    const double estimate = a.dot(precision*r)/(a.dot(precision*a)+1/q);
    const Eigen::Vector2d remaining = r-a*estimate;
    const double explicit_cost = remaining.dot(precision*remaining)+estimate*estimate/q;
    EXPECT_NEAR((n.sqrt_information_per_m*r).squaredNorm(), explicit_cost, 1e-10);
    EXPECT_GT(n.covariance_m2(0,1), 0.0);
    EXPECT_LT((n.sqrt_information_per_m*n.covariance_m2*
               n.sqrt_information_per_m.transpose()-Eigen::Matrix2d::Identity()).norm(), 1e-12);
}

TEST(DualFrequencyTdcpCovariance, PermutingBandsPreservesCost) {
    const Eigen::Vector2d sigma(.002, .003), a(-1., -1.79327), r(.015, -.007);
    const auto n = libgnss::tdcp_frequency::marginalNoise(sigma, a, .0001);
    const auto swapped = libgnss::tdcp_frequency::marginalNoise(
        sigma.reverse().eval(), a.reverse().eval(), .0001);
    EXPECT_NEAR((n.sqrt_information_per_m*r).squaredNorm(),
                (swapped.sqrt_information_per_m*r.reverse()).squaredNorm(), 1e-10);
}

TEST(DualFrequencyTdcpCovariance, RejectsInvalidAndOverflowingInputs) {
    using libgnss::tdcp_frequency::marginalNoise;
    EXPECT_THROW(marginalNoise({0., .003}, {1., 1.8}, .01), std::invalid_argument);
    EXPECT_THROW(marginalNoise({.002, .003}, {1., 1.8}, -.01), std::invalid_argument);
    EXPECT_THROW(marginalNoise({1e308, .003}, {1., 1.8}, .01), std::invalid_argument);
    EXPECT_THROW(marginalNoise({.002, .003}, {NAN, 1.8}, .01), std::invalid_argument);
}

// Hypothesis: y_f = common_range_clock_change - alpha_f * slant_L1_change.
// Other frequency-dependent tracking errors are deliberately not modelled.
Eigen::Matrix2d design(double alpha) {
    Eigen::Matrix2d h;
    h << 1.0, -1.0, 1.0, -alpha;
    return h;
}

TEST(DualFrequencyTdcpObservability, TwoFrequenciesRecoverBothChanges) {
    const double alpha = std::pow(1575.42 / 1176.45, 2);
    const auto h = design(alpha);
    ASSERT_EQ(h.fullPivLu().rank(), 2);
    const Eigen::Vector2d expected(12.0, 0.035);
    const Eigen::Vector2d observed = h * expected;
    const Eigen::Vector2d recovered = h.fullPivLu().solve(observed);
    EXPECT_NEAR(recovered[0], expected[0], 1e-12);
    EXPECT_NEAR(recovered[1], expected[1], 1e-12);
    EXPECT_NEAR(observed[0]-observed[1], (alpha-1.0)*expected[1], 1e-12);
}

TEST(DualFrequencyTdcpObservability, SameFrequencyCannotSeparateStates) {
    EXPECT_EQ(design(1.0).fullPivLu().rank(), 1);
    Eigen::Matrix<double, 1, 2> one_band;
    one_band << 1.0, -1.0;
    EXPECT_EQ(one_band.fullPivLu().rank(), 1);
}

TEST(DualFrequencyTdcpObservability, IndependentBandBiasDestroysUniqueSeparation) {
    const auto h = design(std::pow(1575.42 / 1176.45, 2));
    Eigen::Matrix<double, 2, 3> augmented;
    augmented.leftCols<2>() = h;
    augmented.col(2) << 0.0, 1.0;
    EXPECT_EQ(augmented.fullPivLu().rank(), 2);
    EXPECT_LT(augmented.fullPivLu().rank(), augmented.cols());
    const Eigen::Vector3d null_change(1.0, 1.0, -1.0+h(1,1)*-1.0);
    EXPECT_LT((augmented*null_change).norm(), 1e-12);
}

TEST(DualFrequencyTdcpObservability, EliminatingIonosphereAmplifiesEqualIndependentNoise) {
    const double alpha = std::pow(1575.42 / 1176.45, 2);
    const auto inverse = design(alpha).inverse().eval();
    const Eigen::Matrix2d covariance = inverse * inverse.transpose();
    const double expected_common_variance = (alpha*alpha+1)/std::pow(alpha-1,2);
    EXPECT_NEAR(covariance(0,0), expected_common_variance, 1e-12);
    EXPECT_GT(covariance(0,0), 1.0);
    // Two transformed observations are correlated: do not add them back as
    // independent evidence alongside the original L1/L5 observations.
    EXPECT_GT(std::abs(covariance(0,1)), 0.0);
}
}  // namespace
