// Diagnostic algebra using the actual GTSAM Huber implementation.
#include <gtest/gtest.h>
#include <gtsam/linear/LossFunctions.h>
#include <libgnss++/algorithms/pseudorange_position_information.hpp>

TEST(IonosphereHuberInformation, UsesStandardizedResidualAndSquareRootWeight) {
    const auto huber = gtsam::noiseModel::mEstimator::Huber::Create(.2);
    const double sigma = 2., residual = 4.;
    const double weight = huber->weight(residual / sigma);
    EXPECT_NEAR(weight, .1, 1e-14);
    const double effective_sigma = sigma / std::sqrt(weight);
    EXPECT_NEAR(1. / (effective_sigma * effective_sigma), weight / (sigma * sigma), 1e-14);
    EXPECT_DOUBLE_EQ(huber->weight(0.), 1.);
    EXPECT_DOUBLE_EQ(huber->weight(-2.), weight);
}

TEST(IonosphereHuberInformation, DownweightingCannotIncreaseProjectedAbsoluteInformation) {
    Eigen::MatrixXd nuisance(5,2);
    nuisance << 1,0, 1,1, 1,2, 1,3, 1,4;
    Eigen::VectorXd coefficient(5), sigma(5), residual(5);
    coefficient << 1,3,2,6,4;
    sigma << 1,2,1,3,2;
    residual << .01,4.,.1,15.,1.;
    const auto huber = gtsam::noiseModel::mEstimator::Huber::Create(.2);
    Eigen::VectorXd effective = sigma;
    for (int i = 0; i < 5; ++i)
        effective[i] /= std::sqrt(huber->weight(residual[i] / sigma[i]));
    using libgnss::pseudorange_information::scalarProjectedInformation;
    const double nominal = scalarProjectedInformation(coefficient,nuisance,sigma);
    const double irls = scalarProjectedInformation(coefficient,nuisance,effective);
    EXPECT_GT(irls, 0.);
    EXPECT_LT(irls, nominal);
    // Absolute information is monotone; the remaining/total ratio need not be.
}

TEST(IonosphereHuberInformation, IrlsInformationIsNotExactTailCurvature) {
    const auto huber = gtsam::noiseModel::mEstimator::Huber::Create(.2);
    const double r = 2., h = .001;
    const double curvature = (huber->loss(r+h)-2*huber->loss(r)+huber->loss(r-h))/(h*h);
    EXPECT_NEAR(curvature, 0., 1e-8);
    EXPECT_GT(huber->weight(r), 0.);
}
