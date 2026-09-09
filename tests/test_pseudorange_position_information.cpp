#include <gtest/gtest.h>
#include <libgnss++/algorithms/pseudorange_position_information.hpp>

using libgnss::pseudorange_information::clockProjectedInformation;
using libgnss::pseudorange_information::scalarProjectedInformation;

TEST(ScalarResidualProjection, RecoversSignedStepWithoutNuisanceContamination) {
    Eigen::MatrixXd b(5,2);
    b << 1,0, 1,1, 1,2, 1,3, 1,4;
    Eigen::VectorXd a(5), sigma(5);
    a << 1,3,2,6,4;
    sigma << 1,2,1,3,2;
    const Eigen::VectorXd r = -2.5*a + b*Eigen::Vector2d(7.,-4.);
    const auto p = libgnss::pseudorange_information::scalarResidualProjection(a,b,r,sigma);
    EXPECT_GT(p.information, 0.);
    EXPECT_NEAR(-p.coefficient_residual_dot/p.information, 2.5, 1e-12);
    EXPECT_NEAR(p.information, scalarProjectedInformation(a,b,sigma), 1e-12);
    EXPECT_NEAR(p.residual_energy, 6.25*p.information, 1e-12);
}

TEST(ScalarResidualProjection, PureNuisanceAndUnobservableCasesDoNotInventSignal) {
    const Eigen::MatrixXd b = Eigen::MatrixXd::Ones(4,1);
    const Eigen::VectorXd a = Eigen::VectorXd::LinSpaced(4,1,4);
    const auto p = libgnss::pseudorange_information::scalarResidualProjection(
        a,b,7*Eigen::Vector4d::Ones(),Eigen::Vector4d::Ones());
    EXPECT_LT(p.residual_energy, 1e-24);
    EXPECT_NEAR(p.coefficient_residual_dot, 0., 1e-12);
    const auto zero = libgnss::pseudorange_information::scalarResidualProjection(
        a,Eigen::Matrix4d::Identity(),a,Eigen::Vector4d::Ones());
    EXPECT_LT(zero.information, 1e-24);
    EXPECT_THROW(libgnss::pseudorange_information::scalarResidualProjection(
        a,b,Eigen::Vector3d::Zero(),Eigen::Vector4d::Ones()),std::invalid_argument);
}

TEST(ScalarProjectedInformation, MatchesWeightedExplicitNuisanceFit) {
    Eigen::MatrixXd n(5,2);
    n << 1,0, 1,1, 1,2, 1,3, 1,4;
    Eigen::VectorXd a(5), sigma(5);
    a << 1,3,2,6,4;
    sigma << 1,2,1,3,2;
    const Eigen::MatrixXd b = n.array().colwise() / sigma.array();
    const Eigen::VectorXd y = a.array() / sigma.array();
    const Eigen::VectorXd fit = b.colPivHouseholderQr().solve(y);
    const double expected = (y-b*fit).squaredNorm();
    EXPECT_NEAR(scalarProjectedInformation(a,n,sigma), expected, 1e-12);
    EXPECT_NEAR(scalarProjectedInformation(a,n,2*sigma), expected/4, 1e-12);
    Eigen::MatrixXd redundant(5,3);
    redundant << n, n.col(0);
    EXPECT_NEAR(scalarProjectedInformation(a,redundant,sigma), expected, 1e-12);
    EXPECT_LT(scalarProjectedInformation(n.col(1),n,sigma),1e-24);
}

TEST(ScalarProjectedInformation, RejectsInvalidAndHandlesEmptyNuisance) {
    EXPECT_DOUBLE_EQ(scalarProjectedInformation(Eigen::Vector2d(3,4),
        Eigen::MatrixXd(2,0), Eigen::Vector2d::Ones()),25.);
    EXPECT_DOUBLE_EQ(scalarProjectedInformation(Eigen::VectorXd(0),
        Eigen::MatrixXd(0,10), Eigen::VectorXd(0)),0.);
    EXPECT_THROW(scalarProjectedInformation(Eigen::Vector2d::Ones(),
        Eigen::MatrixXd::Ones(3,1), Eigen::Vector2d::Ones()),std::invalid_argument);
    EXPECT_THROW(scalarProjectedInformation(Eigen::Vector2d::Ones(),
        Eigen::MatrixXd::Ones(2,1), Eigen::Vector2d(0,1)),std::invalid_argument);
}

TEST(PseudorangePositionInformation, SharedClockRemovesCommonPositionComponent) {
    Eigen::MatrixXd a(4,3);
    a << 1,0,0, 0,1,0, 0,0,1, -1,0,0;
    const Eigen::MatrixXd b = Eigen::MatrixXd::Ones(4,1);
    const Eigen::VectorXd sigma = Eigen::VectorXd::Ones(4);
    const Eigen::Matrix3d expected = a.transpose() *
        (Eigen::Matrix4d::Identity()-Eigen::Matrix4d::Constant(.25)) * a;
    const auto actual = clockProjectedInformation(a,b,sigma);
    EXPECT_LT((actual-expected).norm(),1e-12);
    EXPECT_GT(actual.eigenvalues().real().minCoeff(),0.0);
    EXPECT_LT((clockProjectedInformation(a,b,2*sigma)-actual/4).norm(),1e-12);
}

TEST(PseudorangePositionInformation, IndependentClocksRemoveAllPositionInformation) {
    const Eigen::MatrixXd a = Eigen::MatrixXd::Random(4,3);
    EXPECT_LT(clockProjectedInformation(a,Eigen::Matrix4d::Identity(),
                                       Eigen::Vector4d::Ones()).norm(),1e-24);
}

TEST(PseudorangePositionInformation, RepeatedLineOfSightHasNoClockFreePositionSupport) {
    Eigen::MatrixXd a(8,3);
    for (int i=0; i<8; ++i) a.row(i) << 0.6,0.8,0.0;
    const auto info = clockProjectedInformation(a,Eigen::MatrixXd::Ones(8,1),
                                               Eigen::VectorXd::LinSpaced(8,1,3));
    EXPECT_LT(info.norm(),1e-24);
}

TEST(PseudorangePositionInformation, PlanarGeometryLeavesOnePositionDirectionUnobserved) {
    Eigen::MatrixXd a(4,3);
    a << 1,0,0, -1,0,0, 0,1,0, 0,-1,0;
    const auto info = clockProjectedInformation(a,Eigen::MatrixXd::Ones(4,1),
                                               Eigen::Vector4d::Ones());
    Eigen::Matrix3d expected=Eigen::Matrix3d::Zero();
    expected(0,0)=expected(1,1)=2;
    EXPECT_LT((info-expected).norm(),1e-12);
}

TEST(PseudorangePositionInformation, AbsentAndRedundantClockSlotsAreNotPriors) {
    const Eigen::MatrixXd a = Eigen::MatrixXd::Random(5,3);
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(5,7);
    b.col(0).setOnes(); b.col(2).setConstant(2);
    EXPECT_LT((clockProjectedInformation(a,b,Eigen::VectorXd::Ones(5))-
               clockProjectedInformation(a,Eigen::MatrixXd::Ones(5,1),
                                         Eigen::VectorXd::Ones(5))).norm(),1e-12);
}

TEST(PseudorangePositionInformation, InvalidAndEmptyInputs) {
    EXPECT_EQ(clockProjectedInformation(Eigen::MatrixXd(0,3),Eigen::MatrixXd(0,7),
                                        Eigen::VectorXd(0)).norm(),0);
    EXPECT_THROW(clockProjectedInformation(Eigen::MatrixXd::Zero(2,3),
        Eigen::MatrixXd::Zero(2,1),Eigen::VectorXd::Zero(2)),std::invalid_argument);
}

TEST(PseudorangePositionInformation, C7WeightedProjectionIsInvariantToClockGauge) {
    Eigen::MatrixXd a = Eigen::MatrixXd::Random(12,3);
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(12,7);
    // Native C7 selector: base plus selected component, assignment not
    // addition for component zero. Inactive columns stay zero.
    for (int row=0; row<12; ++row) {
        b(row,0)=1;
        b(row,row%3)=1;
    }
    const Eigen::VectorXd sigma = Eigen::VectorXd::LinSpaced(12,0.5,4.0);
    const auto expected = clockProjectedInformation(a,b,sigma);
    const Eigen::MatrixXd shifted = a + b * Eigen::MatrixXd::Random(7,3);
    EXPECT_LT((clockProjectedInformation(shifted,b,sigma)-expected).norm(),1e-11);
    const Eigen::MatrixXd no_clock(12,0);
    const auto unprojected = clockProjectedInformation(a,no_clock,sigma);
    EXPECT_GE((unprojected-expected).selfadjointView<Eigen::Lower>()
                  .eigenvalues().minCoeff(),-1e-11);
}
