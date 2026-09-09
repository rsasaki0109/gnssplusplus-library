#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <libgnss++/algorithms/residual_ionosphere_contract.hpp>
#include <cmath>

TEST(JointIonospherePositionBias, LowerResidualCanWorsenFreePosition) {
    // Local Gaussian counterexample: three free position columns and seven
    // independent clock-group columns (span-equivalent to C7). Not the full
    // robust temporal native graph, and not fitted to any smartphone route.
    Eigen::MatrixXd b=Eigen::MatrixXd::Zero(84,10);
    Eigen::VectorXd a(84);
    for(int g=0;g<7;++g) for(int j=0;j<12;++j) {
        const int r=g*12+j;
        const double el=.2+.09*j, az=.53*j+.31*g;
        b(r,0)=-std::cos(el)*std::cos(az);
        b(r,1)=-std::cos(el)*std::sin(az);
        b(r,2)=-std::sin(el);
        b(r,3+g)=1.;
        a[r]=libgnss::residual_ionosphere::signalCoefficient(
            el,g%2 ? 1176.45e6 : 1575.42e6);
    }
    ASSERT_EQ(b.colPivHouseholderQr().rank(),10);
    const Eigen::VectorXd component=b.colPivHouseholderQr().solve(a);
    // Construct a possible systematic error, orthogonal to baseline nuisance.
    // Truth position/clock/ionosphere are all zero; this is measurement error.
    const Eigen::VectorXd error=a-b*component;
    ASSERT_GT(error.norm(),.1);
    const Eigen::VectorXd baseline=b.colPivHouseholderQr().solve(error);
    EXPECT_LT(baseline.head(3).norm(),1e-10);
    Eigen::MatrixXd joint(84,11); joint<<b,a;
    ASSERT_EQ(joint.colPivHouseholderQr().rank(),11);
    const Eigen::VectorXd candidate=joint.colPivHouseholderQr().solve(error);
    EXPECT_LT((joint*candidate-error).norm(),1e-10);
    EXPECT_GT(candidate.head(3).norm(),.1);
    EXPECT_NEAR(candidate[10],1.,1e-10);
    EXPECT_LT((candidate.head(10)+component).norm(),1e-10);
    // Nonzero projected ionosphere information is not evidence that the
    // projected measurement error is atmospheric or that adding it helps pose.
}
