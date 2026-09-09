#include <gtest/gtest.h>
#include <libgnss++/algorithms/tdcp_endpoint_covariance.hpp>
#include <Eigen/Cholesky>
#include <limits>

TEST(TdcpEndpointCovariance, PairSigmaUsesBothMetreEndpointsAndRejectsMissing) {
    using libgnss::tdcp_endpoint_covariance::pairSigma;
    ASSERT_TRUE(pairSigma(.003,.004));
    EXPECT_NEAR(*pairSigma(.003,.004),.005,1e-15);
    EXPECT_EQ(pairSigma(.003,.004),pairSigma(.004,.003));
    EXPECT_FALSE(pairSigma(0.,.004));
    EXPECT_FALSE(pairSigma(.003,std::numeric_limits<double>::quiet_NaN()));
}

TEST(TdcpEndpointCovariance, ExactDifferencingAndPositiveDefiniteness) {
    const auto c=libgnss::tdcp_endpoint_covariance::make({.001,.002,.003,.004});
    ASSERT_TRUE(c);
    Eigen::Matrix<double,3,4> d;
    d<<-1,1,0,0, 0,-1,1,0, 0,0,-1,1;
    Eigen::Vector4d v; v<<1e-6,4e-6,9e-6,16e-6;
    EXPECT_LT((*c-d*v.asDiagonal()*d.transpose()).norm(),1e-19);
    Eigen::LLT<Eigen::MatrixXd> llt(*c);
    EXPECT_EQ(llt.info(),Eigen::Success);
    EXPECT_DOUBLE_EQ((*c)(0,2),0.);
}
TEST(TdcpEndpointCovariance, SummedDifferencesRetainOnlyOuterEndpointNoise) {
    const auto c=libgnss::tdcp_endpoint_covariance::make({.001,.002,.003,.004});
    ASSERT_TRUE(c);
    EXPECT_NEAR(c->sum(),17e-6,1e-19);
    EXPECT_GT(c->diagonal().sum(),c->sum());
}
TEST(TdcpEndpointCovariance, EqualNoiseGivesNegativeHalfAdjacentCorrelation) {
    const auto c=libgnss::tdcp_endpoint_covariance::make({.003,.003,.003});
    ASSERT_TRUE(c);
    EXPECT_NEAR((*c)(0,1)/std::sqrt((*c)(0,0)*(*c)(1,1)),-.5,1e-15);
}
TEST(TdcpEndpointCovariance, RejectsMissingInvalidAndOverflowingUncertainty) {
    using libgnss::tdcp_endpoint_covariance::make;
    EXPECT_FALSE(make({})); EXPECT_FALSE(make({.001}));
    EXPECT_FALSE(make({0.,.001})); EXPECT_FALSE(make({-.001,.001}));
    EXPECT_FALSE(make({std::numeric_limits<double>::quiet_NaN(),.001}));
    EXPECT_FALSE(make({1e308,.001})); EXPECT_FALSE(make({1e-300,.001}));
}
