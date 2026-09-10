#include <gtest/gtest.h>
#include <libgnss++/algorithms/adjacent_residual_moments.hpp>

// Enumerate equiprobable independent +/-1 endpoint errors exactly. This
// tests algebra, not smartphone noise or a production covariance setting.
TEST(TdcpTemporalCorrelationControls, SharedEndpointCreatesNegativeHalfCorrelation) {
    libgnss::AdjacentResidualMoments<int> stats;
    int key=0;
    for(double a:{-1.,1.}) for(double b:{-1.,1.}) for(double c:{-1.,1.}) {
        stats.add(key,0,1,b-a);
        stats.add(key++,1,2,c-b);
    }
    EXPECT_EQ(stats.pairs(),8);
    ASSERT_TRUE(stats.correlation());
    EXPECT_NEAR(*stats.correlation(),-.5,1e-14);
}

TEST(TdcpTemporalCorrelationControls, StreamOffsetsCanReversePooledCorrelation) {
    libgnss::AdjacentResidualMoments<int> stats;
    int key=0;
    for(double offset:{-10.,10.})
    for(double a:{-1.,1.}) for(double b:{-1.,1.}) for(double c:{-1.,1.}) {
        stats.add(key,0,1,offset+b-a);
        stats.add(key++,1,2,offset+c-b);
    }
    ASSERT_TRUE(stats.correlation());
    // Var=100+2, Cov=100-1. Same endpoint noise as the first test.
    EXPECT_NEAR(*stats.correlation(),99./102.,1e-14);
}

TEST(TdcpTemporalCorrelationControls, FittingCommonRateChangesResidualCorrelation) {
    libgnss::AdjacentResidualMoments<int> stats;
    int key=0;
    for(double a:{-1.,1.}) for(double b:{-1.,1.}) for(double c:{-1.,1.}) {
        const double d1=b-a,d2=c-b;
        const double fitted_rate=(d1+d2)/2.;
        stats.add(key,0,1,d1-fitted_rate);
        stats.add(key++,1,2,d2-fitted_rate);
    }
    ASSERT_TRUE(stats.correlation());
    EXPECT_NEAR(*stats.correlation(),-1.,1e-14);
}
