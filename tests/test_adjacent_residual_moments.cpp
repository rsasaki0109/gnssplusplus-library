#include <gtest/gtest.h>
#include <libgnss++/algorithms/adjacent_residual_moments.hpp>
#include <limits>

TEST(AdjacentResidualMoments, AlternatingAndConstantResiduals) {
    libgnss::AdjacentResidualMoments<int> alternating,constant;
    for(std::size_t i=0;i<10;++i) {
        alternating.add(1,i,i+1,i%2?1.:-1.);
        constant.add(1,i,i+1,2.);
    }
    EXPECT_EQ(alternating.pairs(),9);
    ASSERT_TRUE(alternating.correlation());
    EXPECT_NEAR(*alternating.correlation(),-1.,1e-14);
    EXPECT_FALSE(constant.correlation());
}
TEST(AdjacentResidualMoments, DoesNotBridgeGapsKeysOrInvalidRows) {
    libgnss::AdjacentResidualMoments<int> m;
    m.add(1,0,1,1.); m.add(2,1,2,2.); m.add(1,2,3,3.);
    EXPECT_EQ(m.pairs(),0);
    m.add(1,3,4,std::numeric_limits<double>::quiet_NaN());
    m.add(1,4,5,5.); EXPECT_EQ(m.pairs(),0);
    m.add(1,5,6,6.); EXPECT_EQ(m.pairs(),1);
    m.breakStream(1); m.add(1,6,7,7.); EXPECT_EQ(m.pairs(),1);
    EXPECT_THROW(m.add(1,8,8,1.),std::invalid_argument);
}
