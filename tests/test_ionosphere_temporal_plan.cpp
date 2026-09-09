#include <gtest/gtest.h>
#include <libgnss++/algorithms/ionosphere_temporal_plan.hpp>
using libgnss::GNSSTime;
using libgnss::residual_ionosphere::temporalPlan;

TEST(IonosphereTemporalPlan, OneAnchorPerSegmentAndDensityUsesSquareRootTime) {
    const auto p=temporalPlan({{1,0},{1,1},{1,3},{1,7},{1,8},{1,9}},
                             {false,false,false,false,true,false},2.,.5);
    EXPECT_EQ(p.anchors,(std::vector<std::size_t>{0,3,4}));
    ASSERT_EQ(p.edges.size(),3);
    EXPECT_EQ(p.edges[0].previous,0); EXPECT_EQ(p.edges[0].current,1);
    EXPECT_DOUBLE_EQ(p.edges[0].sigma_m,.5);
    EXPECT_NEAR(p.edges[1].sigma_m,.5*std::sqrt(2.),1e-14);
    EXPECT_EQ(p.edges[2].previous,4); EXPECT_EQ(p.edges[2].current,5);
    EXPECT_EQ(p.edges.size()+p.anchors.size(),6);
}
TEST(IonosphereTemporalPlan, WeekRolloverAndEmptyInput) {
    const auto p=temporalPlan({{1,604799},{2,0}},{true,false},1.,1.);
    ASSERT_EQ(p.edges.size(),1); EXPECT_DOUBLE_EQ(p.edges[0].sigma_m,1.);
    EXPECT_TRUE(temporalPlan({}, {},1.,1.).anchors.empty());
}
TEST(IonosphereTemporalPlan, RejectsMalformedInputsEvenAtReset) {
    EXPECT_THROW(temporalPlan({{1,0}}, {},1.,1.),std::invalid_argument);
    EXPECT_THROW(temporalPlan({{1,1},{1,1}},{false,true},1.,1.),std::invalid_argument);
    EXPECT_THROW(temporalPlan({{1,1},{1,0}},{false,true},1.,1.),std::invalid_argument);
    EXPECT_THROW(temporalPlan({{1,604800}},{false},1.,1.),std::invalid_argument);
    EXPECT_THROW(temporalPlan({}, {},0.,1.),std::invalid_argument);
    EXPECT_THROW(temporalPlan({}, {},1.,-1.),std::invalid_argument);
}
