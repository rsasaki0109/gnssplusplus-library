#include <gtest/gtest.h>
#include <libgnss++/algorithms/code_ionosphere_state_factor.hpp>
#include <libgnss++/algorithms/joint_ionosphere_graph_plan.hpp>
#include "../src/algorithms/fgo_gtsam_internal.hpp"

namespace {
using libgnss::code_ionosphere::ResidualStateFactor;
auto original() {
    return std::make_shared<libgnss::fgo_gtsam_internal::PseudorangeFactorSourceClockArm>(
        1, 2, 45., gtsam::Point3(20.,30.,40.),
        gtsam::gnss::LeverArm(gtsam::Point3(.2,-.1,.3)), 4,
        libgnss::fgo_gtsam_internal::makeNoise(2.,true,.2));
}
gtsam::Values fixture() {
    gtsam::Values x;
    x.insert(1,gtsam::Pose3(gtsam::Rot3::RzRyRx(.1,.2,.3),gtsam::Point3(1.,2.,3.)));
    x.insert(2,gtsam::Vector(gtsam::Vector::Constant(7,.1)));
    x.insert(3,0.);
    return x;
}
TEST(JointIonosphereGraphPlan, StagesReplacementAndOneTemporalConstraintPerState) {
    auto x=fixture();
    gtsam::NonlinearFactorGraph graph; graph.push_back(original());
    const double cost=graph.error(x);
    const auto plan=libgnss::code_ionosphere::planGraph(graph,x,
        {{1,0},{1,1},{1,5}}, {false,false,false}, {10,11,12},
        {{0,0,1.8}}, {},10.,1.,2.);
    ASSERT_EQ(plan.replacements.size(),1);
    ASSERT_EQ(plan.priors.size(),3); // two anchors and one walk
    EXPECT_EQ(plan.states.size(),3);
    EXPECT_EQ(graph.size(),1); EXPECT_EQ(x.size(),3);
    EXPECT_DOUBLE_EQ(graph.error(x),cost);
    graph[plan.replacements[0].first]=plan.replacements[0].second;
    graph.push_back(plan.priors); x.insert(plan.states);
    EXPECT_EQ(graph.size(),4);
    EXPECT_DOUBLE_EQ(graph.error(x),cost); // zero states, no duplicate data
}
TEST(JointIonosphereGraphPlan, InvalidBindingsLeaveOriginalUntouched) {
    const auto x=fixture();
    gtsam::NonlinearFactorGraph graph; graph.push_back(original());
    using libgnss::code_ionosphere::planGraph;
    EXPECT_THROW(planGraph(graph,x,{{1,0}},{false},{10},{{0,0,1.},{0,0,1.}},
        {},10.,1.,2.),std::invalid_argument);
    EXPECT_THROW(planGraph(graph,x,{{1,0}},{false},{1},{{0,0,1.}},
        {},10.,1.,2.),std::invalid_argument);
    EXPECT_EQ(graph.size(),1); EXPECT_EQ(x.size(),3);
}
TEST(CodeIonosphereStateFactor, ZeroStatePreservesActualPoseC7Factor) {
    const auto base=original();
    ResidualStateFactor factor(base,3,1.8);
    const auto x=fixture();
    std::vector<gtsam::Matrix> a(2),b(3);
    EXPECT_LT((factor.unwhitenedError(x,b)-base->unwhitenedError(x,a)).norm(),1e-14);
    EXPECT_EQ(factor.noiseModel().get(),base->noiseModel().get());
    EXPECT_DOUBLE_EQ(factor.error(x),base->error(x));
    ASSERT_EQ(b.size(),3);
    EXPECT_LT((a[0]-b[0]).norm(),1e-14);
    EXPECT_LT((a[1]-b[1]).norm(),1e-14);
    EXPECT_DOUBLE_EQ(b[2](0,0),1.8);
}
TEST(CodeIonosphereStateFactor, PositiveCodeDelayAndDerivativeHaveCorrectSign) {
    const auto base=original();
    ResidualStateFactor factor(base,3,1.8);
    auto x=fixture();
    x.update(3,2.);
    EXPECT_NEAR((factor.unwhitenedError(x)-base->unwhitenedError(x))[0],3.6,1e-12);
    x.update(3,2.+1e-5);
    const double plus=factor.unwhitenedError(x)[0];
    x.update(3,2.-1e-5);
    EXPECT_NEAR((plus-factor.unwhitenedError(x)[0])/2e-5,1.8,1e-8);
}
TEST(CodeIonosphereStateFactor, RejectsBadCoefficientCollisionAndNonfiniteState) {
    const auto base=original();
    EXPECT_THROW(ResidualStateFactor(base,2,1.),std::invalid_argument);
    EXPECT_THROW(ResidualStateFactor(nullptr,3,1.),std::invalid_argument);
    for(double a:{0.,-1.,double(NAN),double(INFINITY)})
        EXPECT_THROW(ResidualStateFactor(base,3,a),std::invalid_argument);
    ResidualStateFactor factor(base,3,1.8);
    auto x=fixture();
    x.update(3,std::numeric_limits<double>::quiet_NaN());
    EXPECT_THROW(factor.unwhitenedError(x),std::invalid_argument);
    EXPECT_TRUE(factor.equals(*factor.clone()));
}
}
