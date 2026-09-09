#include <gtest/gtest.h>
#include <libgnss++/algorithms/ionosphere_temporal_plan.hpp>
#include <cmath>

// Synthetic identifiability control, not a fit to any route or a noise policy.
// A code error parallel to the ionosphere coefficient is indistinguishable
// from ionosphere in code alone. A temporal walk cannot reject its constant
// component. Hold geometry/clocks fixed to isolate that ambiguity.
TEST(JointIonosphereConstantMode, TemporalPriorOnlyPenalizesConstantOnce) {
    using namespace libgnss;
    std::vector<GNSSTime> times(100);
    for (std::size_t i=0;i<times.size();++i) {
        times[i].week=2200; times[i].tow=100.+i;
    }
    const auto plan=residual_ionosphere::temporalPlan(
        times,std::vector<bool>(times.size(),false),1.5,0.02);
    ASSERT_EQ(plan.anchors.size(),1u);
    ASSERT_EQ(plan.edges.size(),99u);
    const double constant=3., anchor_sigma=3.;
    double energy=plan.anchors.size()*std::pow(constant/anchor_sigma,2);
    for (const auto& edge:plan.edges)
        energy+=std::pow((constant-constant)/edge.sigma_m,2);
    EXPECT_DOUBLE_EQ(energy,1.);
    // Equal endpoint coefficients: constant state gives zero TDCP correction.
    EXPECT_DOUBLE_EQ(2.*constant-2.*constant,0.);
    // Changing mapping does constrain the constant mode; it is not a universal
    // null direction of the actual graph.
    EXPECT_NE(2.*constant-2.1*constant,0.);
}

TEST(JointIonosphereConstantMode, RepeatedCodeBiasCanOverwhelmSingleAnchor) {
    const double bias=2., code_sigma=1., anchor_sigma=3.;
    const auto optimum=[&](double rows) {
        return (rows*bias/(code_sigma*code_sigma))/
            (rows/(code_sigma*code_sigma)+1./(anchor_sigma*anchor_sigma));
    };
    EXPECT_GT(optimum(1000),optimum(1));
    EXPECT_NEAR(optimum(1000),bias,0.001);
    // This exact toy result proves no causal attribution for U; actual code
    // has robust losses and free geometry/clocks, unlike this control.
}
