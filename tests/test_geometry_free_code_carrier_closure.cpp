#include <gtest/gtest.h>
#include <array>

namespace {
// Synthetic synchronized two-band observables in metres. Receiver/satellite
// geometry and clock are common; frequency-dependent hardware terms are not.
std::array<double,4> observables(double common, double iono,
                                double code1=0., double code2=0.,
                                double phase1=0., double phase2=0.) {
    constexpr double gamma=1.8;
    return {common+iono+code1,common+gamma*iono+code2,
            common-iono+phase1,common-gamma*iono+phase2};
}
double closure(const std::array<double,4>& before,
               const std::array<double,4>& after) {
    const auto gf=[](const auto& x){return (x[0]-x[1])+(x[2]-x[3]);};
    return gf(after)-gf(before);
}
}

TEST(GeometryFreeCodeCarrierClosure, CommonMotionAndDispersiveChangeCancel) {
    EXPECT_NEAR(closure(observables(100.,2.),observables(107.,3.)),0.,1e-12);
}
TEST(GeometryFreeCodeCarrierClosure, DifferentialCodeJumpRemains) {
    EXPECT_NEAR(closure(observables(100.,2.),observables(107.,3.,4.)),4.,1e-12);
}
TEST(GeometryFreeCodeCarrierClosure, ConstantBiasIsInvisible) {
    EXPECT_NEAR(closure(observables(100.,2.,20.,-5.,9.,3.),
                        observables(107.,3.,20.,-5.,9.,3.)),0.,1e-12);
}
TEST(GeometryFreeCodeCarrierClosure, CarrierSlipCanMimicCodeJump) {
    EXPECT_NEAR(closure(observables(100.,2.),observables(107.,3.,0.,0.,4.)),4.,1e-12);
}
TEST(GeometryFreeCodeCarrierClosure, CommonCodeErrorIsInvisible) {
    EXPECT_NEAR(closure(observables(100.,2.),observables(107.,3.,4.,4.)),0.,1e-12);
}
TEST(GeometryFreeCodeCarrierClosure, DifferentialClockChangeDoesNotCancel) {
    // A first-band clock term shared by code and phase contributes twice.
    EXPECT_NEAR(closure(observables(100.,2.),observables(107.,3.,4.,0.,4.)),8.,1e-12);
}
