#include <gtest/gtest.h>

#include <libgnss++/algorithms/ppp_clas.hpp>

using namespace libgnss;

namespace {

OSRCorrection makeCorrection() {
    OSRCorrection correction;
    correction.num_frequencies = 2;
    correction.trop_correction_m = 2.0;
    correction.relativity_correction_m = 0.3;
    correction.receiver_antenna_m[0] = 0.4;
    correction.receiver_antenna_m[1] = 0.6;
    correction.code_bias_m[0] = 1.1;
    correction.code_bias_m[1] = 1.3;
    correction.phase_bias_m[0] = 0.7;
    correction.phase_bias_m[1] = 0.9;
    correction.windup_m[0] = 0.2;
    correction.windup_m[1] = 0.25;
    correction.phase_compensation_m[0] = 0.05;
    correction.phase_compensation_m[1] = 0.08;
    correction.PRC[0] = 5.0;
    correction.PRC[1] = 7.0;
    correction.CPC[0] = 6.0;
    correction.CPC[1] = 8.0;
    return correction;
}

}  // namespace

TEST(PPPClasTest, FullOsrModeUsesPrecomputedObservationSpaceCorrections) {
    const auto correction = makeCorrection();
    const auto applied = ppp_clas::selectAppliedOsrCorrections(
        correction,
        0,
        ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::FULL_OSR);

    // Trop removed from PRC/CPC; KF estimates with tight CLAS prior.
    EXPECT_DOUBLE_EQ(applied.pseudorange_correction_m, 3.0);
    EXPECT_DOUBLE_EQ(applied.carrier_phase_correction_m, 4.0);
    EXPECT_TRUE(ppp_clas::usesClasTropospherePrior(
        ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::FULL_OSR));
}

TEST(PPPClasTest, OrbitClockBiasModeSuppressesAtmosphereButKeepsBiasTerms) {
    const auto correction = makeCorrection();
    const auto applied = ppp_clas::selectAppliedOsrCorrections(
        correction,
        1,
        ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::ORBIT_CLOCK_BIAS);

    EXPECT_DOUBLE_EQ(applied.pseudorange_correction_m, 2.2);
    EXPECT_DOUBLE_EQ(applied.carrier_phase_correction_m, 2.13);
    EXPECT_FALSE(ppp_clas::usesClasTropospherePrior(
        ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::ORBIT_CLOCK_BIAS));
}

TEST(PPPClasTest, OrbitClockOnlyModeDropsBiasAndPhaseCompensationTerms) {
    const auto correction = makeCorrection();
    const auto applied = ppp_clas::selectAppliedOsrCorrections(
        correction,
        1,
        ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::ORBIT_CLOCK_ONLY);

    EXPECT_DOUBLE_EQ(applied.pseudorange_correction_m, 0.9);
    EXPECT_DOUBLE_EQ(applied.carrier_phase_correction_m, 1.15);
    EXPECT_FALSE(ppp_clas::usesClasTropospherePrior(
        ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::ORBIT_CLOCK_ONLY));
}
