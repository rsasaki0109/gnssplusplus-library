#include <gtest/gtest.h>

#include <libgnss++/algorithms/ssr2obs.hpp>
#include <libgnss++/core/constants.hpp>

using namespace libgnss;

TEST(SSR2OBSTest, MatchesPppClasOsrSubtractionForFullOsr) {
    ObservationData raw(GNSSTime(2200, 100.0));
    Observation o(SatelliteId(GNSSSystem::GPS, 3), SignalType::GPS_L1CA);
    o.has_pseudorange = true;
    o.pseudorange = 20'000'000.0;
    o.has_carrier_phase = true;
    o.carrier_phase = 105'000.0;
    o.valid = true;
    raw.addObservation(o);

    OSRCorrection osr;
    osr.satellite = SatelliteId(GNSSSystem::GPS, 3);
    osr.valid = true;
    osr.num_frequencies = 1;
    osr.signals[0] = SignalType::GPS_L1CA;
    osr.wavelengths[0] = constants::GPS_L1_WAVELENGTH;
    osr.PRC[0] = 12.0;
    osr.CPC[0] = 15.0;
    osr.trop_correction_m = 3.0;

    ppp_shared::PPPConfig config;
    config.clas_correction_application_policy =
        ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::FULL_OSR;

    const ObservationData corrected = ssr2obs::buildSsrCorrectedObservations(
        raw, {osr}, config);

    ASSERT_EQ(corrected.observations.size(), 1u);
    const double pr_sub = 12.0 - 3.0;
    const double cp_sub_m = 15.0 - 3.0;
    EXPECT_NEAR(corrected.observations[0].pseudorange, 20'000'000.0 - pr_sub, 1e-9);
    EXPECT_NEAR(
        corrected.observations[0].carrier_phase,
        105'000.0 - cp_sub_m / constants::GPS_L1_WAVELENGTH,
        1e-9);
}

TEST(SSR2OBSTest, ClasContextHelperUsesOsrVectorFromEpochContext) {
    ObservationData raw(GNSSTime(2200, 100.0));
    Observation o(SatelliteId(GNSSSystem::GPS, 3), SignalType::GPS_L1CA);
    o.has_pseudorange = true;
    o.pseudorange = 20'000'000.0;
    o.valid = true;
    raw.addObservation(o);

    OSRCorrection osr;
    osr.satellite = SatelliteId(GNSSSystem::GPS, 3);
    osr.valid = true;
    osr.num_frequencies = 1;
    osr.signals[0] = SignalType::GPS_L1CA;
    osr.wavelengths[0] = constants::GPS_L1_WAVELENGTH;
    osr.PRC[0] = 10.0;
    osr.CPC[0] = 10.0;
    osr.trop_correction_m = 1.0;

    CLASEpochContext ctx;
    ctx.osr_corrections = {osr};

    ppp_shared::PPPConfig config;
    config.clas_correction_application_policy =
        ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::FULL_OSR;

    const ObservationData corrected = ssr2obs::buildSsrCorrectedObservationsFromClasContext(
        raw, ctx, config);
    EXPECT_NEAR(corrected.observations[0].pseudorange, 20'000'000.0 - 9.0, 1e-9);
}

TEST(SSR2OBSTest, LeavesObsUnchangedWhenNoMatchingOsr) {
    ObservationData raw(GNSSTime(2200, 100.0));
    Observation o(SatelliteId(GNSSSystem::GPS, 5), SignalType::GPS_L1CA);
    o.has_pseudorange = true;
    o.pseudorange = 1.0;
    o.valid = true;
    raw.addObservation(o);

    ppp_shared::PPPConfig config;
    const ObservationData corrected =
        ssr2obs::buildSsrCorrectedObservations(raw, {}, config);
    ASSERT_EQ(corrected.observations.size(), 1u);
    EXPECT_DOUBLE_EQ(corrected.observations[0].pseudorange, 1.0);
}
