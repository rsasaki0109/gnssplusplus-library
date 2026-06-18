#include <gtest/gtest.h>

#include <libgnss++/algorithms/madoca_parity.hpp>
#include <libgnss++/algorithms/ppp_bias_identity.hpp>
#include <libgnss++/algorithms/ppp_osr.hpp>
#include <libgnss++/core/coordinates.hpp>

#include <chrono>
#include <cmath>
#include <ctime>
#include <map>

using namespace libgnss;

namespace {

GNSSTime makeTime(int year, int month, int day, int hour, int minute, double second) {
    std::tm epoch_tm{};
    epoch_tm.tm_year = year - 1900;
    epoch_tm.tm_mon = month - 1;
    epoch_tm.tm_mday = day;
    epoch_tm.tm_hour = hour;
    epoch_tm.tm_min = minute;
    epoch_tm.tm_sec = static_cast<int>(std::floor(second));
    const time_t unix_seconds = timegm(&epoch_tm);
    const auto tp = std::chrono::system_clock::from_time_t(unix_seconds) +
        std::chrono::microseconds(
            static_cast<long long>(std::llround((second - std::floor(second)) * 1e6)));
    return GNSSTime::fromSystemTime(tp);
}

SSROrbitClockCorrection makeAtmosCorrection(
    const SatelliteId& satellite,
    const GNSSTime& time,
    int network_id) {
    SSROrbitClockCorrection correction;
    correction.satellite = satellite;
    correction.time = time;
    correction.atmos_valid = true;
    correction.atmos_network_id = network_id;
    correction.atmos_tokens = {
        {"atmos_network_id", std::to_string(network_id)},
        {"atmos_trop_t00_m", "0.1"},
    };
    return correction;
}

Vector3d receiverPositionNearClasNetwork9() {
    return geodetic2ecef(39.98056 * M_PI / 180.0, 141.22509 * M_PI / 180.0, 0.0);
}

Observation makeGpsL2Observation(
    const SatelliteId& satellite,
    const char* pseudorange_code,
    const char* carrier_code,
    double pseudorange,
    double carrier_phase) {
    Observation obs(satellite, SignalType::GPS_L2C);
    obs.valid = true;
    obs.has_pseudorange = true;
    obs.has_carrier_phase = true;
    obs.pseudorange_observation_type = pseudorange_code;
    obs.carrier_phase_observation_type = carrier_code;
    obs.pseudorange = pseudorange;
    obs.carrier_phase = carrier_phase;
    return obs;
}

}  // namespace

TEST(PPPOSRTest, GridFirstPrefersNearestGridEvenWhenStale) {
    SSRProducts ssr_products;
    const GNSSTime epoch_time = makeTime(2018, 11, 24, 0, 0, 0.0);
    const SatelliteId sat_a(GNSSSystem::GPS, 1);
    const SatelliteId sat_b(GNSSSystem::GPS, 2);
    ssr_products.addCorrection(makeAtmosCorrection(sat_a, epoch_time - 90.0, 9));
    ssr_products.addCorrection(makeAtmosCorrection(sat_b, epoch_time - 1.0, 2));

    ppp_shared::PPPConfig config;
    config.clas_atmos_selection_policy =
        ppp_shared::PPPConfig::ClasAtmosSelectionPolicy::GRID_FIRST;
    config.clas_atmos_stale_after_seconds = 15.0;

    const auto selected = selectClasEpochAtmosTokens(
        ssr_products,
        std::vector<SatelliteId>{sat_a, sat_b},
        epoch_time,
        receiverPositionNearClasNetwork9(),
        config);
    EXPECT_EQ(preferredClasNetworkId(selected), 9);
}

TEST(PPPOSRTest, ExactObservationIdentitySelectsGpsL2wBiasId) {
    namespace bias = libgnss::algorithms::ppp_bias_identity;
    namespace mp = libgnss::algorithms::madoca_parity;

    EXPECT_EQ(bias::rtklibCodeForObservationType("C2W"), mp::kCodeL2W);
    EXPECT_EQ(bias::rtklibCodeForObservationType("L2W"), mp::kCodeL2W);
    EXPECT_EQ(bias::rtklibCodeForObservationType("C2X"), mp::kCodeL2X);

    EXPECT_EQ(
        static_cast<int>(bias::madocaBiasIdentityIdForObservation(
            GNSSSystem::GPS, SignalType::GPS_L2C, "C2W", true)),
        mp::kCodeL2W);
    EXPECT_EQ(
        static_cast<int>(bias::madocaBiasIdentityIdForObservation(
            GNSSSystem::GPS, SignalType::GPS_L2C, "L2W", true)),
        mp::kCodeL2W);
    EXPECT_EQ(
        static_cast<int>(bias::madocaBiasIdentityIdForObservation(
            GNSSSystem::GPS, SignalType::GPS_L2C, "C2X", true)),
        mp::kCodeL2X);
    EXPECT_EQ(
        static_cast<int>(bias::madocaBiasIdentityIdForObservation(
            GNSSSystem::GPS, SignalType::GPS_L2C, "C2W", false)),
        8);
    EXPECT_EQ(
        static_cast<int>(bias::rtcmSsrSignalIdForObservation(
            GNSSSystem::GPS, SignalType::GPS_L2C, "C2W", true)),
        9);
    EXPECT_EQ(
        static_cast<int>(bias::rtcmSsrSignalIdForObservation(
            GNSSSystem::GPS, SignalType::GPS_L2C, "L2W", true)),
        9);
    EXPECT_EQ(
        static_cast<int>(bias::rtcmSsrSignalIdForObservation(
            GNSSSystem::GPS, SignalType::GPS_L2C, "C2X", true)),
        8);
    EXPECT_TRUE(
        bias::isGpsL2wObservation(
            GNSSSystem::GPS, SignalType::GPS_L2C, "C2W", "L2W"));
    EXPECT_FALSE(
        bias::isGpsL2wObservation(
            GNSSSystem::GPS, SignalType::GPS_L2C, "C2X", "L2X"));
}

TEST(PPPOSRTest, ExactOsrFrequencyLookupUsesStoredRinexIdentity) {
    const SatelliteId sat(GNSSSystem::GPS, 14);
    ObservationData obs(GNSSTime(2068, 230425.0));
    obs.addObservation(makeGpsL2Observation(sat, "C2X", "L2X", 100.0, 10.0));
    obs.addObservation(makeGpsL2Observation(sat, "C2W", "L2W", 200.0, 20.0));

    OSRCorrection osr;
    osr.satellite = sat;
    osr.num_frequencies = 1;
    osr.signals[0] = SignalType::GPS_L2C;
    osr.pseudorange_rinex_codes[0] = "C2W";
    osr.carrier_rinex_codes[0] = "L2W";
    osr.bias_exact_identity[0] = true;

    const Observation* selected = findOsrFrequencyObservation(obs, osr, 0);
    ASSERT_NE(selected, nullptr);
    EXPECT_EQ(selected->pseudorange_observation_type, "C2W");
    EXPECT_EQ(selected->carrier_phase_observation_type, "L2W");
    EXPECT_DOUBLE_EQ(selected->pseudorange, 200.0);
}

TEST(PPPOSRTest, ExactOsrFrequencyLookupSkipsInvalidStoredRinexIdentity) {
    const SatelliteId sat(GNSSSystem::GPS, 14);
    ObservationData obs(GNSSTime(2068, 230425.0));
    obs.addObservation(makeGpsL2Observation(sat, "C2X", "L2X", 100.0, 10.0));
    Observation invalid_exact = makeGpsL2Observation(sat, "C2W", "L2W", 200.0, 20.0);
    invalid_exact.valid = false;
    obs.addObservation(invalid_exact);

    OSRCorrection osr;
    osr.satellite = sat;
    osr.num_frequencies = 1;
    osr.signals[0] = SignalType::GPS_L2C;
    osr.pseudorange_rinex_codes[0] = "C2W";
    osr.carrier_rinex_codes[0] = "L2W";
    osr.bias_exact_identity[0] = true;

    const Observation* selected = findOsrFrequencyObservation(obs, osr, 0);
    ASSERT_NE(selected, nullptr);
    EXPECT_EQ(selected->pseudorange_observation_type, "C2X");
    EXPECT_EQ(selected->carrier_phase_observation_type, "L2X");
    EXPECT_DOUBLE_EQ(selected->pseudorange, 100.0);
}

TEST(PPPOSRTest, OsrFrequencyLookupPreservesSignalTypeFallbackWhenExactGateIsOff) {
    const SatelliteId sat(GNSSSystem::GPS, 14);
    ObservationData obs(GNSSTime(2068, 230425.0));
    obs.addObservation(makeGpsL2Observation(sat, "C2X", "L2X", 100.0, 10.0));
    obs.addObservation(makeGpsL2Observation(sat, "C2W", "L2W", 200.0, 20.0));

    OSRCorrection osr;
    osr.satellite = sat;
    osr.num_frequencies = 1;
    osr.signals[0] = SignalType::GPS_L2C;
    osr.pseudorange_rinex_codes[0] = "C2W";
    osr.carrier_rinex_codes[0] = "L2W";

    const Observation* selected = findOsrFrequencyObservation(obs, osr, 0);
    ASSERT_NE(selected, nullptr);
    EXPECT_EQ(selected->pseudorange_observation_type, "C2X");
    EXPECT_EQ(selected->carrier_phase_observation_type, "L2X");
    EXPECT_DOUBLE_EQ(selected->pseudorange, 100.0);
}

TEST(PPPOSRTest, BalancedPrefersFreshNetworkWhenNearestGridIsStale) {
    SSRProducts ssr_products;
    const GNSSTime epoch_time = makeTime(2018, 11, 24, 0, 0, 0.0);
    const SatelliteId sat_a(GNSSSystem::GPS, 1);
    const SatelliteId sat_b(GNSSSystem::GPS, 2);
    ssr_products.addCorrection(makeAtmosCorrection(sat_a, epoch_time - 90.0, 9));
    ssr_products.addCorrection(makeAtmosCorrection(sat_b, epoch_time - 1.0, 2));

    ppp_shared::PPPConfig config;
    config.clas_atmos_selection_policy =
        ppp_shared::PPPConfig::ClasAtmosSelectionPolicy::BALANCED;
    config.clas_atmos_stale_after_seconds = 15.0;

    const auto selected = selectClasEpochAtmosTokens(
        ssr_products,
        std::vector<SatelliteId>{sat_a, sat_b},
        epoch_time,
        receiverPositionNearClasNetwork9(),
        config);
    EXPECT_EQ(preferredClasNetworkId(selected), 2);
}

TEST(PPPOSRTest, BalancedKeepsNearestGridWhileCorrectionsAreFresh) {
    SSRProducts ssr_products;
    const GNSSTime epoch_time = makeTime(2018, 11, 24, 0, 0, 0.0);
    const SatelliteId sat_a(GNSSSystem::GPS, 1);
    const SatelliteId sat_b(GNSSSystem::GPS, 2);
    ssr_products.addCorrection(makeAtmosCorrection(sat_a, epoch_time - 5.0, 9));
    ssr_products.addCorrection(makeAtmosCorrection(sat_b, epoch_time - 1.0, 2));

    ppp_shared::PPPConfig config;
    config.clas_atmos_selection_policy =
        ppp_shared::PPPConfig::ClasAtmosSelectionPolicy::BALANCED;
    config.clas_atmos_stale_after_seconds = 15.0;

    const auto selected = selectClasEpochAtmosTokens(
        ssr_products,
        std::vector<SatelliteId>{sat_a, sat_b},
        epoch_time,
        receiverPositionNearClasNetwork9(),
        config);
    EXPECT_EQ(preferredClasNetworkId(selected), 9);
}

TEST(PPPOSRTest, GridGuardedRejectsStaleNearestGridTokens) {
    SSRProducts ssr_products;
    const GNSSTime epoch_time = makeTime(2018, 11, 24, 0, 0, 0.0);
    const SatelliteId sat_a(GNSSSystem::GPS, 1);
    const SatelliteId sat_b(GNSSSystem::GPS, 2);
    ssr_products.addCorrection(makeAtmosCorrection(sat_a, epoch_time - 90.0, 9));
    ssr_products.addCorrection(makeAtmosCorrection(sat_b, epoch_time - 1.0, 2));

    ppp_shared::PPPConfig config;
    config.clas_atmos_selection_policy =
        ppp_shared::PPPConfig::ClasAtmosSelectionPolicy::GRID_GUARDED;
    config.clas_atmos_stale_after_seconds = 15.0;

    const auto selected = selectClasEpochAtmosTokens(
        ssr_products,
        std::vector<SatelliteId>{sat_a, sat_b},
        epoch_time,
        receiverPositionNearClasNetwork9(),
        config);
    EXPECT_TRUE(selected.empty());
}

TEST(PPPOSRTest, FreshnessFirstAlwaysPrefersFreshestGridBackedTokens) {
    SSRProducts ssr_products;
    const GNSSTime epoch_time = makeTime(2018, 11, 24, 0, 0, 0.0);
    const SatelliteId sat_a(GNSSSystem::GPS, 1);
    const SatelliteId sat_b(GNSSSystem::GPS, 2);
    ssr_products.addCorrection(makeAtmosCorrection(sat_a, epoch_time - 90.0, 9));
    ssr_products.addCorrection(makeAtmosCorrection(sat_b, epoch_time - 1.0, 2));

    ppp_shared::PPPConfig config;
    config.clas_atmos_selection_policy =
        ppp_shared::PPPConfig::ClasAtmosSelectionPolicy::FRESHNESS_FIRST;
    config.clas_atmos_stale_after_seconds = 15.0;

    const auto selected = selectClasEpochAtmosTokens(
        ssr_products,
        std::vector<SatelliteId>{sat_a, sat_b},
        epoch_time,
        receiverPositionNearClasNetwork9(),
        config);
    EXPECT_EQ(preferredClasNetworkId(selected), 2);
}

TEST(PPPOSRTest, FullRepairContinuityPolicyKeepsBiasTermsAndRepairs) {
    using Policy = ppp_shared::PPPConfig::ClasPhaseContinuityPolicy;
    EXPECT_TRUE(usesClasPhaseBiasTerms(Policy::FULL_REPAIR));
    EXPECT_TRUE(usesClasSisContinuity(Policy::FULL_REPAIR));
    EXPECT_TRUE(usesClasPhaseBiasRepair(Policy::FULL_REPAIR));
    EXPECT_STREQ(clasPhaseContinuityPolicyName(Policy::FULL_REPAIR), "full-repair");
}

TEST(PPPOSRTest, FullPhaseBiasValuePolicyKeepsBothBiasAndCompensationTerms) {
    using Policy = ppp_shared::PPPConfig::ClasPhaseBiasValuePolicy;
    EXPECT_TRUE(usesClasRawPhaseBiasValues(Policy::FULL));
    EXPECT_TRUE(usesClasPhaseCompensationValues(Policy::FULL));
    EXPECT_STREQ(clasPhaseBiasValuePolicyName(Policy::FULL), "full");
}

TEST(PPPOSRTest, PhaseBiasOnlyValuePolicyDropsCompensationTerm) {
    using Policy = ppp_shared::PPPConfig::ClasPhaseBiasValuePolicy;
    EXPECT_TRUE(usesClasRawPhaseBiasValues(Policy::PHASE_BIAS_ONLY));
    EXPECT_FALSE(usesClasPhaseCompensationValues(Policy::PHASE_BIAS_ONLY));
    EXPECT_STREQ(clasPhaseBiasValuePolicyName(Policy::PHASE_BIAS_ONLY), "phase-bias-only");
}

TEST(PPPOSRTest, CompensationOnlyValuePolicyDropsRawBiasTerm) {
    using Policy = ppp_shared::PPPConfig::ClasPhaseBiasValuePolicy;
    EXPECT_FALSE(usesClasRawPhaseBiasValues(Policy::COMPENSATION_ONLY));
    EXPECT_TRUE(usesClasPhaseCompensationValues(Policy::COMPENSATION_ONLY));
    EXPECT_STREQ(
        clasPhaseBiasValuePolicyName(Policy::COMPENSATION_ONLY),
        "compensation-only");
}

TEST(PPPOSRTest, PhaseBiasReferenceTimePolicyDefaultsToCompactPhaseBiasEpoch) {
    using Policy = ppp_shared::PPPConfig::ClasPhaseBiasReferenceTimePolicy;
    const GNSSTime phase_bias_time = makeTime(2019, 8, 27, 0, 0, 30.0);
    const GNSSTime clock_time = makeTime(2019, 8, 27, 0, 1, 0.0);
    const GNSSTime obs_time = makeTime(2019, 8, 27, 0, 1, 1.0);
    EXPECT_EQ(
        selectClasPhaseBiasReferenceTime(
            Policy::PHASE_BIAS_REFERENCE,
            phase_bias_time,
            clock_time,
            obs_time),
        phase_bias_time);
    EXPECT_STREQ(
        clasPhaseBiasReferenceTimePolicyName(Policy::PHASE_BIAS_REFERENCE),
        "phase-bias-reference");
}

TEST(PPPOSRTest, PhaseBiasReferenceTimePolicyCanBindToClockEpoch) {
    using Policy = ppp_shared::PPPConfig::ClasPhaseBiasReferenceTimePolicy;
    const GNSSTime phase_bias_time = makeTime(2019, 8, 27, 0, 0, 30.0);
    const GNSSTime clock_time = makeTime(2019, 8, 27, 0, 1, 0.0);
    const GNSSTime obs_time = makeTime(2019, 8, 27, 0, 1, 1.0);
    EXPECT_EQ(
        selectClasPhaseBiasReferenceTime(
            Policy::CLOCK_REFERENCE,
            phase_bias_time,
            clock_time,
            obs_time),
        clock_time);
    EXPECT_STREQ(
        clasPhaseBiasReferenceTimePolicyName(Policy::CLOCK_REFERENCE),
        "clock-reference");
}

TEST(PPPOSRTest, PhaseBiasReferenceTimePolicyCanBindToObservationEpoch) {
    using Policy = ppp_shared::PPPConfig::ClasPhaseBiasReferenceTimePolicy;
    const GNSSTime phase_bias_time = makeTime(2019, 8, 27, 0, 0, 30.0);
    const GNSSTime clock_time = makeTime(2019, 8, 27, 0, 1, 0.0);
    const GNSSTime obs_time = makeTime(2019, 8, 27, 0, 1, 1.0);
    EXPECT_EQ(
        selectClasPhaseBiasReferenceTime(
            Policy::OBSERVATION_EPOCH,
            phase_bias_time,
            clock_time,
            obs_time),
        obs_time);
    EXPECT_STREQ(
        clasPhaseBiasReferenceTimePolicyName(Policy::OBSERVATION_EPOCH),
        "observation-epoch");
}

TEST(PPPOSRTest, SisContinuityOnlyPolicyKeepsBiasTermsAndSisDeltaWithoutRepair) {
    using Policy = ppp_shared::PPPConfig::ClasPhaseContinuityPolicy;
    EXPECT_TRUE(usesClasPhaseBiasTerms(Policy::SIS_CONTINUITY_ONLY));
    EXPECT_TRUE(usesClasSisContinuity(Policy::SIS_CONTINUITY_ONLY));
    EXPECT_FALSE(usesClasPhaseBiasRepair(Policy::SIS_CONTINUITY_ONLY));
    EXPECT_STREQ(
        clasPhaseContinuityPolicyName(Policy::SIS_CONTINUITY_ONLY),
        "sis-continuity-only");
}

TEST(PPPOSRTest, RepairOnlyPolicyKeepsBiasTermsAndRepairWithoutSisDelta) {
    using Policy = ppp_shared::PPPConfig::ClasPhaseContinuityPolicy;
    EXPECT_TRUE(usesClasPhaseBiasTerms(Policy::REPAIR_ONLY));
    EXPECT_FALSE(usesClasSisContinuity(Policy::REPAIR_ONLY));
    EXPECT_TRUE(usesClasPhaseBiasRepair(Policy::REPAIR_ONLY));
    EXPECT_STREQ(clasPhaseContinuityPolicyName(Policy::REPAIR_ONLY), "repair-only");
}

TEST(PPPOSRTest, RawPhaseBiasContinuityPolicyDisablesRepairButKeepsBiasTerms) {
    using Policy = ppp_shared::PPPConfig::ClasPhaseContinuityPolicy;
    EXPECT_TRUE(usesClasPhaseBiasTerms(Policy::RAW_PHASE_BIAS));
    EXPECT_FALSE(usesClasSisContinuity(Policy::RAW_PHASE_BIAS));
    EXPECT_FALSE(usesClasPhaseBiasRepair(Policy::RAW_PHASE_BIAS));
    EXPECT_STREQ(clasPhaseContinuityPolicyName(Policy::RAW_PHASE_BIAS), "raw-phase-bias");
}

TEST(PPPOSRTest, NoPhaseBiasContinuityPolicyDropsBiasTermsEntirely) {
    using Policy = ppp_shared::PPPConfig::ClasPhaseContinuityPolicy;
    EXPECT_FALSE(usesClasPhaseBiasTerms(Policy::NO_PHASE_BIAS));
    EXPECT_FALSE(usesClasSisContinuity(Policy::NO_PHASE_BIAS));
    EXPECT_FALSE(usesClasPhaseBiasRepair(Policy::NO_PHASE_BIAS));
    EXPECT_STREQ(clasPhaseContinuityPolicyName(Policy::NO_PHASE_BIAS), "no-phase-bias");
}

TEST(PPPOSRTest, LagTolerantSsrTimingPolicyKeepsExistingCarryBehavior) {
    using Policy = ppp_shared::PPPConfig::ClasSsrTimingPolicy;
    EXPECT_FALSE(usesClasClockBoundPhaseBias(Policy::LAG_TOLERANT));
    EXPECT_FALSE(usesClasClockBoundAtmos(Policy::LAG_TOLERANT));
    EXPECT_STREQ(clasSsrTimingPolicyName(Policy::LAG_TOLERANT), "lag-tolerant");
}

TEST(PPPOSRTest, ClockBoundPhaseBiasPolicyOnlyGuardsPhaseBiasTiming) {
    using Policy = ppp_shared::PPPConfig::ClasSsrTimingPolicy;
    EXPECT_TRUE(usesClasClockBoundPhaseBias(Policy::CLOCK_BOUND_PHASE_BIAS));
    EXPECT_FALSE(usesClasClockBoundAtmos(Policy::CLOCK_BOUND_PHASE_BIAS));
    EXPECT_STREQ(
        clasSsrTimingPolicyName(Policy::CLOCK_BOUND_PHASE_BIAS),
        "clock-bound-phase-bias");
}

TEST(PPPOSRTest, ClockBoundAtmosAndPhaseBiasPolicyGuardsBothTimingLayers) {
    using Policy = ppp_shared::PPPConfig::ClasSsrTimingPolicy;
    EXPECT_TRUE(usesClasClockBoundPhaseBias(Policy::CLOCK_BOUND_ATMOS_AND_PHASE_BIAS));
    EXPECT_TRUE(usesClasClockBoundAtmos(Policy::CLOCK_BOUND_ATMOS_AND_PHASE_BIAS));
    EXPECT_STREQ(
        clasSsrTimingPolicyName(Policy::CLOCK_BOUND_ATMOS_AND_PHASE_BIAS),
        "clock-bound-atmos-and-phase-bias");
}
