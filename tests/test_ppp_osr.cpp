#include <gtest/gtest.h>

#include <libgnss++/algorithms/ppp_clas.hpp>
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

SSROrbitClockCorrection makePhaseBiasCorrection(
    const SatelliteId& satellite,
    const GNSSTime& time,
    int network_id,
    double l1_phase_bias_m) {
    SSROrbitClockCorrection correction;
    correction.satellite = satellite;
    correction.time = time;
    correction.phase_bias_m[2] = l1_phase_bias_m;
    correction.phase_bias_valid = true;
    correction.bias_network_id = network_id;
    return correction;
}

Vector3d receiverPositionNearClasNetwork9() {
    return geodetic2ecef(41.30 * M_PI / 180.0, 140.70 * M_PI / 180.0, 0.0);
}

}  // namespace

TEST(PPPOSRTest, GridFirstPrefersNearestGridEvenWhenStale) {
    SSRProducts ssr_products;
    const GNSSTime epoch_time = makeTime(2018, 11, 24, 0, 0, 0.0);
    const SatelliteId sat_a(GNSSSystem::GPS, 1);
    const SatelliteId sat_b(GNSSSystem::GPS, 2);
    ssr_products.addCorrection(makeAtmosCorrection(sat_a, epoch_time - 90.0, 9));
    ssr_products.addCorrection(makeAtmosCorrection(sat_b, epoch_time - 1.0, 10));

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

TEST(PPPOSRTest, ExactTimeBiasSelectionHonorsPreferredNetworkAcrossRecentGroups) {
    SSRProducts ssr_products;
    const GNSSTime epoch_time = makeTime(2019, 8, 27, 0, 0, 25.0);
    const SatelliteId satellite(GNSSSystem::GPS, 14);
    ssr_products.addCorrection(
        makePhaseBiasCorrection(satellite, epoch_time - 25.0, 7, -1.165));
    ssr_products.addCorrection(
        makePhaseBiasCorrection(satellite, epoch_time, 1, 5.390));

    Vector3d orbit = Vector3d::Zero();
    double clock_m = 0.0;
    std::map<uint8_t, double> phase_bias;
    GNSSTime phase_bias_time;
    int selected_phase_network = 0;

    EXPECT_TRUE(ssr_products.interpolateCorrection(
        satellite,
        epoch_time,
        orbit,
        clock_m,
        nullptr,
        nullptr,
        &phase_bias,
        nullptr,
        nullptr,
        &phase_bias_time,
        nullptr,
        7,
        true,
        false,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        &selected_phase_network));
    EXPECT_EQ(selected_phase_network, 7);
    ASSERT_NE(phase_bias.find(2), phase_bias.end());
    EXPECT_DOUBLE_EQ(phase_bias.at(2), -1.165);
    EXPECT_DOUBLE_EQ(phase_bias_time.tow, (epoch_time - 25.0).tow);

    phase_bias.clear();
    phase_bias_time = GNSSTime();
    selected_phase_network = 0;
    EXPECT_TRUE(ssr_products.interpolateCorrection(
        satellite,
        epoch_time,
        orbit,
        clock_m,
        nullptr,
        nullptr,
        &phase_bias,
        nullptr,
        nullptr,
        &phase_bias_time,
        nullptr,
        0,
        true,
        false,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        &selected_phase_network));
    EXPECT_EQ(selected_phase_network, 1);
    ASSERT_NE(phase_bias.find(2), phase_bias.end());
    EXPECT_DOUBLE_EQ(phase_bias.at(2), 5.390);
    EXPECT_DOUBLE_EQ(phase_bias_time.tow, epoch_time.tow);
}

TEST(PPPOSRTest, FinalizeClasEpochSolutionCarriesTimeAndGeodeticPosition) {
    ppp_shared::PPPState state;
    state.state = VectorXd::Zero(9);
    state.covariance = MatrixXd::Identity(9, 9);

    constexpr double kLatitudeRad = 35.0 * M_PI / 180.0;
    constexpr double kLongitudeRad = 139.0 * M_PI / 180.0;
    constexpr double kHeightM = 42.0;
    state.state.segment(0, 3) = geodetic2ecef(kLatitudeRad, kLongitudeRad, kHeightM);
    state.state(state.clock_index) = 12.0;
    state.state(state.glo_clock_index) = -3.0;

    const GNSSTime epoch_time(2324, 187470.2);
    const PositionSolution solution = ppp_clas::finalizeEpochSolution(
        state,
        epoch_time,
        false,
        0.0,
        0,
        7);

    EXPECT_EQ(solution.time.week, epoch_time.week);
    EXPECT_DOUBLE_EQ(solution.time.tow, epoch_time.tow);
    EXPECT_NEAR(solution.position_geodetic.latitude, kLatitudeRad, 1e-10);
    EXPECT_NEAR(solution.position_geodetic.longitude, kLongitudeRad, 1e-10);
    EXPECT_NEAR(solution.position_geodetic.height, kHeightM, 1e-5);
    EXPECT_EQ(solution.num_satellites, 7);
}

TEST(PPPOSRTest, BalancedPrefersFreshNetworkWhenNearestGridIsStale) {
    SSRProducts ssr_products;
    const GNSSTime epoch_time = makeTime(2018, 11, 24, 0, 0, 0.0);
    const SatelliteId sat_a(GNSSSystem::GPS, 1);
    const SatelliteId sat_b(GNSSSystem::GPS, 2);
    ssr_products.addCorrection(makeAtmosCorrection(sat_a, epoch_time - 90.0, 9));
    ssr_products.addCorrection(makeAtmosCorrection(sat_b, epoch_time - 1.0, 10));

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
    EXPECT_EQ(preferredClasNetworkId(selected), 10);
}

TEST(PPPOSRTest, BalancedKeepsNearestGridWhileCorrectionsAreFresh) {
    SSRProducts ssr_products;
    const GNSSTime epoch_time = makeTime(2018, 11, 24, 0, 0, 0.0);
    const SatelliteId sat_a(GNSSSystem::GPS, 1);
    const SatelliteId sat_b(GNSSSystem::GPS, 2);
    ssr_products.addCorrection(makeAtmosCorrection(sat_a, epoch_time - 5.0, 9));
    ssr_products.addCorrection(makeAtmosCorrection(sat_b, epoch_time - 1.0, 10));

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
    ssr_products.addCorrection(makeAtmosCorrection(sat_b, epoch_time - 1.0, 10));

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
    ssr_products.addCorrection(makeAtmosCorrection(sat_b, epoch_time - 1.0, 10));

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
    EXPECT_EQ(preferredClasNetworkId(selected), 10);
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
