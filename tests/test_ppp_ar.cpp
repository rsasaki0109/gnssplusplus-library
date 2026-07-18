#include <gtest/gtest.h>

#include <libgnss++/algorithms/ppp_ar.hpp>

using namespace libgnss;

TEST(PPPArTest, MrtklibParCandidatesRequireTwoActiveFrequencyStates) {
    const SatelliteId gps6(GNSSSystem::GPS, 6);
    const SatelliteId gps6_l2(GNSSSystem::GPS, 106);
    const SatelliteId gps11(GNSSSystem::GPS, 11);
    const SatelliteId gps11_l2(GNSSSystem::GPS, 111);
    const SatelliteId qzss2(GNSSSystem::QZSS, 2);
    const SatelliteId gps17(GNSSSystem::GPS, 17);
    const SatelliteId gps17_l2(GNSSSystem::GPS, 117);

    const std::vector<SatelliteId> eligible = {
        gps6, gps6_l2, gps11, gps11_l2, qzss2, gps17, gps17_l2};
    const std::map<SatelliteId, double> elevations = {
        {gps6, 54.0 * M_PI / 180.0},
        {gps11, 20.3 * M_PI / 180.0},
        {qzss2, 33.0 * M_PI / 180.0},
        {gps17, 19.9 * M_PI / 180.0},
    };

    const auto candidates =
        ppp_ar::selectMrtklibParCandidates(eligible, elevations);

    ASSERT_EQ(candidates.size(), 2u);
    EXPECT_EQ(candidates[0], gps11);
    EXPECT_EQ(candidates[1], gps6);
}

TEST(PPPArTest, ParCandidatesCanIncludeSingleFrequencyConstellationFallback) {
    const SatelliteId gps6(GNSSSystem::GPS, 6);
    const SatelliteId gps6_l2(GNSSSystem::GPS, 106);
    const SatelliteId qzss2(GNSSSystem::QZSS, 2);

    const std::vector<SatelliteId> eligible = {gps6, gps6_l2, qzss2};
    const std::map<SatelliteId, double> elevations = {
        {gps6, 54.0 * M_PI / 180.0},
        {qzss2, 33.0 * M_PI / 180.0},
    };

    const auto candidates =
        ppp_ar::selectMrtklibParCandidates(eligible, elevations, 1);

    ASSERT_EQ(candidates.size(), 2u);
    EXPECT_EQ(candidates[0], qzss2);
    EXPECT_EQ(candidates[1], gps6);
}

TEST(PPPArTest, FrequencyLifecycleTracksSignalsIndependently) {
    ppp_shared::PPPAmbiguityInfo ambiguity;
    const GNSSTime first_time(2360, 100.0);
    const GNSSTime second_time(2360, 130.0);

    ppp_shared::updateFrequencyAmbiguityLifecycle(
        ambiguity, SignalType::GPS_L1CA, 123.5, first_time, 42.0);
    ppp_shared::updateFrequencyAmbiguityLifecycle(
        ambiguity, SignalType::GPS_L5, 456.25, first_time, 39.0);
    ppp_shared::updateFrequencyAmbiguityLifecycle(
        ambiguity, SignalType::GPS_L1CA, 124.0, second_time, 43.0);

    ASSERT_EQ(ambiguity.frequency_lifecycle.size(), 2u);
    const auto& l1 = ambiguity.frequency_lifecycle.at(SignalType::GPS_L1CA);
    EXPECT_EQ(l1.lock_count, 2);
    EXPECT_DOUBLE_EQ(l1.last_phase, 124.0);
    EXPECT_EQ(l1.last_time.week, second_time.week);
    EXPECT_DOUBLE_EQ(l1.last_time.tow, second_time.tow);
    EXPECT_DOUBLE_EQ(l1.quality_indicator, 43.0);
    EXPECT_TRUE(l1.has_last_phase);

    const auto& l5 = ambiguity.frequency_lifecycle.at(SignalType::GPS_L5);
    EXPECT_EQ(l5.lock_count, 1);
    EXPECT_DOUBLE_EQ(l5.last_phase, 456.25);
    EXPECT_DOUBLE_EQ(l5.quality_indicator, 39.0);
    EXPECT_TRUE(l5.has_last_phase);

    // The per-signal helper must not mutate the legacy satellite-level view.
    EXPECT_EQ(ambiguity.lock_count, 0);
    EXPECT_DOUBLE_EQ(ambiguity.last_phase, 0.0);
}

TEST(PPPArTest, WlnlPreparationTracksEligibilitySkipReasons) {
    ppp_shared::PPPConfig config;
    config.convergence_min_epochs = 3;

    ppp_shared::PPPState state;
    state.amb_index = 4;
    state.total_states = 8;
    state.state = VectorXd::Zero(8);
    state.covariance = MatrixXd::Identity(8, 8);

    const SatelliteId sat1(GNSSSystem::GPS, 1);
    const SatelliteId sat2(GNSSSystem::GPS, 2);
    const SatelliteId sat3(GNSSSystem::GPS, 3);
    const SatelliteId sat4(GNSSSystem::GPS, 4);
    const SatelliteId sat5(GNSSSystem::GPS, 5);

    state.ambiguity_indices[sat1] = 4;
    state.ambiguity_indices[sat2] = 5;
    state.ambiguity_indices[sat3] = 6;
    state.ambiguity_indices[sat4] = 7;
    state.ambiguity_indices[sat5] = 3;

    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo> ambiguity_states;

    ppp_shared::PPPAmbiguityInfo reinit;
    reinit.needs_reinitialization = true;
    reinit.lock_count = 10;
    reinit.ambiguity_scale_m = 0.19;
    ambiguity_states[sat1] = reinit;

    ppp_shared::PPPAmbiguityInfo low_lock;
    low_lock.needs_reinitialization = false;
    low_lock.lock_count = 2;
    low_lock.ambiguity_scale_m = 0.19;
    ambiguity_states[sat2] = low_lock;

    ppp_shared::PPPAmbiguityInfo bad_scale;
    bad_scale.needs_reinitialization = false;
    bad_scale.lock_count = 5;
    bad_scale.ambiguity_scale_m = 0.0;
    ambiguity_states[sat3] = bad_scale;

    ppp_shared::PPPAmbiguityInfo eligible;
    eligible.needs_reinitialization = false;
    eligible.lock_count = 5;
    eligible.ambiguity_scale_m = 0.24;
    ambiguity_states[sat4] = eligible;

    ppp_shared::PPPAmbiguityInfo bad_index;
    bad_index.needs_reinitialization = false;
    bad_index.lock_count = 5;
    bad_index.ambiguity_scale_m = 0.19;
    ambiguity_states[sat5] = bad_index;

    const auto preparation = ppp_ar::prepareWlnlCandidates(
        config, state, ambiguity_states, false, GNSSTime{}, false);

    EXPECT_EQ(preparation.min_lock_count, 3);
    EXPECT_EQ(preparation.eligible_ambiguities.total_ambiguities, 5);
    EXPECT_EQ(preparation.eligible_ambiguities.skipped_reinitialization, 1);
    EXPECT_EQ(preparation.eligible_ambiguities.skipped_lock, 1);
    EXPECT_EQ(preparation.eligible_ambiguities.skipped_scale, 1);
    EXPECT_EQ(preparation.eligible_ambiguities.skipped_index, 1);
    ASSERT_EQ(preparation.eligible_ambiguities.satellites.size(), 1u);
    EXPECT_EQ(preparation.eligible_ambiguities.satellites.front(), sat4);
    ASSERT_EQ(preparation.eligible_ambiguities.state_indices.size(), 1u);
    EXPECT_EQ(preparation.eligible_ambiguities.state_indices.front(), 7);
}

TEST(PPPArTest, WlnlPreparationAppliesWideLaneFixesAndSummarizesCounts) {
    ppp_shared::PPPConfig config;
    config.convergence_min_epochs = 3;
    config.wl_min_averaging_epochs = 4;

    ppp_shared::PPPState state;
    state.amb_index = 4;
    state.total_states = 8;
    state.state = VectorXd::Zero(8);
    state.covariance = MatrixXd::Identity(8, 8);

    const SatelliteId sat1(GNSSSystem::GPS, 1);
    const SatelliteId sat2(GNSSSystem::GPS, 2);
    const SatelliteId sat3(GNSSSystem::GPS, 3);

    state.ambiguity_indices[sat1] = 4;
    state.ambiguity_indices[sat2] = 5;
    state.ambiguity_indices[sat3] = 6;

    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo> ambiguity_states;

    ppp_shared::PPPAmbiguityInfo already_fixed;
    already_fixed.needs_reinitialization = false;
    already_fixed.lock_count = 10;
    already_fixed.ambiguity_scale_m = 0.19;
    already_fixed.wl_is_fixed = true;
    already_fixed.mw_count = 6;
    ambiguity_states[sat1] = already_fixed;

    ppp_shared::PPPAmbiguityInfo fixable;
    fixable.needs_reinitialization = false;
    fixable.lock_count = 10;
    fixable.ambiguity_scale_m = 0.19;
    fixable.mw_count = 5;
    fixable.mw_mean_cycles = 12.08;
    ambiguity_states[sat2] = fixable;

    ppp_shared::PPPAmbiguityInfo rejected;
    rejected.needs_reinitialization = false;
    rejected.lock_count = 10;
    rejected.ambiguity_scale_m = 0.19;
    rejected.mw_count = 7;
    rejected.mw_mean_cycles = 4.41;
    ambiguity_states[sat3] = rejected;

    const auto preparation = ppp_ar::prepareWlnlCandidates(
        config, state, ambiguity_states, true, GNSSTime{}, false);

    EXPECT_EQ(preparation.min_lock_count, 4);
    EXPECT_EQ(preparation.wl_summary.fixed_count, 2);
    EXPECT_EQ(preparation.wl_summary.max_mw_count, 7);
    EXPECT_TRUE(ambiguity_states.at(sat1).wl_is_fixed);
    EXPECT_TRUE(ambiguity_states.at(sat2).wl_is_fixed);
    EXPECT_EQ(ambiguity_states.at(sat2).wl_fixed_integer, 12);
    EXPECT_FALSE(ambiguity_states.at(sat3).wl_is_fixed);
}

TEST(PPPArTest, BuildWlnlNlInfoMapUsesOnlyWideLaneFixedSatellites) {
    const SatelliteId sat1(GNSSSystem::GPS, 1);
    const SatelliteId sat2(GNSSSystem::GPS, 2);
    const SatelliteId sat3(GNSSSystem::GPS, 3);

    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo> ambiguity_states;
    ambiguity_states[sat1].wl_is_fixed = true;
    ambiguity_states[sat2].wl_is_fixed = false;
    ambiguity_states[sat3].wl_is_fixed = true;

    int provider_calls = 0;
    const auto info_map = ppp_ar::buildWlnlNlInfoMap(
        {sat1, sat2, sat3},
        ambiguity_states,
        [&](const SatelliteId& satellite, ppp_ar::WlnlNlInfo& info) {
            ++provider_calls;
            if (satellite == sat3) {
                return false;
            }
            info.valid = true;
            info.nl_ambiguity_cycles = 3.25;
            info.lambda_nl_m = 0.11;
            return true;
        });

    EXPECT_EQ(provider_calls, 2);
    ASSERT_EQ(info_map.size(), 1u);
    EXPECT_NE(info_map.find(sat1), info_map.end());
    EXPECT_EQ(info_map.find(sat2), info_map.end());
    EXPECT_EQ(info_map.find(sat3), info_map.end());
    EXPECT_DOUBLE_EQ(info_map.at(sat1).nl_ambiguity_cycles, 3.25);
}

TEST(PPPArTest, ResolveWlnlFixUsesOnlyWideLaneFixedEligibleSatellites) {
    ppp_shared::PPPConfig config;

    ppp_shared::PPPState state;
    state.amb_index = 4;
    state.total_states = 9;
    state.state = VectorXd::Zero(9);
    state.covariance = MatrixXd::Identity(9, 9);

    const SatelliteId sat1(GNSSSystem::GPS, 1);
    const SatelliteId sat2(GNSSSystem::GPS, 2);
    const SatelliteId sat3(GNSSSystem::GPS, 3);
    const SatelliteId sat4(GNSSSystem::GPS, 4);
    const SatelliteId sat5(GNSSSystem::GPS, 5);

    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo> ambiguity_states;
    for (const auto& satellite : {sat1, sat2, sat3, sat4}) {
        auto& ambiguity = ambiguity_states[satellite];
        ambiguity.wl_is_fixed = true;
        ambiguity.lock_count = 10;
    }
    ambiguity_states[sat5].wl_is_fixed = false;
    ambiguity_states[sat5].lock_count = 10;

    ppp_ar::EligibleAmbiguities eligible;
    eligible.satellites = {sat1, sat2, sat3, sat4, sat5};
    eligible.state_indices = {4, 5, 6, 7, 8};

    int provider_calls = 0;
    const auto attempt = ppp_ar::resolveWlnlFix(
        config,
        state,
        ambiguity_states,
        eligible,
        [&](const SatelliteId& satellite, ppp_ar::WlnlNlInfo& info) {
            ++provider_calls;
            info.valid = true;
            info.nl_ambiguity_cycles = static_cast<double>(satellite.prn);
            info.lambda_nl_m = 0.14;
            info.lambda_wl_m = 0.86;
            info.beta = 0.11;
            info.group = {GNSSSystem::GPS, {1, 0}};
            return true;
        },
        false);

    EXPECT_EQ(provider_calls, 4);
    EXPECT_FALSE(attempt.fixed);
    EXPECT_EQ(attempt.nb, 3);
}

TEST(PPPArTest, DirectStateDdHoldUsesOnlyCurrentAcceptedRows) {
    ppp_shared::PPPConfig config;
    config.clas_mrtklib_float_parity = true;
    config.use_clas_osr_filter = true;
    config.kinematic_mode = true;
    config.use_dynamics_model = true;
    config.low_dynamics_mode = false;

    ppp_shared::PPPState state;
    state.pos_index = 0;
    state.amb_index = 3;
    state.total_states = 11;
    state.state = VectorXd::Zero(state.total_states);
    state.covariance = MatrixXd::Identity(state.total_states, state.total_states) * 1e-6;

    const std::vector<SatelliteId> satellites = {
        {GNSSSystem::GPS, 1}, {GNSSSystem::GPS, 2},
        {GNSSSystem::GPS, 3}, {GNSSSystem::GPS, 4},
        {GNSSSystem::GPS, 101}, {GNSSSystem::GPS, 102},
        {GNSSSystem::GPS, 103}, {GNSSSystem::GPS, 104},
    };
    ppp_ar::EligibleAmbiguities eligible;
    for (int i = 0; i < static_cast<int>(satellites.size()); ++i) {
        const int state_index = state.amb_index + i;
        eligible.satellites.push_back(satellites[static_cast<size_t>(i)]);
        eligible.state_indices.push_back(state_index);
        eligible.scales.push_back(0.19);
        state.ambiguity_indices[satellites[static_cast<size_t>(i)]] = state_index;
        state.state(state_index) =
            (i < 4 ? static_cast<double>(i) : 10.0 + static_cast<double>(i - 4)) * 0.19;
    }

    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo> ambiguity_states;
    for (const auto& satellite : satellites) {
        ambiguity_states[satellite].ambiguity_scale_m = 0.19;
    }
    const SatelliteId stale_satellite(GNSSSystem::GPS, 5);
    ambiguity_states[stale_satellite].is_fixed = true;
    ambiguity_states[stale_satellite].wl_is_fixed = true;
    ambiguity_states[stale_satellite].nl_is_fixed = true;

    std::map<SatelliteId, double> elevations;
    for (int prn = 1; prn <= 4; ++prn) {
        elevations[{GNSSSystem::GPS, static_cast<uint8_t>(prn)}] =
            (35.0 + prn) * M_PI / 180.0;
    }

    const auto attempt = ppp_ar::resolveWlnlFix(
        config, state, state.covariance, ambiguity_states, eligible,
        ppp_ar::WlnlNlInfoProvider{}, false, &elevations);

    ASSERT_TRUE(attempt.fixed);
    EXPECT_EQ(attempt.nb, 6);
    ASSERT_EQ(attempt.hold_constraints.size(), 6u);
    for (const auto& constraint : attempt.hold_constraints) {
        EXPECT_NE(ppp_ar::clasRealSatellite(constraint.ref_satellite), stale_satellite);
        EXPECT_NE(ppp_ar::clasRealSatellite(constraint.sat_satellite), stale_satellite);
    }
}

TEST(PPPArTest, BuildFixedObservationHelpersFilterInvalidProviders) {
    const SatelliteId sat1(GNSSSystem::GPS, 1);
    const SatelliteId sat2(GNSSSystem::GPS, 2);

    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo> ambiguity_states;
    ambiguity_states[sat1].is_fixed = true;
    ambiguity_states[sat1].wl_is_fixed = true;
    ambiguity_states[sat1].nl_is_fixed = true;
    ambiguity_states[sat1].nl_fixed_cycles = 10.0;
    ambiguity_states[sat2].is_fixed = true;
    ambiguity_states[sat2].wl_is_fixed = true;
    ambiguity_states[sat2].nl_is_fixed = true;
    ambiguity_states[sat2].nl_fixed_cycles = 20.0;

    const auto nl_observations = ppp_ar::buildFixedNlObservations(
        ambiguity_states,
        [&](const SatelliteId& satellite,
            const ppp_shared::PPPAmbiguityInfo& ambiguity,
            ppp_ar::FixedNlObservation& observation) {
            if (satellite == sat2) {
                return false;
            }
            observation.fixed_nl_cycles = ambiguity.nl_fixed_cycles;
            observation.lambda_nl_m = 0.12;
            observation.nl_phase_m = 1.5;
            return true;
        });

    ASSERT_EQ(nl_observations.size(), 1u);
    EXPECT_DOUBLE_EQ(nl_observations.front().fixed_nl_cycles, 10.0);

    const auto carrier_observations = ppp_ar::buildFixedCarrierObservations(
        3,
        [&](size_t index, ppp_ar::FixedCarrierObservation& observation) {
            if (index == 1) {
                return false;
            }
            observation.carrier_phase_if = static_cast<double>(index) + 1.0;
            return true;
        });

    ASSERT_EQ(carrier_observations.size(), 2u);
    EXPECT_DOUBLE_EQ(carrier_observations[0].carrier_phase_if, 1.0);
    EXPECT_DOUBLE_EQ(carrier_observations[1].carrier_phase_if, 3.0);
}
