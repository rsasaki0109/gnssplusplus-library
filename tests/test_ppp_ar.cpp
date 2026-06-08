#include <gtest/gtest.h>

#include <libgnss++/algorithms/ppp_ar.hpp>

using namespace libgnss;

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
        config, state, ambiguity_states, false, false);

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
        config, state, ambiguity_states, true, false);

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
