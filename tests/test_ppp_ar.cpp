#include <gtest/gtest.h>

#include <libgnss++/algorithms/ppp_ar.hpp>

#include <cmath>
#include <limits>

using namespace libgnss;

namespace {

ppp_shared::PPPState makeDirectDdState(double dd_float_cycles) {
    ppp_shared::PPPState state;
    state.pos_index = 0;
    state.clock_index = 3;
    state.amb_index = 4;
    state.total_states = 6;
    state.state = VectorXd::Zero(state.total_states);
    state.covariance = MatrixXd::Identity(state.total_states, state.total_states);

    const SatelliteId ref_sat(GNSSSystem::GPS, 1);
    const SatelliteId sat(GNSSSystem::GPS, 2);
    state.ambiguity_indices[ref_sat] = 4;
    state.ambiguity_indices[sat] = 5;
    state.state(4) = 0.0;
    state.state(5) = -dd_float_cycles;
    return state;
}

std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo> makeDirectDdAmbiguities() {
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo> ambiguity_states;
    for (const auto& satellite : {SatelliteId(GNSSSystem::GPS, 1),
                                  SatelliteId(GNSSSystem::GPS, 2)}) {
        auto& ambiguity = ambiguity_states[satellite];
        ambiguity.needs_reinitialization = false;
        ambiguity.lock_count = 10;
        ambiguity.ambiguity_scale_m = 1.0;
    }
    return ambiguity_states;
}

struct MadocaDesignFixture {
    SatelliteId satellite{GNSSSystem::GPS, 2};
    SatelliteId reference{GNSSSystem::GPS, 1};
    std::map<std::pair<SatelliteId, int>, int> indices;
    std::map<std::pair<SatelliteId, int>, double> wavelengths;
    VectorXd state = VectorXd::Zero(8);
    MatrixXd covariance = MatrixXd::Zero(8, 8);

    MadocaDesignFixture() {
        const double lambda0 = 0.19;
        const double lambda1 = 0.24;
        const double lambda2 = 0.25;
        const double lambda3 = 0.21;
        const double lambda_by_frequency[] = {lambda0, lambda1, lambda2, lambda3};
        for (int frequency = 0; frequency < 4; ++frequency) {
            indices[{satellite, frequency}] = frequency;
            indices[{reference, frequency}] = frequency + 4;
            wavelengths[{satellite, frequency}] = lambda_by_frequency[frequency];
            wavelengths[{reference, frequency}] = lambda_by_frequency[frequency];
        }

        state(0) = 3.0 * lambda0;
        state(1) = 1.0 * lambda1;
        state(2) = 5.0 * lambda2;
        state(3) = 11.0 * lambda3;
        state(4) = 7.0 * lambda0;
        state(5) = 2.0 * lambda1;
        state(6) = 13.0 * lambda2;
        state(7) = 17.0 * lambda3;

        covariance(0, 0) = 4.0;
        covariance(4, 4) = 9.0;
        covariance(0, 4) = 1.5;
        covariance(4, 0) = 1.5;
    }

    int stateIndex(const SatelliteId& row_satellite, int frequency_index) const {
        const auto it = indices.find({row_satellite, frequency_index});
        return it == indices.end() ? -1 : it->second;
    }

    double wavelength(const SatelliteId& row_satellite, int frequency_index) const {
        const auto it = wavelengths.find({row_satellite, frequency_index});
        return it == wavelengths.end() ? 0.0 : it->second;
    }
};

}  // namespace

TEST(PPPArTest, MadocaN1DesignRowMatchesSingleFrequencyDifference) {
    const MadocaDesignFixture fixture;
    const auto row = ppp_ar::buildMadocaArDesignRow(
        fixture.satellite,
        fixture.reference,
        0,
        -1,
        [&](const SatelliteId& satellite, int frequency_index) {
            return fixture.stateIndex(satellite, frequency_index);
        },
        [&](const SatelliteId& satellite, int frequency_index) {
            return fixture.wavelength(satellite, frequency_index);
        });

    ASSERT_TRUE(row.valid);
    ASSERT_EQ(row.terms.size(), 2u);
    EXPECT_EQ(row.terms[0].satellite, fixture.satellite);
    EXPECT_EQ(row.terms[0].frequency_index, 0);
    EXPECT_EQ(row.terms[0].state_index, 0);
    EXPECT_NEAR(row.terms[0].coefficient, 1.0 / 0.19, 1e-12);
    EXPECT_EQ(row.terms[1].satellite, fixture.reference);
    EXPECT_EQ(row.terms[1].frequency_index, 0);
    EXPECT_EQ(row.terms[1].state_index, 4);
    EXPECT_NEAR(row.terms[1].coefficient, -1.0 / 0.19, 1e-12);
    EXPECT_NEAR(ppp_ar::evaluateMadocaArDesignRow(row, fixture.state), -4.0, 1e-12);
    EXPECT_NEAR(
        ppp_ar::madocaArDesignRowVariance(row, fixture.covariance),
        10.0 / (0.19 * 0.19),
        1e-12);
}

TEST(PPPArTest, MadocaWideLaneDesignRowMatchesF1MinusF2Difference) {
    const MadocaDesignFixture fixture;
    const auto row = ppp_ar::buildMadocaArDesignRow(
        fixture.satellite,
        fixture.reference,
        0,
        1,
        [&](const SatelliteId& satellite, int frequency_index) {
            return fixture.stateIndex(satellite, frequency_index);
        },
        [&](const SatelliteId& satellite, int frequency_index) {
            return fixture.wavelength(satellite, frequency_index);
        });

    ASSERT_TRUE(row.valid);
    ASSERT_EQ(row.terms.size(), 4u);
    EXPECT_EQ(row.terms[0].state_index, 0);
    EXPECT_NEAR(row.terms[0].coefficient, 1.0 / 0.19, 1e-12);
    EXPECT_EQ(row.terms[1].state_index, 1);
    EXPECT_NEAR(row.terms[1].coefficient, -1.0 / 0.24, 1e-12);
    EXPECT_EQ(row.terms[2].state_index, 4);
    EXPECT_NEAR(row.terms[2].coefficient, -1.0 / 0.19, 1e-12);
    EXPECT_EQ(row.terms[3].state_index, 5);
    EXPECT_NEAR(row.terms[3].coefficient, 1.0 / 0.24, 1e-12);
    EXPECT_NEAR(ppp_ar::evaluateMadocaArDesignRow(row, fixture.state), -3.0, 1e-12);
}

TEST(PPPArTest, MadocaExtraWideLaneDesignRowMatchesF2MinusF3Difference) {
    const MadocaDesignFixture fixture;
    const auto row = ppp_ar::buildMadocaArDesignRow(
        fixture.satellite,
        fixture.reference,
        1,
        2,
        [&](const SatelliteId& satellite, int frequency_index) {
            return fixture.stateIndex(satellite, frequency_index);
        },
        [&](const SatelliteId& satellite, int frequency_index) {
            return fixture.wavelength(satellite, frequency_index);
        });

    ASSERT_TRUE(row.valid);
    ASSERT_EQ(row.terms.size(), 4u);
    EXPECT_EQ(row.terms[0].state_index, 1);
    EXPECT_NEAR(row.terms[0].coefficient, 1.0 / 0.24, 1e-12);
    EXPECT_EQ(row.terms[1].state_index, 2);
    EXPECT_NEAR(row.terms[1].coefficient, -1.0 / 0.25, 1e-12);
    EXPECT_EQ(row.terms[2].state_index, 5);
    EXPECT_NEAR(row.terms[2].coefficient, -1.0 / 0.24, 1e-12);
    EXPECT_EQ(row.terms[3].state_index, 6);
    EXPECT_NEAR(row.terms[3].coefficient, 1.0 / 0.25, 1e-12);
    EXPECT_NEAR(ppp_ar::evaluateMadocaArDesignRow(row, fixture.state), 7.0, 1e-12);
}

TEST(PPPArTest, MadocaDesignRowRejectsMissingIndexOrWavelength) {
    const MadocaDesignFixture fixture;
    const auto missing_index = ppp_ar::buildMadocaArDesignRow(
        fixture.satellite,
        fixture.reference,
        0,
        4,
        [&](const SatelliteId& satellite, int frequency_index) {
            return fixture.stateIndex(satellite, frequency_index);
        },
        [&](const SatelliteId& satellite, int frequency_index) {
            return fixture.wavelength(satellite, frequency_index);
        });
    EXPECT_FALSE(missing_index.valid);
    EXPECT_TRUE(missing_index.terms.empty());

    const auto missing_wavelength = ppp_ar::buildMadocaArDesignRow(
        fixture.satellite,
        fixture.reference,
        0,
        -1,
        [&](const SatelliteId& satellite, int frequency_index) {
            return fixture.stateIndex(satellite, frequency_index);
        },
        [&](const SatelliteId&, int frequency_index) {
            return frequency_index == 0 ? 0.0 : 1.0;
        });
    EXPECT_FALSE(missing_wavelength.valid);
    EXPECT_TRUE(missing_wavelength.terms.empty());
}

TEST(PPPArTest, MadocaDesignRowCanUsePppStateFrequencyAmbiguityTable) {
    const MadocaDesignFixture fixture;
    ppp_shared::PPPState filter_state;
    filter_state.total_states = fixture.state.size();
    filter_state.state = fixture.state;
    filter_state.covariance = fixture.covariance;
    for (const auto& [key, state_index] : fixture.indices) {
        filter_state.frequency_ambiguity_indices[key] = state_index;
    }

    const auto provider =
        ppp_ar::makeMadocaFrequencyAmbiguityStateIndexProvider(filter_state);
    const auto row = ppp_ar::buildMadocaArDesignRow(
        fixture.satellite,
        fixture.reference,
        1,
        2,
        provider,
        [&](const SatelliteId& satellite, int frequency_index) {
            return fixture.wavelength(satellite, frequency_index);
        });

    ASSERT_TRUE(row.valid);
    EXPECT_NEAR(ppp_ar::evaluateMadocaArDesignRow(row, filter_state.state), 7.0, 1e-12);
    EXPECT_EQ(ppp_ar::madocaFrequencyAmbiguityStateIndex(filter_state, fixture.satellite, 2), 2);
    EXPECT_EQ(ppp_ar::madocaFrequencyAmbiguityStateIndex(filter_state, fixture.satellite, -1), -1);
    EXPECT_EQ(ppp_ar::madocaFrequencyAmbiguityStateIndex(
                  filter_state, SatelliteId(GNSSSystem::GPS, 9), 0),
              -1);
}

TEST(PPPArTest, ClaslibRatioThresholdTableMatchesAlpha10Lookup) {
    EXPECT_TRUE(std::isinf(ppp_ar::claslibRatioThresholdForNb(0)));
    EXPECT_NEAR(ppp_ar::claslibRatioThresholdForNb(1), 39.86, 1e-12);
    EXPECT_NEAR(ppp_ar::claslibRatioThresholdForNb(3), 5.39, 1e-12);
    EXPECT_NEAR(ppp_ar::claslibRatioThresholdForNb(30), 1.61, 1e-12);
    EXPECT_NEAR(ppp_ar::claslibRatioThresholdForNb(60), 1.40, 1e-12);
    EXPECT_NEAR(ppp_ar::claslibRatioThresholdForNb(99), 1.40, 1e-12);
}

TEST(PPPArTest, DirectDdFixUsesClaslibRatioThresholdWhenClasFilterEnabled) {
    ppp_shared::PPPConfig config;
    config.min_satellites_for_ar = 1;
    config.ar_ratio_threshold = 2.0;

    const auto satellites = std::vector<SatelliteId>{
        SatelliteId(GNSSSystem::GPS, 1),
        SatelliteId(GNSSSystem::GPS, 2),
    };
    const std::vector<int> state_indices = {4, 5};
    const std::vector<double> scales = {1.0, 1.0};
    const auto state = makeDirectDdState(0.16);
    const auto ambiguity_states = makeDirectDdAmbiguities();

    auto loose_config = config;
    loose_config.use_clas_osr_filter = false;
    const auto loose_attempt = ppp_ar::tryDirectDdFix(
        loose_config,
        state,
        MatrixXd{},
        ambiguity_states,
        satellites,
        state_indices,
        scales,
        {},
        false);
    EXPECT_TRUE(loose_attempt.fixed);
    EXPECT_GT(loose_attempt.ratio, loose_attempt.required_ratio);
    EXPECT_DOUBLE_EQ(loose_attempt.required_ratio, 2.0);

    auto clas_config = config;
    clas_config.use_clas_osr_filter = true;
    const auto clas_attempt = ppp_ar::tryDirectDdFix(
        clas_config,
        state,
        MatrixXd{},
        ambiguity_states,
        satellites,
        state_indices,
        scales,
        {},
        false);
    EXPECT_FALSE(clas_attempt.fixed);
    EXPECT_GT(clas_attempt.ratio, clas_config.ar_ratio_threshold);
    EXPECT_NEAR(clas_attempt.required_ratio, 39.86, 1e-12);
    EXPECT_LT(clas_attempt.ratio, clas_attempt.required_ratio);
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
