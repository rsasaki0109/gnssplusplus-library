#include <gtest/gtest.h>

#include <libgnss++/algorithms/carrier_code_leveling.hpp>

#include <cmath>
#include <cstdint>
#include <string>

namespace {

using libgnss::GNSSSystem;
using libgnss::GNSSTime;
using libgnss::Observation;
using libgnss::ObservationData;
using libgnss::ObservationSeries;
using libgnss::SatelliteId;
using libgnss::SignalType;
using libgnss::carrier_code_leveling::Config;

Observation makeE1(double code_m, double adr_m, uint8_t lli = 0U) {
    Observation observation(SatelliteId(GNSSSystem::Galileo, 5),
                            SignalType::GAL_E1);
    const double wavelength = libgnss::signalWavelengthMeters(observation);
    observation.pseudorange = code_m;
    observation.has_pseudorange = true;
    observation.carrier_phase = adr_m / wavelength;
    observation.has_carrier_phase = true;
    observation.lli = lli;
    observation.valid = true;
    return observation;
}

Observation makeL1(double code_m, double adr_m, uint8_t lli = 0U) {
    Observation observation(SatelliteId(GNSSSystem::GPS, 5),
                            SignalType::GPS_L1CA);
    const double wavelength = libgnss::signalWavelengthMeters(observation);
    observation.pseudorange = code_m;
    observation.has_pseudorange = true;
    observation.carrier_phase = adr_m / wavelength;
    observation.has_carrier_phase = true;
    observation.lli = lli;
    observation.valid = true;
    return observation;
}

Observation makeE5a(double code_m, double adr_m, uint8_t lli = 0U) {
    Observation observation(SatelliteId(GNSSSystem::Galileo, 5),
                            SignalType::GAL_E5A);
    const double wavelength = libgnss::signalWavelengthMeters(observation);
    observation.pseudorange = code_m;
    observation.has_pseudorange = true;
    observation.carrier_phase = adr_m / wavelength;
    observation.has_carrier_phase = true;
    observation.lli = lli;
    observation.valid = true;
    return observation;
}

ObservationSeries threeEpochSeries() {
    ObservationSeries series;
    for (int i = 0; i < 3; ++i) {
        ObservationData epoch(GNSSTime(2200, 100.0 + i));
        epoch.addObservation(makeE1(20'000'000.0 + 0.4 * i,
                                    100.0 + 0.2 * i));
        series.addEpoch(epoch);
    }
    return series;
}

ObservationSeries twoEpochCodeSeries(double current_code_m) {
    ObservationSeries series;
    ObservationData first(GNSSTime(2200, 100.0));
    first.addObservation(makeE1(20'000'000.0, 100.0));
    ObservationData second(GNSSTime(2200, 101.0));
    // Keeping ADR constant makes the innovation equal to the hand-chosen
    // raw-code step, which exercises the exact upstream 40 m boundary.
    second.addObservation(makeE1(current_code_m, 100.0));
    series.addEpoch(first);
    series.addEpoch(second);
    return series;
}

ObservationSeries twoEpochE5aCodeSeries(double current_code_m) {
    ObservationSeries series;
    ObservationData first(GNSSTime(2200, 100.0));
    first.addObservation(makeE5a(20'000'000.0, 100.0));
    ObservationData second(GNSSTime(2200, 101.0));
    second.addObservation(makeE5a(current_code_m, 100.0));
    series.addEpoch(first);
    series.addEpoch(second);
    return series;
}

TEST(CarrierCodeLevelingTest, UsesCausalThirtySampleRecursion) {
    const auto input = threeEpochSeries();
    const auto result = libgnss::carrier_code_leveling::apply(
        input, {1'000, 2'000, 3'000}, {0, 0, 0});
    ASSERT_TRUE(result.ok) << result.error;
    ASSERT_EQ(result.observations.epochs.size(), 3U);
    const auto& first = result.observations.epochs[0].observations.front();
    const auto& second = result.observations.epochs[1].observations.front();
    const auto& third = result.observations.epochs[2].observations.front();
    EXPECT_DOUBLE_EQ(first.pseudorange, 20'000'000.0);
    // predicted = 20,000,000 + 0.2; N=2; average with raw 20,000,000.4
    EXPECT_DOUBLE_EQ(second.pseudorange, 20'000'000.3);
    // predicted = 20,000,000.5; N=3; average with raw 20,000,000.8
    EXPECT_DOUBLE_EQ(third.pseudorange, 20'000'000.6);
    EXPECT_EQ(result.diagnostics.arcs_started, 1U);
    EXPECT_EQ(result.diagnostics.updates, 2U);
    EXPECT_EQ(result.diagnostics.smoothed_rows, 2U);
    EXPECT_NEAR(result.diagnostics.max_abs_phase_increment_m, 0.2, 1e-12);
}

TEST(CarrierCodeLevelingTest, ResetsOnClockChangeAndGap) {
    ObservationSeries series;
    for (int i = 0; i < 3; ++i) {
        ObservationData epoch(GNSSTime(2200, 100.0 + i));
        epoch.addObservation(makeE1(20'000'000.0 + i, 100.0 + i));
        series.addEpoch(epoch);
    }
    const auto result = libgnss::carrier_code_leveling::apply(
        series, {1'000, 2'000, 5'000}, {0, 1, 1});
    ASSERT_TRUE(result.ok) << result.error;
    EXPECT_EQ(result.diagnostics.reset_clock_discontinuity, 1U);
    EXPECT_EQ(result.diagnostics.reset_gap, 1U);
    EXPECT_EQ(result.diagnostics.updates, 0U);
    EXPECT_EQ(result.diagnostics.arcs_started, 3U);
    for (const auto& epoch : result.observations.epochs) {
        EXPECT_TRUE(std::isfinite(epoch.observations.front().pseudorange));
    }
}

TEST(CarrierCodeLevelingTest, ResetRowsEmitRawAndNonTargetIsUntouched) {
    ObservationSeries series;
    ObservationData epoch0(GNSSTime(2200, 100.0));
    epoch0.addObservation(makeE1(20'000'000.0, 100.0));
    Observation other(SatelliteId(GNSSSystem::GPS, 3), SignalType::GPS_L1CA);
    other.pseudorange = 21'000'000.0;
    other.has_pseudorange = true;
    epoch0.addObservation(other);
    ObservationData epoch1(GNSSTime(2200, 101.0));
    epoch1.addObservation(makeE1(20'000'001.0, 100.2, 1U));
    series.addEpoch(epoch0);
    series.addEpoch(epoch1);

    const auto result = libgnss::carrier_code_leveling::apply(
        series, {1'000, 2'000}, {0, 0});
    ASSERT_TRUE(result.ok) << result.error;
    EXPECT_DOUBLE_EQ(result.observations.epochs[1].observations.front().pseudorange,
                     20'000'001.0);
    EXPECT_DOUBLE_EQ(result.observations.epochs[0].observations.back().pseudorange,
                     21'000'000.0);
    EXPECT_EQ(result.diagnostics.reset_cycle_slip, 1U);
}

TEST(CarrierCodeLevelingTest, RejectsMissingClockVectorByDefault) {
    const auto result = libgnss::carrier_code_leveling::apply(
        threeEpochSeries(), {1'000, 2'000, 3'000}, {});
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.error.find("hardware clock"), std::string::npos);
}

TEST(CarrierCodeLevelingTest, RejectsNonMonotonicEpochKeys) {
    const auto result = libgnss::carrier_code_leveling::apply(
        threeEpochSeries(), {1'000, 1'000, 3'000}, {0, 0, 0});
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.error.find("strictly increasing"), std::string::npos);
}

TEST(CarrierCodeLevelingTest, InnovationResetUsesUpstreamL1Boundary) {
    Config config;
    config.enable_innovation_reset = true;

    const auto below = libgnss::carrier_code_leveling::apply(
        twoEpochCodeSeries(20'000'039.9), {1'000, 2'000}, {0, 0}, config);
    ASSERT_TRUE(below.ok) << below.error;
    EXPECT_EQ(below.diagnostics.reset_innovation, 0U);
    EXPECT_EQ(below.diagnostics.updates, 1U);
    EXPECT_NEAR(
        below.observations.epochs[1].observations.front().pseudorange,
        20'000'019.95, 1.0e-9);
    EXPECT_NEAR(below.diagnostics.max_abs_innovation_accepted_m, 39.9,
                1.0e-8);

    const auto exact = libgnss::carrier_code_leveling::apply(
        twoEpochCodeSeries(20'000'040.0), {1'000, 2'000}, {0, 0}, config);
    ASSERT_TRUE(exact.ok) << exact.error;
    EXPECT_DOUBLE_EQ(exact.diagnostics.innovation_reset_threshold_m, 40.0);
    EXPECT_EQ(exact.diagnostics.reset_innovation, 0U);
    EXPECT_EQ(exact.diagnostics.updates, 1U);
    EXPECT_DOUBLE_EQ(
        exact.observations.epochs[1].observations.front().pseudorange,
        20'000'020.0);
    EXPECT_DOUBLE_EQ(exact.diagnostics.max_abs_innovation_accepted_m, 40.0);
    EXPECT_DOUBLE_EQ(exact.diagnostics.max_abs_innovation_rejected_m, 0.0);

    const auto over = libgnss::carrier_code_leveling::apply(
        twoEpochCodeSeries(20'000'040.1), {1'000, 2'000}, {0, 0}, config);
    ASSERT_TRUE(over.ok) << over.error;
    EXPECT_EQ(over.diagnostics.reset_innovation, 1U);
    EXPECT_EQ(over.diagnostics.updates, 0U);
    EXPECT_DOUBLE_EQ(
        over.observations.epochs[1].observations.front().pseudorange,
        20'000'040.1);
    EXPECT_NEAR(over.diagnostics.max_abs_innovation_rejected_m, 40.1,
                1.0e-8);
    EXPECT_EQ(over.diagnostics.arcs_started, 2U);
}

TEST(CarrierCodeLevelingTest, InnovationResetIsOptInAndDefaultPreservesHatch) {
    const auto result = libgnss::carrier_code_leveling::apply(
        twoEpochCodeSeries(20'000'040.1), {1'000, 2'000}, {0, 0});
    ASSERT_TRUE(result.ok) << result.error;
    EXPECT_EQ(result.diagnostics.reset_innovation, 0U);
    EXPECT_EQ(result.diagnostics.innovation_reset_threshold_m, 0.0);
    EXPECT_NEAR(
        result.observations.epochs[1].observations.front().pseudorange,
        20'000'020.05, 1.0e-9);
}

TEST(CarrierCodeLevelingTest, GPSInnovationResetUsesSameUpstreamL1Boundary) {
    Config config;
    config.signal = SignalType::GPS_L1CA;
    config.enable_innovation_reset = true;
    ObservationSeries series;
    ObservationData first(GNSSTime(2200, 100.0));
    first.addObservation(makeL1(20'000'000.0, 100.0));
    ObservationData second(GNSSTime(2200, 101.0));
    second.addObservation(makeL1(20'000'040.1, 100.0));
    series.addEpoch(first);
    series.addEpoch(second);

    const auto result = libgnss::carrier_code_leveling::apply(
        series, {1'000, 2'000}, {0, 0}, config);
    ASSERT_TRUE(result.ok) << result.error;
    ASSERT_EQ(result.diagnostics.per_signal.size(), 1U);
    EXPECT_EQ(result.diagnostics.per_signal[0].signal, SignalType::GPS_L1CA);
    EXPECT_EQ(result.diagnostics.per_signal[0].reset_innovation, 1U);
    EXPECT_DOUBLE_EQ(result.diagnostics.per_signal[0].innovation_reset_threshold_m,
                     40.0);
    EXPECT_DOUBLE_EQ(
        result.observations.epochs[1].observations.front().pseudorange,
        20'000'040.1);
}

TEST(CarrierCodeLevelingTest, ExplicitPrimarySetKeepsSignalArcsIndependent) {
    Config config;
    config.signals = {SignalType::GPS_L1CA, SignalType::GAL_E1};
    config.enable_innovation_reset = true;

    ObservationSeries series;
    ObservationData first(GNSSTime(2200, 100.0));
    first.addObservation(makeL1(20'000'000.0, 100.0));
    first.addObservation(makeE1(21'000'000.0, 200.0));
    ObservationData second(GNSSTime(2200, 101.0));
    second.addObservation(makeL1(20'000'000.4, 100.2));
    second.addObservation(makeE1(21'000'000.4, 200.2));
    Observation untouched(SatelliteId(GNSSSystem::GPS, 5), SignalType::GPS_L5);
    untouched.pseudorange = 22'000'000.0;
    untouched.has_pseudorange = true;
    untouched.valid = true;
    second.addObservation(untouched);
    series.addEpoch(first);
    series.addEpoch(second);

    const auto result = libgnss::carrier_code_leveling::apply(
        series, {1'000, 2'000}, {0, 0}, config);
    ASSERT_TRUE(result.ok) << result.error;
    ASSERT_EQ(result.diagnostics.per_signal.size(), 2U);
    EXPECT_EQ(result.diagnostics.per_signal[0].signal, SignalType::GPS_L1CA);
    EXPECT_EQ(result.diagnostics.per_signal[1].signal, SignalType::GAL_E1);
    for (const auto& diagnostics : result.diagnostics.per_signal) {
        EXPECT_EQ(diagnostics.target_rows, 2U);
        EXPECT_EQ(diagnostics.eligible_rows, 2U);
        EXPECT_EQ(diagnostics.updates, 1U);
        EXPECT_EQ(diagnostics.reset_innovation, 0U);
        EXPECT_DOUBLE_EQ(diagnostics.innovation_reset_threshold_m, 40.0);
    }
    EXPECT_DOUBLE_EQ(result.observations.epochs[1].observations[2].pseudorange,
                     22'000'000.0);
}

TEST(CarrierCodeLevelingTest, ExplicitPrimarySetRejectsUnsupportedBand) {
    Config config;
    config.signals = {SignalType::GPS_L5};
    const auto result = libgnss::carrier_code_leveling::apply(
        twoEpochCodeSeries(20'000'000.0), {1'000, 2'000}, {0, 0}, config);
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.error.find("GPS_L1CA and GAL_E1"), std::string::npos);
}

TEST(CarrierCodeLevelingTest, GalileoE5aInnovationResetUsesTwentyMeterBoundary) {
    Config config;
    config.include_gal_e5a = true;
    config.enable_innovation_reset = true;

    const auto below = libgnss::carrier_code_leveling::apply(
        twoEpochE5aCodeSeries(20'000'019.9), {1'000, 2'000}, {0, 0}, config);
    ASSERT_TRUE(below.ok) << below.error;
    ASSERT_EQ(below.diagnostics.per_signal.size(), 2U);
    EXPECT_EQ(below.diagnostics.per_signal[1].signal, SignalType::GAL_E5A);
    EXPECT_DOUBLE_EQ(below.diagnostics.per_signal[1].innovation_reset_threshold_m,
                     20.0);
    EXPECT_EQ(below.diagnostics.per_signal[1].reset_innovation, 0U);
    EXPECT_EQ(below.diagnostics.per_signal[1].updates, 1U);
    EXPECT_NEAR(below.observations.epochs[1].observations.front().pseudorange,
                20'000'009.95, 1.0e-9);

    const auto exact = libgnss::carrier_code_leveling::apply(
        twoEpochE5aCodeSeries(20'000'020.0), {1'000, 2'000}, {0, 0}, config);
    ASSERT_TRUE(exact.ok) << exact.error;
    EXPECT_EQ(exact.diagnostics.per_signal[1].reset_innovation, 0U);
    EXPECT_EQ(exact.diagnostics.per_signal[1].updates, 1U);
    EXPECT_DOUBLE_EQ(exact.diagnostics.per_signal[1].max_abs_innovation_accepted_m,
                     20.0);
    EXPECT_NEAR(exact.observations.epochs[1].observations.front().pseudorange,
                20'000'010.0, 1.0e-9);

    const auto over = libgnss::carrier_code_leveling::apply(
        twoEpochE5aCodeSeries(20'000'020.1), {1'000, 2'000}, {0, 0}, config);
    ASSERT_TRUE(over.ok) << over.error;
    EXPECT_EQ(over.diagnostics.per_signal[1].reset_innovation, 1U);
    EXPECT_EQ(over.diagnostics.per_signal[1].updates, 0U);
    EXPECT_DOUBLE_EQ(over.observations.epochs[1].observations.front().pseudorange,
                     20'000'020.1);
    EXPECT_NEAR(over.diagnostics.per_signal[1].max_abs_innovation_rejected_m,
                20.1, 1.0e-8);
}

TEST(CarrierCodeLevelingTest, GalileoE1E5aCandidateKeepsArcsAndGPSL1Separate) {
    Config config;
    config.include_gal_e5a = true;
    config.enable_innovation_reset = true;

    ObservationSeries series;
    ObservationData first(GNSSTime(2200, 100.0));
    first.addObservation(makeE1(21'000'000.0, 100.0));
    first.addObservation(makeE5a(22'000'000.0, 200.0));
    first.addObservation(makeL1(23'000'000.0, 300.0));
    ObservationData second(GNSSTime(2200, 101.0));
    second.addObservation(makeE1(21'000'000.4, 100.2));
    // The E5a reset must not affect the E1 arc, and GPS L1 is not selected by
    // the Phase 19 candidate at all.
    second.addObservation(makeE5a(22'000'020.1, 200.0));
    second.addObservation(makeL1(23'000'040.0, 300.0));
    series.addEpoch(first);
    series.addEpoch(second);

    const auto result = libgnss::carrier_code_leveling::apply(
        series, {1'000, 2'000}, {0, 0}, config);
    ASSERT_TRUE(result.ok) << result.error;
    ASSERT_EQ(result.diagnostics.per_signal.size(), 2U);
    EXPECT_EQ(result.diagnostics.per_signal[0].signal, SignalType::GAL_E1);
    EXPECT_EQ(result.diagnostics.per_signal[1].signal, SignalType::GAL_E5A);
    EXPECT_EQ(result.diagnostics.per_signal[0].updates, 1U);
    EXPECT_EQ(result.diagnostics.per_signal[0].reset_innovation, 0U);
    EXPECT_EQ(result.diagnostics.per_signal[1].updates, 0U);
    EXPECT_EQ(result.diagnostics.per_signal[1].reset_innovation, 1U);
    EXPECT_DOUBLE_EQ(result.observations.epochs[1].observations[0].pseudorange,
                     21'000'000.3);
    EXPECT_DOUBLE_EQ(result.observations.epochs[1].observations[1].pseudorange,
                     22'000'020.1);
    EXPECT_DOUBLE_EQ(result.observations.epochs[1].observations[2].pseudorange,
                     23'000'040.0);
    EXPECT_EQ(result.diagnostics.target_rows, 4U);
}

TEST(CarrierCodeLevelingTest, GalileoE5aExtensionRejectsExplicitMixedSet) {
    Config config;
    config.include_gal_e5a = true;
    config.signals = {SignalType::GAL_E1};
    const auto result = libgnss::carrier_code_leveling::apply(
        twoEpochCodeSeries(20'000'000.0), {1'000, 2'000}, {0, 0}, config);
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.error.find("single GAL_E1"), std::string::npos);
}

}  // namespace
