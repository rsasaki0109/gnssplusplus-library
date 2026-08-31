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

}  // namespace
