#include <gtest/gtest.h>

#include "../apps/native/observable_upstream_preprocessing.hpp"

#include <cmath>
#include <limits>
#include <vector>

namespace {

using libgnss::SignalType;
using libgnss::observable_upstream::ObservationBand;
using libgnss::observable_upstream::carrierDopplerDifference;
using libgnss::observable_upstream::acceptsAbsoluteDopplerResidual;
using libgnss::observable_upstream::dopplerResidualAfterReceiverClock;
using libgnss::observable_upstream::linearPercentile;
using libgnss::observable_upstream::pairThreshold;
using libgnss::observable_upstream::pseudorangeDopplerDifference;
using libgnss::observable_upstream::signalTypeFactor;
using libgnss::observable_upstream::snrPercentileSigma;
using libgnss::observable_upstream::snrScale;
using libgnss::observable_upstream::SnrPercentiles;
using libgnss::observable_upstream::EpochMask;
using libgnss::observable_upstream::applyAdjacentMasks;

TEST(UpstreamObservablePreprocessingTest, MatchesMatlabPrctileMidpointRanks) {
    const std::vector<double> values{0.0, 10.0};
    EXPECT_DOUBLE_EQ(linearPercentile(values, 25.0), 0.0);
    EXPECT_DOUBLE_EQ(linearPercentile(values, 50.0), 5.0);
    EXPECT_DOUBLE_EQ(linearPercentile(values, 75.0), 10.0);
    EXPECT_DOUBLE_EQ(linearPercentile(values, 85.0), 10.0);
    EXPECT_TRUE(std::isnan(linearPercentile({}, 85.0)));
}

TEST(UpstreamObservablePreprocessingTest, IgnoresNonfiniteSnrForPercentile) {
    const std::vector<double> values{
        std::numeric_limits<double>::quiet_NaN(), 30.0, 40.0, 50.0};
    EXPECT_DOUBLE_EQ(linearPercentile(values, 50.0), 40.0);
    EXPECT_DOUBLE_EQ(snrScale(40.0, 40.0), 1.0);
    EXPECT_NEAR(snrScale(20.0, 40.0), 10.0, 1e-12);
}

TEST(UpstreamObservablePreprocessingTest, UsesPublishedSignalFactorsAndRatios) {
    SnrPercentiles percentiles;
    percentiles.l1_dbhz = 40.0;
    percentiles.l5_dbhz = 40.0;
    EXPECT_DOUBLE_EQ(signalTypeFactor(SignalType::GPS_L1CA), 0.8);
    EXPECT_DOUBLE_EQ(signalTypeFactor(SignalType::GLO_L1CA), 1.5);
    EXPECT_DOUBLE_EQ(signalTypeFactor(SignalType::GAL_E1), 0.8);
    EXPECT_DOUBLE_EQ(signalTypeFactor(SignalType::GPS_L5), 0.5);
    EXPECT_DOUBLE_EQ(
        snrPercentileSigma(SignalType::GPS_L1CA, 40.0, percentiles, 'P'), 0.8);
    EXPECT_NEAR(
        snrPercentileSigma(SignalType::GPS_L1CA, 40.0, percentiles, 'D'),
        1.0 / 12.0,
        1e-12);
    EXPECT_NEAR(
        snrPercentileSigma(SignalType::GPS_L1CA, 40.0, percentiles, 'L'),
        0.8 / 400.0,
        1e-12);
    EXPECT_DOUBLE_EQ(
        snrPercentileSigma(SignalType::GPS_L5, 40.0, percentiles, 'P'), 0.5);
}

TEST(UpstreamObservablePreprocessingTest, PortsHandComputablePairEquations) {
    // (-lambda*(D1+D2)/2*dt) - (P2-P1), D in cycles/s.
    EXPECT_DOUBLE_EQ(
        pseudorangeDopplerDifference(100.0, 98.0, 10.0, 10.0, 2.0, 1.0),
        -18.0);
    // The optional 1.117 m device offset is subtracted exactly as upstream.
    EXPECT_NEAR(
        carrierDopplerDifference(50.0, 40.0, 10.0, 10.0, 2.0, 1.0, 1.117),
        -1.117,
        1e-12);
    EXPECT_DOUBLE_EQ(pairThreshold(ObservationBand::L1, 'P'), 40.0);
    EXPECT_DOUBLE_EQ(pairThreshold(ObservationBand::L5, 'P'), 20.0);
    EXPECT_DOUBLE_EQ(pairThreshold(ObservationBand::L1, 'L'), 1.5);
}

TEST(UpstreamObservablePreprocessingTest, AppliesTwoSidedAdjacentMasksAndGapGate) {
    const libgnss::SatelliteId sat(libgnss::GNSSSystem::GPS, 7);
    libgnss::Observation first(sat, SignalType::GPS_L1CA);
    first.valid = true;
    first.has_pseudorange = true;
    first.has_doppler = true;
    first.pseudorange = 20'000'000.0;
    first.doppler = 0.0;
    first.snr = 40.0;
    libgnss::Observation second = first;
    second.pseudorange += 100.0;  // |dDP| = 100 m > the 40 m L1 bound.
    libgnss::ObservationData epoch0(libgnss::GNSSTime(1, 10.0));
    epoch0.addObservation(first);
    libgnss::ObservationData epoch1(libgnss::GNSSTime(1, 11.0));
    epoch1.addObservation(second);
    std::vector<EpochMask> masks;
    std::size_t pd_rejections = 0;
    std::size_t ld_rejections = 0;
    applyAdjacentMasks({epoch0, epoch1}, "pixel7pro", masks,
                       pd_rejections, ld_rejections);
    ASSERT_EQ(masks.size(), 2U);
    EXPECT_EQ(pd_rejections, 2U);
    EXPECT_EQ(masks[0].pseudorange.size(), 1U);
    EXPECT_EQ(masks[1].pseudorange.size(), 1U);

    // The exact 1.5 s continuity contract does not compare across a gap.
    epoch1.time = libgnss::GNSSTime(1, 11.501);
    applyAdjacentMasks({epoch0, epoch1}, "pixel7pro", masks,
                       pd_rejections, ld_rejections);
    EXPECT_EQ(pd_rejections, 0U);
    EXPECT_TRUE(masks[0].pseudorange.empty());
    EXPECT_TRUE(masks[1].pseudorange.empty());
}

TEST(UpstreamObservablePreprocessingTest,
     AppliesRawClockDriftToAbsoluteDopplerResidual) {
    // DriftNanosPerSecond has already been converted to range m/s by the
    // Android adapter.  The upstream residual rule then divides by the
    // scalar observation interval before applying its 3 m/s bound.
    EXPECT_DOUBLE_EQ(dopplerResidualAfterReceiverClock(10.0, 2.0, 2.0), 9.0);
    EXPECT_TRUE(acceptsAbsoluteDopplerResidual(4.0, 2.0, 2.0, 3.0));
    EXPECT_FALSE(acceptsAbsoluteDopplerResidual(4.01, 2.0, 2.0, 3.0));
    EXPECT_FALSE(acceptsAbsoluteDopplerResidual(
        10.0, std::numeric_limits<double>::quiet_NaN(), 1.0, 3.0));
    EXPECT_TRUE(std::isnan(dopplerResidualAfterReceiverClock(
        10.0, 2.0, 0.0)));
}

}  // namespace
