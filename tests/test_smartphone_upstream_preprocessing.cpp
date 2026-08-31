#include <gtest/gtest.h>

#include "../apps/native/observable_upstream_preprocessing.hpp"

#include <cmath>
#include <limits>
#include <vector>

namespace {

using libgnss::SignalType;
using libgnss_apps::upstream::ObservationBand;
using libgnss_apps::upstream::carrierDopplerDifference;
using libgnss_apps::upstream::linearPercentile;
using libgnss_apps::upstream::pairThreshold;
using libgnss_apps::upstream::pseudorangeDopplerDifference;
using libgnss_apps::upstream::signalTypeFactor;
using libgnss_apps::upstream::snrPercentileSigma;
using libgnss_apps::upstream::snrScale;
using libgnss_apps::upstream::SnrPercentiles;

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

}  // namespace
