#include <gtest/gtest.h>

#include "../apps/native/observable_upstream_preprocessing.hpp"
#include <libgnss++/algorithms/source_pseudorange_miss_mask.hpp>

#include <cmath>
#include <limits>
#include <vector>

namespace {

TEST(UpstreamBaseHandoffTest, CopiedCorrectedRowsRejectSecondApplication) {
    using namespace libgnss;
    FGOProcessor::FGOProblem main;
    FGOProcessor::EpochSeed epoch;
    epoch.time = GNSSTime(2200, 100.0);
    main.epochs.push_back(epoch);
    FGOProcessor::PseudorangeFactor row;
    row.epoch_index = 0;
    row.signal = SignalType::GPS_L1CA;
    row.corrected_pseudorange_m = 20000000.0;
    row.sigma_m = 2.0;
    main.pseudorange_factors.push_back(row);
    int lookups = 0;
    const auto has_stream = [](const SatelliteId&, SignalType) { return true; };
    const auto correction = [&](const GNSSTime&, const SatelliteId&,
                                SignalType, double& value) {
        ++lookups;
        value = 12.0;
        return true;
    };
    source_pseudorange_miss_mask::Report report;
    ASSERT_TRUE(source_pseudorange_miss_mask::apply(
        main.pseudorange_factors, main.epochs, has_stream, correction, report));
    ASSERT_EQ(main.pseudorange_factors.size(), 1U);
    EXPECT_DOUBLE_EQ(main.pseudorange_factors.front().corrected_pseudorange_m,
                     19999988.0);
    auto stage = main;  // Same value-copy boundary as the ECEF-D CLI stage.
    EXPECT_TRUE(stage.pseudorange_factors.front().native_base_pseudorange_correction_applied);
    EXPECT_FALSE(source_pseudorange_miss_mask::apply(
        stage.pseudorange_factors, stage.epochs, has_stream, correction, report));
    EXPECT_TRUE(report.correction_already_applied);
    EXPECT_EQ(lookups, 1);
    EXPECT_EQ(report.correction_application_passes, 0U);
    EXPECT_DOUBLE_EQ(stage.pseudorange_factors.front().corrected_pseudorange_m,
                     main.pseudorange_factors.front().corrected_pseudorange_m);
    EXPECT_DOUBLE_EQ(stage.pseudorange_factors.front().sigma_m, 2.0);
}

TEST(UpstreamPseudorangeRemaskingTest, SeedErrorChangesAdmissionWithoutThresholdChange) {
    using namespace libgnss::observable_upstream;
    // Synthetic common-frame ranges: three equal-range good observations
    // from +X,+Y,-Y, and one -X observation with a fixed 25 m code error.
    // A +30 m receiver seed displacement hides that fourth error while
    // pushing the first good measurement outside the centered 20 m gate.
    constexpr double range = 20000000.0;
    const double lateral = std::hypot(range, 30.0);
    const std::vector<double> biased{30.0, range-lateral, range-lateral, -5.0};
    const std::vector<double> corrected{0.0, 0.0, 0.0, 25.0};
    const double biased_median = range-lateral;
    const double threshold = residualThreshold(ObservationBand::L1, 'P');
    EXPECT_TRUE(acceptsCenteredPseudorangeResidual(biased[3], biased_median, threshold));
    EXPECT_FALSE(acceptsCenteredPseudorangeResidual(corrected[3], 0.0, threshold));
    EXPECT_FALSE(acceptsCenteredPseudorangeResidual(biased[0], biased_median, threshold));
    EXPECT_TRUE(acceptsCenteredPseudorangeResidual(corrected[0], 0.0, threshold));
    EXPECT_TRUE(acceptsCenteredPseudorangeResidual(20.0, 0.0, threshold));
    EXPECT_FALSE(acceptsCenteredPseudorangeResidual(20.001, 0.0, threshold));
    EXPECT_FALSE(acceptsCenteredPseudorangeResidual(
        std::numeric_limits<double>::quiet_NaN(), 0.0, threshold));
}

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
using libgnss::observable_upstream::officialTdcpSigmaMeters;
using libgnss::observable_upstream::collectOfficialTdcpSnrPercentiles;
using libgnss::observable_upstream::kOfficialCarrierPhaseSnrRatio;
using libgnss::observable_upstream::kOfficialSnrPercentile;
using libgnss::observable_upstream::SnrPercentiles;
using libgnss::observable_upstream::EpochMask;
using libgnss::observable_upstream::applyAdjacentMasks;

TEST(UpstreamSourceTdcpMetersTest, UsesResidualMetresWithoutWavelengthConversion) {
    using libgnss::observable_upstream::sourceTdcpSigmaMeters;
    const SnrPercentiles p{40.0, 40.0};
    EXPECT_NEAR(sourceTdcpSigmaMeters(SignalType::GPS_L1CA, 40.0, p), .002, 1e-15);
    EXPECT_NEAR(sourceTdcpSigmaMeters(SignalType::GPS_L1CA, 20.0, p), .02, 1e-15);
    EXPECT_NEAR(sourceTdcpSigmaMeters(SignalType::GPS_L1CA, 60.0, p), .0002, 1e-15);
    EXPECT_NEAR(sourceTdcpSigmaMeters(SignalType::GLO_L1CA, 40.0, p), .00375, 1e-15);
    EXPECT_NEAR(sourceTdcpSigmaMeters(SignalType::GAL_E1, 40.0, p), .002, 1e-15);
    EXPECT_NEAR(sourceTdcpSigmaMeters(SignalType::GPS_L5, 40.0, p), .00125, 1e-15);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(std::isnan(sourceTdcpSigmaMeters(SignalType::GPS_L1CA, nan, p)));
    EXPECT_TRUE(std::isnan(sourceTdcpSigmaMeters(SignalType::GPS_L1CA, 0.0, p)));
    EXPECT_TRUE(std::isnan(sourceTdcpSigmaMeters(SignalType::GPS_L1CA, 40.0, {nan, 40.0})));
    // Historical helper is deliberately different; no source-unit claim here.
    EXPECT_NEAR(officialTdcpSigmaMeters(SignalType::GPS_L1CA, 40.0, p, .19),
                sourceTdcpSigmaMeters(SignalType::GPS_L1CA, 40.0, p) * .19, 1e-15);
}

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

TEST(UpstreamObservablePreprocessingTest,
     ConvertsOfficialCarrierSigmaFromCyclesToMetres) {
    EXPECT_DOUBLE_EQ(kOfficialSnrPercentile, 85.0);
    EXPECT_DOUBLE_EQ(kOfficialCarrierPhaseSnrRatio, 1.0 / 400.0);
    SnrPercentiles percentiles;
    percentiles.l1_dbhz = 40.0;
    percentiles.l5_dbhz = 40.0;

    // obserrmodel.m's L value is in source-domain carrier cycles.  Native
    // TDCP is metre-valued, so the retained wavelength is applied exactly
    // once at this boundary.
    EXPECT_NEAR(officialTdcpSigmaMeters(SignalType::GPS_L1CA, 40.0,
                                        percentiles, 2.0),
                0.8 / 400.0 * 2.0, 1e-12);
    EXPECT_NEAR(officialTdcpSigmaMeters(SignalType::GLO_L1CA, 40.0,
                                        percentiles, 2.0),
                1.5 / 400.0 * 2.0, 1e-12);
    EXPECT_NEAR(officialTdcpSigmaMeters(SignalType::GAL_E1, 40.0,
                                        percentiles, 2.0),
                0.8 / 400.0 * 2.0, 1e-12);
    EXPECT_NEAR(officialTdcpSigmaMeters(SignalType::BDS_B1I, 40.0,
                                        percentiles, 2.0),
                0.8 / 400.0 * 2.0, 1e-12);
    EXPECT_NEAR(officialTdcpSigmaMeters(SignalType::BDS_B1C, 40.0,
                                        percentiles, 2.0),
                0.8 / 400.0 * 2.0, 1e-12);
    EXPECT_NEAR(officialTdcpSigmaMeters(SignalType::GPS_L5, 40.0,
                                        percentiles, 2.0),
                0.5 / 400.0 * 2.0, 1e-12);
    EXPECT_NEAR(officialTdcpSigmaMeters(SignalType::GAL_E5A, 40.0,
                                        percentiles, 2.0),
                0.5 / 400.0 * 2.0, 1e-12);
    EXPECT_NEAR(officialTdcpSigmaMeters(SignalType::BDS_B2A, 40.0,
                                        percentiles, 2.0),
                0.5 / 400.0 * 2.0, 1e-12);
    // sysfreq2sigtype.m maps unsupported/other signals to a NaN factor.
    EXPECT_TRUE(std::isnan(officialTdcpSigmaMeters(
        SignalType::QZS_L1CA, 40.0, percentiles, 2.0)));
}

TEST(UpstreamObservablePreprocessingTest,
     OfficialTdcpPercentileIgnoresMissingSnrSentinels) {
    const libgnss::SatelliteId sat(libgnss::GNSSSystem::GPS, 3);
    libgnss::Observation valid_l1(sat, SignalType::GPS_L1CA);
    valid_l1.snr = 40.0;
    libgnss::Observation second_l1(sat, SignalType::GPS_L1CA);
    second_l1.snr = 50.0;
    libgnss::Observation missing_l1(sat, SignalType::GPS_L1CA);
    missing_l1.snr = 0.0;
    libgnss::Observation valid_l5(sat, SignalType::GPS_L5);
    valid_l5.snr = 45.0;
    libgnss::ObservationData epoch(libgnss::GNSSTime(1, 1.0));
    epoch.addObservation(valid_l1);
    epoch.addObservation(second_l1);
    epoch.addObservation(missing_l1);
    epoch.addObservation(valid_l5);

    const auto percentiles = collectOfficialTdcpSnrPercentiles({epoch});
    // With one missing zero sentinel removed, the L1 p85 of [40,50] clips to
    // the upper endpoint under MATLAB's midpoint rank convention.
    EXPECT_DOUBLE_EQ(percentiles.l1_dbhz, 50.0);
    EXPECT_DOUBLE_EQ(percentiles.l5_dbhz, 45.0);
}

TEST(UpstreamObservablePreprocessingTest,
     OfficialTdcpSigmaFailsClosedForInvalidMetadata) {
    SnrPercentiles percentiles;
    percentiles.l1_dbhz = 40.0;
    percentiles.l5_dbhz = 40.0;
    const auto nan = std::numeric_limits<double>::quiet_NaN();
    const auto inf = std::numeric_limits<double>::infinity();
    const auto sigma = [&](double snr, double percentile, double wavelength) {
        SnrPercentiles p = percentiles;
        p.l1_dbhz = percentile;
        return officialTdcpSigmaMeters(SignalType::GPS_L1CA, snr, p,
                                        wavelength);
    };

    EXPECT_TRUE(std::isnan(sigma(nan, 40.0, 0.2)));
    EXPECT_TRUE(std::isnan(sigma(0.0, 40.0, 0.2)));
    EXPECT_TRUE(std::isnan(sigma(40.0, nan, 0.2)));
    EXPECT_TRUE(std::isnan(sigma(40.0, 40.0, 0.0)));
    EXPECT_TRUE(std::isnan(sigma(40.0, 40.0, nan)));
    EXPECT_TRUE(std::isnan(sigma(40.0, 40.0, inf)));
    // pow(10, ...) overflow/underflow is also rejected rather than clipped.
    EXPECT_TRUE(std::isnan(sigma(-1.0e308, 40.0, 0.2)));
    EXPECT_TRUE(std::isnan(sigma(1.0e308, 40.0, 0.2)));
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
