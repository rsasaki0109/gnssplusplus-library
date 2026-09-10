#include <gtest/gtest.h>

#include <libgnss++/algorithms/base_pseudorange_compensation.hpp>
#include <libgnss++/algorithms/fgo_config.hpp>
#include <libgnss++/algorithms/phase131_canonical_correction_key.hpp>
#include <libgnss++/algorithms/source_pseudorange_miss_mask.hpp>
#include <libgnss++/core/signal_policy.hpp>

#include <cmath>
#include <set>
#include <vector>

namespace {

using libgnss::FGOProcessor;
using libgnss::GNSSTime;
using libgnss::GNSSSystem;
using libgnss::SatelliteId;
using libgnss::SignalType;

FGOProcessor::PseudorangeFactor factor(
    std::size_t epoch_index, const SatelliteId& satellite,
    SignalType signal, bool has_fcn = false, int fcn = 0) {
    FGOProcessor::PseudorangeFactor result;
    result.epoch_index = epoch_index;
    result.satellite = satellite;
    result.signal = signal;
    result.has_glonass_frequency_channel = has_fcn;
    result.glonass_frequency_channel = fcn;
    result.corrected_pseudorange_m = 100.0;
    return result;
}

}  // namespace

TEST(Phase131CanonicalCorrectionKey, SelectorsAreDefaultOff) {
    EXPECT_FALSE(libgnss::fgo::Config{}
                     .use_native_phase131_canonical_correction_band_key);
    EXPECT_FALSE(libgnss::base_pseudorange_compensation::Config{}
                     .use_phase131_canonical_correction_band_key);
}

TEST(Phase131CanonicalCorrectionKey, SamePhysicalFamilyIgnoresLiteralTrackCode) {
    const SatelliteId glo(GNSSSystem::GLONASS, 7);
    const auto ca = libgnss::phase131_canonical::canonicalize(
        glo, SignalType::GLO_L1CA, true, -4);
    const auto p = libgnss::phase131_canonical::canonicalize(
        glo, SignalType::GLO_L1P, true, -4);
    ASSERT_TRUE(ca.accepted);
    ASSERT_TRUE(p.accepted);
    EXPECT_EQ(ca.key, p.key);
    EXPECT_EQ(ca.key.family,
              libgnss::phase131_canonical::PhysicalFrequencyFamily::L1);
    EXPECT_FALSE(ca.key == libgnss::phase131_canonical::canonicalize(
                              glo, SignalType::GLO_L1P, true, -3)
                              .key);
    EXPECT_EQ(libgnss::phase131_canonical::keyString(ca.key),
              "2:7:L1:fcn=-4");
}

TEST(Phase131CanonicalCorrectionKey, TypedRinexAliasesMapToTheSameFamily) {
    const auto c1c = libgnss::phase131_canonical::familyForRinexObservationType(
        GNSSSystem::GLONASS, "C1C");
    const auto c1p = libgnss::phase131_canonical::familyForRinexObservationType(
        GNSSSystem::GLONASS, "C1P");
    EXPECT_EQ(c1c, libgnss::phase131_canonical::PhysicalFrequencyFamily::L1);
    EXPECT_EQ(c1p, libgnss::phase131_canonical::PhysicalFrequencyFamily::L1);
    EXPECT_EQ(libgnss::phase131_canonical::familyForRinexObservationType(
                  GNSSSystem::GLONASS, "C2P"),
              libgnss::phase131_canonical::PhysicalFrequencyFamily::Unknown);
    EXPECT_EQ(libgnss::phase131_canonical::familyForAndroidSignalType(
                  SignalType::GPS_L5),
              libgnss::phase131_canonical::PhysicalFrequencyFamily::L5);
}

TEST(Phase131CanonicalCorrectionKey, FCNAndUnknownBandAreFailClosed) {
    const SatelliteId glo(GNSSSystem::GLONASS, 7);
    EXPECT_FALSE(libgnss::phase131_canonical::canonicalize(
                     glo, SignalType::GLO_L1CA, false, 0)
                     .accepted);
    EXPECT_FALSE(libgnss::phase131_canonical::canonicalize(
                     glo, SignalType::GLO_L1CA, true, -8)
                     .accepted);
    EXPECT_FALSE(libgnss::phase131_canonical::canonicalize(
                     glo, SignalType::GLO_L2CA, true, 0)
                     .accepted);
    EXPECT_FALSE(libgnss::phase131_canonical::canonicalize(
                     SatelliteId(GNSSSystem::GPS, 1), SignalType::GPS_L1CA,
                     true, 0)
                     .accepted);
}

TEST(Phase131CanonicalCorrectionKey,
     CanonicalMissMaskPassesCertifiedFCNAndPreservesFactorOrder) {
    const SatelliteId gps(GNSSSystem::GPS, 1);
    const SatelliteId glo(GNSSSystem::GLONASS, 7);
    const SatelliteId uncertified(GNSSSystem::GLONASS, 8);
    std::vector<FGOProcessor::EpochSeed> epochs(1);
    epochs[0].time = GNSSTime(2200, 100.0);
    std::vector<FGOProcessor::PseudorangeFactor> factors = {
        factor(0U, gps, SignalType::GPS_L1CA),
        factor(0U, glo, SignalType::GLO_L1P, true, -4),
        factor(0U, uncertified, SignalType::GLO_L1CA),
    };
    std::set<int> observed_fcn;
    libgnss::source_pseudorange_miss_mask::Report report;
    ASSERT_TRUE(libgnss::source_pseudorange_miss_mask::applyCanonical(
        factors, epochs,
        [&observed_fcn, glo](const SatelliteId& satellite, SignalType signal,
                             bool has_fcn, int fcn) {
            if (satellite == glo) {
                EXPECT_EQ(signal, SignalType::GLO_L1P);
                EXPECT_TRUE(has_fcn);
                observed_fcn.insert(fcn);
                return fcn == -4;
            }
            return satellite.system == GNSSSystem::GPS &&
                   signal == SignalType::GPS_L1CA && !has_fcn;
        },
        [](const GNSSTime&, const SatelliteId&, SignalType, bool has_fcn,
           int fcn, double& correction_m) {
            correction_m = has_fcn ? 2.0 + static_cast<double>(fcn) : 1.0;
            return std::isfinite(correction_m);
        },
        report));
    ASSERT_EQ(factors.size(), 2U);
    EXPECT_EQ(factors[0].satellite, gps);
    EXPECT_EQ(factors[1].satellite, glo);
    EXPECT_EQ(observed_fcn, std::set<int>({-4}));
    EXPECT_TRUE(report.canonical_key_mode);
    EXPECT_EQ(report.matched_exact_stream_rows, 2U);
    EXPECT_EQ(report.dropped_missing_exact_stream_rows, 1U);
    EXPECT_TRUE(report.factor_count_consistent);
    EXPECT_TRUE(report.signal_count_consistent);
    EXPECT_TRUE(factors[0].native_base_pseudorange_correction_applied);
    EXPECT_TRUE(factors[1].native_base_pseudorange_correction_applied);
    EXPECT_DOUBLE_EQ(factors[1].corrected_pseudorange_m, 102.0);
}

TEST(Phase131CanonicalCorrectionKey, AlreadyCorrectedInputIsRejectedBeforeMutation) {
    const SatelliteId gps(GNSSSystem::GPS, 1);
    std::vector<FGOProcessor::EpochSeed> epochs(1);
    epochs[0].time = GNSSTime(2200, 100.0);
    auto input = factor(0U, gps, SignalType::GPS_L1CA);
    input.native_base_pseudorange_correction_applied = true;
    const double before = input.corrected_pseudorange_m;
    std::vector<FGOProcessor::PseudorangeFactor> factors = {input};
    libgnss::source_pseudorange_miss_mask::Report report;
    EXPECT_FALSE(libgnss::source_pseudorange_miss_mask::applyCanonical(
        factors, epochs,
        [](const SatelliteId&, SignalType, bool, int) { return true; },
        [](const GNSSTime&, const SatelliteId&, SignalType, bool, int,
           double& correction_m) {
            correction_m = 1.0;
            return true;
        },
        report));
    ASSERT_EQ(factors.size(), 1U);
    EXPECT_DOUBLE_EQ(factors.front().corrected_pseudorange_m, before);
    EXPECT_TRUE(report.correction_already_applied);
    EXPECT_TRUE(report.canonical_key_mode);
}

TEST(Phase131CanonicalCorrectionKey, CompositionFailureIsFailClosed) {
    libgnss::fgo::Config fgo_config;
    fgo_config.use_native_phase131_canonical_correction_band_key = true;
    libgnss::FGOProcessor processor(fgo_config);
    const auto problem = processor.buildPseudorangeProblem({},
                                                           libgnss::NavigationData{});
    EXPECT_FALSE(problem.diagnostics.phase131_configuration_valid);
    EXPECT_FALSE(problem.diagnostics.phase131_configuration_failure.empty());

    libgnss::base_pseudorange_compensation::Config base_config;
    base_config.use_phase131_canonical_correction_band_key = true;
    libgnss::base_pseudorange_compensation::Model model;
    libgnss::ObservationSeries empty;
    EXPECT_FALSE(model.build(empty, libgnss::NavigationData{}, base_config));
    EXPECT_FALSE(model.diagnostics().phase131_configuration_valid);
    EXPECT_FALSE(model.diagnostics().phase131_configuration_failure.empty());
}
