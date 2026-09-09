#include <gtest/gtest.h>

#include <libgnss++/algorithms/base_pseudorange_compensation.hpp>
#include <libgnss++/algorithms/fgo.hpp>
#include <libgnss++/algorithms/fgo_config.hpp>
#include <libgnss++/algorithms/phase129_glonass_local_miss.hpp>
#include <libgnss++/algorithms/source_pseudorange_miss_mask.hpp>

#include <map>
#include <string>
#include <vector>

namespace {

using libgnss::FGOProcessor;
using libgnss::GNSSTime;
using libgnss::GNSSSystem;
using libgnss::NavigationData;
using libgnss::SatelliteId;
using libgnss::SignalType;
using libgnss::phase127_glonass::Result;

Result certifiedResult() {
    Result result;
    result.accepted = true;
    return result;
}
Result rejectedResult(const std::string& reason) {
    Result result;
    result.diagnostics.failure_code = reason;
    result.diagnostics.failure = reason;
    return result;
}

FGOProcessor::PseudorangeFactor factor(std::size_t epoch_index,
                                       const SatelliteId& satellite,
                                       SignalType signal) {
    FGOProcessor::PseudorangeFactor result;
    result.epoch_index = epoch_index;
    result.satellite = satellite;
    result.signal = signal;
    result.corrected_pseudorange_m = 100.0;
    return result;
}

}  // namespace

TEST(Phase129GlonassLocalMiss, SelectorsAreDefaultOff) {
    EXPECT_FALSE(libgnss::fgo::Config{}
                     .use_native_phase129_glonass_local_miss_mask);
    EXPECT_FALSE(libgnss::base_pseudorange_compensation::Config{}
                     .use_phase129_glonass_local_miss_mask);
}

TEST(Phase129GlonassLocalMiss, SharedLedgerSeparatesCertifiedAndLocalMiss) {
    std::size_t source_rows = 0U;
    std::size_t certified_rows = 0U;
    std::size_t local_miss_rows = 0U;
    std::map<std::string, std::size_t> reason_counts;

    for (const auto& result : {certifiedResult(),
                               rejectedResult("header-fcn-mismatch"),
                               rejectedResult("geph-query-time-gap")}) {
        ++source_rows;
        const auto classification =
            libgnss::phase129_glonass_local_miss::classify(result, true);
        if (classification.accepted) {
            ++certified_rows;
        } else if (classification.local_miss) {
            ++local_miss_rows;
            ++reason_counts[classification.reason_code];
        }
    }

    EXPECT_EQ(certified_rows, 1U);
    EXPECT_EQ(local_miss_rows, 2U);
    EXPECT_EQ(reason_counts.at("header-fcn-mismatch"), 1U);
    EXPECT_EQ(reason_counts.at("geph-query-time-gap"), 1U);
    EXPECT_TRUE(libgnss::phase129_glonass_local_miss::rowLedgerConsistent(
        source_rows, certified_rows, local_miss_rows));

    const auto global =
        libgnss::phase129_glonass_local_miss::classify(
            rejectedResult("state-geometry-failure"), false);
    EXPECT_FALSE(global.accepted);
    EXPECT_FALSE(global.local_miss);
}

TEST(Phase129GlonassLocalMiss,
     SharedFactorAdmissionKeepsCertifiedGlonassAndNonGlonassRows) {
    const SatelliteId gps(GNSSSystem::GPS, 1);
    const SatelliteId certified_glo(GNSSSystem::GLONASS, 2);
    const SatelliteId uncertified_glo(GNSSSystem::GLONASS, 3);
    std::vector<FGOProcessor::EpochSeed> epochs(1);
    epochs[0].time = GNSSTime(2200, 100.0);
    std::vector<FGOProcessor::PseudorangeFactor> factors = {
        factor(0U, gps, SignalType::GPS_L1CA),
        factor(0U, certified_glo, SignalType::GLO_L1CA),
        factor(0U, uncertified_glo, SignalType::GLO_L1CA),
    };

    libgnss::source_pseudorange_miss_mask::Report report;
    ASSERT_TRUE(libgnss::source_pseudorange_miss_mask::apply(
        factors, epochs,
        [certified_glo, uncertified_glo](const SatelliteId& satellite,
                                         SignalType) {
            return satellite != uncertified_glo;
        },
        [](const GNSSTime&, const SatelliteId&, SignalType, double& correction) {
            correction = 2.0;
            return true;
        },
        report));

    ASSERT_EQ(factors.size(), 2U);
    EXPECT_EQ(factors[0].satellite, gps);
    EXPECT_EQ(factors[1].satellite, certified_glo);
    EXPECT_EQ(report.dropped_missing_exact_stream_rows, 1U);
    EXPECT_TRUE(report.factor_count_consistent);
    EXPECT_TRUE(report.signal_count_consistent);
    EXPECT_TRUE(factors[0].native_base_pseudorange_correction_applied);
    EXPECT_TRUE(factors[1].native_base_pseudorange_correction_applied);
}

TEST(Phase129GlonassLocalMiss, AllUncertifiedRowsBecomeExplicitEmptyMiss) {
    const SatelliteId glo(GNSSSystem::GLONASS, 4);
    std::vector<FGOProcessor::EpochSeed> epochs(1);
    epochs[0].time = GNSSTime(2200, 100.0);
    std::vector<FGOProcessor::PseudorangeFactor> factors = {
        factor(0U, glo, SignalType::GLO_L1CA)};
    libgnss::source_pseudorange_miss_mask::Report report;
    ASSERT_TRUE(libgnss::source_pseudorange_miss_mask::apply(
        factors, epochs,
        [](const SatelliteId&, SignalType) { return false; },
        [](const GNSSTime&, const SatelliteId&, SignalType, double&) {
            ADD_FAILURE() << "uncertified row must not request a correction";
            return false;
        },
        report));
    EXPECT_TRUE(factors.empty());
    EXPECT_EQ(report.dropped_missing_exact_stream_rows, 1U);
    EXPECT_TRUE(report.factor_count_consistent);
}

TEST(Phase129GlonassLocalMiss, CompositionFailureIsFailClosed) {
    libgnss::fgo::Config fgo_config;
    fgo_config.use_native_phase129_glonass_local_miss_mask = true;
    FGOProcessor processor(fgo_config);
    const auto problem = processor.buildPseudorangeProblem({}, NavigationData{});
    EXPECT_FALSE(problem.diagnostics.phase129_configuration_valid);
    EXPECT_FALSE(problem.diagnostics.phase129_configuration_failure.empty());
    EXPECT_EQ(problem.diagnostics.phase127_failure,
              problem.diagnostics.phase129_configuration_failure);

    libgnss::base_pseudorange_compensation::Config base_config;
    base_config.use_phase129_glonass_local_miss_mask = true;
    libgnss::base_pseudorange_compensation::Model model;
    libgnss::ObservationSeries empty;
    EXPECT_FALSE(model.build(empty, NavigationData{}, base_config));
    EXPECT_FALSE(model.diagnostics().phase129_configuration_valid);
    EXPECT_FALSE(model.diagnostics().phase129_configuration_failure.empty());
}
