#include <gtest/gtest.h>

#include <libgnss++/algorithms/base_pseudorange_compensation.hpp>
#include <libgnss++/algorithms/source_pseudorange_miss_mask.hpp>
#include <libgnss++/io/rinex.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>

namespace {

std::string headerLine(const std::string& content, const std::string& label) {
    std::string line = content;
    if (line.size() < 60U) line.append(60U - line.size(), ' ');
    line += label;
    return line + "\n";
}

std::string observationField(const std::string& value) {
    std::string field;
    if (value.size() < 14U) field.append(14U - value.size(), ' ');
    field += value;
    field += "  ";
    return field;
}

void writeAdditionalBandFixture(const std::filesystem::path& path) {
    std::ofstream file(path);
    ASSERT_TRUE(file.is_open());
    file << headerLine("     3.04           OBSERVATION DATA    G                   ",
                       "RINEX VERSION / TYPE");
    file << headerLine("G    6 C1C L1C C2W L2W C5Q L5Q", "SYS / # / OBS TYPES");
    file << headerLine("", "END OF HEADER");
    file << "> 2024 08 03 09 51 20.0000000  0  1\n";
    file << "G01"
         << observationField("22000000.000")
         << observationField("110000000.000")
         << observationField("22000010.000")
         << observationField("110000010.000")
         << observationField("22000020.000")
         << observationField("110000020.000") << "\n";
}

}  // namespace

using namespace libgnss;
using namespace libgnss::base_pseudorange_compensation;

TEST(BasePseudorangeCompensationTest, CenteredMovingMeanShrinksAtEdges) {
    const auto result = centeredMovingMean({1.0, 2.0, 3.0, 4.0, 5.0}, 3U);
    ASSERT_EQ(result.size(), 5U);
    EXPECT_DOUBLE_EQ(result[0], 1.5);
    EXPECT_DOUBLE_EQ(result[1], 2.0);
    EXPECT_DOUBLE_EQ(result[2], 3.0);
    EXPECT_DOUBLE_EQ(result[3], 4.0);
    EXPECT_DOUBLE_EQ(result[4], 4.5);
}

TEST(BasePseudorangeCompensationTest, MovingMeanOmitsNonfiniteSamples) {
    const auto result = centeredMovingMean(
        {1.0, std::numeric_limits<double>::quiet_NaN(), 3.0}, 3U);
    ASSERT_EQ(result.size(), 3U);
    EXPECT_DOUBLE_EQ(result[0], 1.0);
    EXPECT_DOUBLE_EQ(result[1], 2.0);
    EXPECT_DOUBLE_EQ(result[2], 3.0);
}

TEST(BasePseudorangeCompensationTest, SourceSignSubtractsPositiveCorrection) {
    EXPECT_DOUBLE_EQ(subtractCorrection(2'000'000.0, 12.5), 1'999'987.5);
    EXPECT_DOUBLE_EQ(subtractCorrection(2'000'000.0, -12.5), 2'000'012.5);
}

TEST(BasePseudorangeCompensationTest, InvalidBuildFailsClosedWithoutPayloadIo) {
    Model model;
    ObservationSeries empty;
    NavigationData nav;
    Config config;
    config.base_position_ecef = Vector3d(1.0, 2.0, 3.0);
    config.expected_interval_s = 1.0;
    config.moving_mean_samples = 151U;
    EXPECT_FALSE(model.build(empty, nav, config));
    EXPECT_FALSE(model.diagnostics().built);
    EXPECT_FALSE(model.diagnostics().failure.empty());
}

TEST(BasePseudorangeCompensationTest, AdditionalBandsAreExplicitlyOptIn) {
    io::RINEXReader reader;
    EXPECT_FALSE(reader.preservesAdditionalFrequencyBands());
    reader.setPreserveAdditionalFrequencyBands(true);
    EXPECT_TRUE(reader.preservesAdditionalFrequencyBands());
    reader.setPreserveAdditionalFrequencyBands(false);
    EXPECT_FALSE(reader.preservesAdditionalFrequencyBands());
}

TEST(BasePseudorangeCompensationTest, AdditionalBandFixtureAddsGpsL5OnlyWhenEnabled) {
    const auto path = std::filesystem::temp_directory_path() /
                      "libgnss_phase71_additional_band_fixture.obs";
    std::filesystem::remove(path);
    writeAdditionalBandFixture(path);

    io::RINEXReader default_reader;
    ASSERT_TRUE(default_reader.open(path.string()));
    io::RINEXReader::RINEXHeader default_header;
    ASSERT_TRUE(default_reader.readHeader(default_header));
    ObservationData default_epoch;
    ASSERT_TRUE(default_reader.readObservationEpoch(default_epoch));
    EXPECT_EQ(default_epoch.observations.size(), 2U);
    EXPECT_NE(default_epoch.getObservation(
                  SatelliteId(GNSSSystem::GPS, 1), SignalType::GPS_L1CA),
              nullptr);
    EXPECT_NE(default_epoch.getObservation(
                  SatelliteId(GNSSSystem::GPS, 1), SignalType::GPS_L2C),
              nullptr);
    EXPECT_EQ(default_epoch.getObservation(
                  SatelliteId(GNSSSystem::GPS, 1), SignalType::GPS_L5),
              nullptr);
    default_reader.close();

    io::RINEXReader preserved_reader;
    preserved_reader.setPreserveAdditionalFrequencyBands(true);
    ASSERT_TRUE(preserved_reader.open(path.string()));
    io::RINEXReader::RINEXHeader preserved_header;
    ASSERT_TRUE(preserved_reader.readHeader(preserved_header));
    ObservationData preserved_epoch;
    ASSERT_TRUE(preserved_reader.readObservationEpoch(preserved_epoch));
    EXPECT_EQ(preserved_epoch.observations.size(), 3U);
    EXPECT_NE(preserved_epoch.getObservation(
                  SatelliteId(GNSSSystem::GPS, 1), SignalType::GPS_L5),
              nullptr);
    preserved_reader.close();
    std::filesystem::remove(path);
}

TEST(BasePseudorangeCompensationTest,
     SourceExactMissMaskDropsOnlyNonfinitePcRowsAndPreservesEpochIndices) {
    using namespace libgnss::source_pseudorange_miss_mask;
    std::vector<FGOProcessor::EpochSeed> epochs(3);
    epochs[0].time = GNSSTime(2200, 100.0);
    epochs[1].time = GNSSTime(2200, 101.0);
    epochs[2].time = GNSSTime(2200, 102.0);

    const SatelliteId gps1(GNSSSystem::GPS, 1);
    const SatelliteId gps2(GNSSSystem::GPS, 2);
    FGOProcessor::PseudorangeFactor retained;
    retained.epoch_index = 2U;
    retained.satellite = gps1;
    retained.signal = SignalType::GPS_L1CA;
    retained.corrected_pseudorange_m = 100.0;
    FGOProcessor::PseudorangeFactor missing = retained;
    missing.epoch_index = 0U;
    missing.satellite = gps2;
    FGOProcessor::PseudorangeFactor outside = retained;
    outside.epoch_index = 1U;
    FGOProcessor::PseudorangeFactor nonfinite = retained;
    nonfinite.epoch_index = 0U;
    nonfinite.signal = SignalType::GPS_L5;
    std::vector<FGOProcessor::PseudorangeFactor> factors = {
        retained, missing, outside, nonfinite};

    Report report;
    ASSERT_TRUE(apply(
        factors, epochs,
        [gps1](const SatelliteId& satellite, SignalType) {
            return satellite == gps1;
        },
        [](const GNSSTime& time, const SatelliteId&, SignalType signal,
           double& correction) {
            if (time.tow == 101.0) return false;
            if (signal == SignalType::GPS_L5) {
                correction = std::numeric_limits<double>::quiet_NaN();
                return true;
            }
            correction = 2.0;
            return true;
        },
        report));

    ASSERT_EQ(factors.size(), 1U);
    EXPECT_EQ(factors.front().epoch_index, 2U);
    EXPECT_DOUBLE_EQ(factors.front().corrected_pseudorange_m, 98.0);
    EXPECT_EQ(report.original_adopted_rows, 4U);
    EXPECT_EQ(report.retained_finite_pc_rows, 1U);
    EXPECT_EQ(report.matched_exact_stream_rows, 3U);
    EXPECT_EQ(report.finite_correction_rows_among_matched, 1U);
    EXPECT_EQ(report.dropped_missing_exact_stream_rows, 1U);
    EXPECT_EQ(report.dropped_out_of_domain_rows, 1U);
    EXPECT_EQ(report.dropped_nonfinite_correction_rows, 1U);
    EXPECT_TRUE(report.factor_count_consistent);
    EXPECT_DOUBLE_EQ(report.retained_finite_pc_fraction, 1.0);
    EXPECT_DOUBLE_EQ(report.retained_over_original_fraction, 0.25);
    EXPECT_DOUBLE_EQ(report.correction_abs_p50_m, 2.0);
    EXPECT_DOUBLE_EQ(report.correction_abs_p95_m, 2.0);
    EXPECT_DOUBLE_EQ(report.correction_abs_max_m, 2.0);
}

TEST(BasePseudorangeCompensationTest, SourceExactMissMaskRejectsMissingCallbacksWithoutMutation) {
    using namespace libgnss::source_pseudorange_miss_mask;
    std::vector<FGOProcessor::EpochSeed> epochs(1);
    epochs[0].time = GNSSTime(2200, 100.0);
    FGOProcessor::PseudorangeFactor factor;
    factor.epoch_index = 0U;
    factor.corrected_pseudorange_m = 100.0;
    std::vector<FGOProcessor::PseudorangeFactor> factors = {factor};
    Report report;
    EXPECT_FALSE(apply(factors, epochs, HasStream{}, CorrectionAt{}, report));
    ASSERT_EQ(factors.size(), 1U);
    EXPECT_DOUBLE_EQ(factors.front().corrected_pseudorange_m, 100.0);
    EXPECT_FALSE(report.callback_contract_valid);
    EXPECT_EQ(report.original_adopted_rows, 1U);
}
