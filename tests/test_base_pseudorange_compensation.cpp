#include <gtest/gtest.h>

#include <libgnss++/algorithms/base_pseudorange_compensation.hpp>
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
