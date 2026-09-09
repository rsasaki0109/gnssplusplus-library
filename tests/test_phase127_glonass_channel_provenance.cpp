#include <gtest/gtest.h>

#include <libgnss++/algorithms/fgo_config.hpp>
#include <libgnss++/algorithms/base_pseudorange_compensation.hpp>
#include <libgnss++/algorithms/phase127_glonass_channel_provenance.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/io/rinex.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

namespace {

using namespace libgnss;
using namespace libgnss::phase127_glonass;

const SatelliteId kSatellite(GNSSSystem::GLONASS, 7);
const GNSSTime kQueryTime(2200, 1000.0);

Ephemeris ephemerisAt(const GNSSTime& toe,
                      int channel,
                      bool channel_present = true) {
    Ephemeris ephemeris;
    ephemeris.satellite = kSatellite;
    ephemeris.toe = toe;
    ephemeris.toc = toe;
    ephemeris.tof = toe;
    ephemeris.glonass_frequency_channel = channel;
    ephemeris.glonass_frequency_channel_present = channel_present;
    ephemeris.valid = true;
    return ephemeris;
}

NavigationData navigationWith(std::initializer_list<Ephemeris> records) {
    NavigationData navigation;
    for (const auto& record : records) {
        navigation.addEphemeris(record);
    }
    return navigation;
}

std::string headerLine(const std::string& content, const std::string& label) {
    std::string line = content;
    if (line.size() < 60U) line.append(60U - line.size(), ' ');
    line += label;
    return line + "\n";
}

}  // namespace

TEST(Phase127GlonassChannelProvenance,
     HeaderPrimaryRequiresMatchingSelectedBroadcastEphemeris) {
    const auto navigation = navigationWith({ephemerisAt(kQueryTime, -4)});
    const auto result = resolve(
        kSatellite, kQueryTime, navigation,
        {{kSatellite, -4}});

    ASSERT_TRUE(result.accepted);
    EXPECT_EQ(result.channel, -4);
    EXPECT_EQ(result.source, ChannelSource::Header);
    EXPECT_TRUE(result.diagnostics.header_present);
    EXPECT_TRUE(result.diagnostics.header_match);
    EXPECT_FALSE(result.diagnostics.used_ephemeris_fallback);
    EXPECT_EQ(result.diagnostics.ephemeris_candidates, 1U);
}

TEST(Phase127GlonassChannelProvenance,
     MissingHeaderUsesOnlyMatchingBroadcastFallback) {
    const auto navigation = navigationWith({ephemerisAt(kQueryTime, 6)});
    const auto result = resolve(kSatellite, kQueryTime, navigation, {});

    ASSERT_TRUE(result.accepted);
    EXPECT_EQ(result.channel, 6);
    EXPECT_EQ(result.source, ChannelSource::BroadcastEphemeris);
    EXPECT_TRUE(result.diagnostics.used_ephemeris_fallback);
    EXPECT_FALSE(result.diagnostics.header_present);
}

TEST(Phase127GlonassChannelProvenance, HeaderAndBroadcastMismatchFailsClosed) {
    const auto navigation = navigationWith({ephemerisAt(kQueryTime, -3)});
    const auto result = resolve(kSatellite, kQueryTime, navigation,
                                {{kSatellite, -4}});

    EXPECT_FALSE(result.accepted);
    EXPECT_EQ(result.diagnostics.failure_code, "header-geph-fcn-mismatch");
}

TEST(Phase127GlonassChannelProvenance,
     MissingSatelliteAndQueryTimeCoverageGapFailClosed) {
    NavigationData empty_navigation;
    const auto missing = resolve(kSatellite, kQueryTime, empty_navigation, {});
    EXPECT_FALSE(missing.accepted);
    EXPECT_EQ(missing.diagnostics.failure_code, "ephemeris-missing");

    const auto stale_navigation =
        navigationWith({ephemerisAt(kQueryTime - 1800.0001, -4)});
    const auto stale = resolve(kSatellite, kQueryTime, stale_navigation, {});
    EXPECT_FALSE(stale.accepted);
    EXPECT_EQ(stale.diagnostics.failure_code, "query-time-coverage-gap");
    EXPECT_EQ(stale.diagnostics.query_time_coverage_gaps, 1U);
}

TEST(Phase127GlonassChannelProvenance,
     ExactValidityBoundaryAndOutOfRangeChannelsAreExplicit) {
    const auto at_boundary =
        navigationWith({ephemerisAt(kQueryTime - 1800.0, -7)});
    EXPECT_TRUE(resolve(kSatellite, kQueryTime, at_boundary, {}).accepted);

    const auto just_outside =
        navigationWith({ephemerisAt(kQueryTime - 1800.000001, -7)});
    EXPECT_FALSE(resolve(kSatellite, kQueryTime, just_outside, {}).accepted);

    const auto bad_ephemeris =
        navigationWith({ephemerisAt(kQueryTime, 7)});
    const auto bad_result = resolve(kSatellite, kQueryTime, bad_ephemeris, {});
    EXPECT_FALSE(bad_result.accepted);
    EXPECT_EQ(bad_result.diagnostics.failure_code, "ephemeris-fcn-invalid");

    const auto bad_header = navigationWith({ephemerisAt(kQueryTime, 0)});
    const auto bad_header_result =
        resolve(kSatellite, kQueryTime, bad_header, {{kSatellite, -8}});
    EXPECT_FALSE(bad_header_result.accepted);
    EXPECT_EQ(bad_header_result.diagnostics.failure_code, "header-fcn-invalid");
}

TEST(Phase127GlonassChannelProvenance,
     MissingAndNonfiniteEphemerisChannelPresenceFailClosed) {
    const auto missing = navigationWith({ephemerisAt(kQueryTime, 0, false)});
    const auto missing_result = resolve(kSatellite, kQueryTime, missing, {});
    EXPECT_FALSE(missing_result.accepted);
    EXPECT_EQ(missing_result.diagnostics.failure_code, "ephemeris-fcn-missing");

    GNSSTime invalid_time = kQueryTime;
    invalid_time.tow = std::numeric_limits<double>::quiet_NaN();
    const auto invalid_result = resolve(
        kSatellite, invalid_time, navigationWith({ephemerisAt(kQueryTime, 0)}), {});
    EXPECT_FALSE(invalid_result.accepted);
    EXPECT_EQ(invalid_result.diagnostics.failure_code, "query-time-invalid");
}

TEST(Phase127GlonassChannelProvenance,
     EqualAgeSameChannelIsCountedAndDifferentChannelConflicts) {
    const auto duplicate_navigation = navigationWith({
        ephemerisAt(kQueryTime - 10.0, -4),
        ephemerisAt(kQueryTime + 10.0, -4)});
    const auto duplicate =
        resolve(kSatellite, kQueryTime, duplicate_navigation, {});
    ASSERT_TRUE(duplicate.accepted);
    EXPECT_EQ(duplicate.diagnostics.ephemeris_ties, 2U);
    EXPECT_EQ(duplicate.diagnostics.ephemeris_duplicate_entries, 1U);
    EXPECT_EQ(duplicate.diagnostics.ephemeris_conflict_entries, 0U);

    const auto conflict_navigation = navigationWith({
        ephemerisAt(kQueryTime - 10.0, -4),
        ephemerisAt(kQueryTime + 10.0, -3)});
    const auto conflict =
        resolve(kSatellite, kQueryTime, conflict_navigation, {});
    EXPECT_FALSE(conflict.accepted);
    EXPECT_EQ(conflict.diagnostics.failure_code, "ephemeris-tie-fcn-conflict");
    EXPECT_EQ(conflict.diagnostics.ephemeris_conflict_entries, 1U);
}

TEST(Phase127GlonassChannelProvenance,
     HeaderDuplicatesAreCountedAndConflictsFailClosed) {
    const auto navigation = navigationWith({ephemerisAt(kQueryTime, -4)});
    const auto duplicate = resolve(
        kSatellite, kQueryTime, navigation,
        {{kSatellite, -4}, {kSatellite, -4}});
    ASSERT_TRUE(duplicate.accepted);
    EXPECT_EQ(duplicate.diagnostics.header_entries_seen, 2U);
    EXPECT_EQ(duplicate.diagnostics.header_duplicate_entries, 1U);

    const auto conflict = resolve(
        kSatellite, kQueryTime, navigation,
        {{kSatellite, -4}, {kSatellite, -3}});
    EXPECT_FALSE(conflict.accepted);
    EXPECT_EQ(conflict.diagnostics.failure_code, "header-fcn-conflict");

    const auto malformed = resolve(kSatellite, kQueryTime, navigation, {}, 1U);
    EXPECT_FALSE(malformed.accepted);
    EXPECT_EQ(malformed.diagnostics.failure_code, "header-fcn-malformed");
}

TEST(Phase127GlonassChannelProvenance, FrequencyAndWavelengthUseOnlySignedDomain) {
    const double l1 = frequencyHz(SignalType::GLO_L1CA, -4);
    const double l2 = frequencyHz(SignalType::GLO_L2CA, -4);
    EXPECT_DOUBLE_EQ(l1, constants::GLO_L1_BASE_FREQ - 4.0 * constants::GLO_L1_STEP_FREQ);
    EXPECT_DOUBLE_EQ(l2, constants::GLO_L2_BASE_FREQ - 4.0 * constants::GLO_L2_STEP_FREQ);
    EXPECT_TRUE(std::isfinite(wavelengthMeters(SignalType::GLO_L1CA, -4)));
    EXPECT_TRUE(std::isfinite(wavelengthMeters(SignalType::GLO_L2CA, -4)));
    EXPECT_TRUE(std::isnan(frequencyHz(SignalType::GLO_L1CA, -8)));
    EXPECT_TRUE(std::isnan(wavelengthMeters(SignalType::GPS_L1CA, -4)));
}

TEST(Phase127GlonassChannelProvenance,
     AnnotationUsesBroadcastChannelInsteadOfExistingRawFrequencyMetadata) {
    Observation observation(kSatellite, SignalType::GLO_L1CA);
    observation.has_glonass_frequency_channel = true;
    observation.glonass_frequency_channel = 3;
    const auto navigation = navigationWith({ephemerisAt(kQueryTime, -4)});

    const auto result = resolveAndAnnotate(
        observation, kQueryTime, navigation);
    ASSERT_TRUE(result.accepted);
    EXPECT_EQ(result.source, ChannelSource::BroadcastEphemeris);
    EXPECT_EQ(observation.glonass_frequency_channel, -4);
    EXPECT_TRUE(observation.has_glonass_frequency_channel);
}

TEST(Phase127GlonassChannelProvenance, SelectorAndLegacyDefaultsRemainOff) {
    EXPECT_FALSE(libgnss::fgo::Config{}
                     .use_native_phase127_glonass_channel_provenance);
    EXPECT_FALSE(libgnss::base_pseudorange_compensation::Config{}
                     .use_phase127_glonass_channel_provenance);
}

TEST(Phase127GlonassChannelProvenance, RINEXHeaderKeepsLegacyMapAndRawLedger) {
    const auto path = std::filesystem::temp_directory_path() /
                      "libgnss_phase127_glonass_header_fixture.obs";
    std::filesystem::remove(path);
    {
        std::ofstream file(path);
        ASSERT_TRUE(file.is_open());
        file << headerLine(
            "     3.04           OBSERVATION DATA    G                   ",
            "RINEX VERSION / TYPE");
        std::string slots(60U, ' ');
        slots.replace(4U, 7U, "R07 -4 ");
        slots.replace(11U, 7U, "R07 -4 ");
        file << headerLine(slots, "GLONASS SLOT / FRQ #");
        file << headerLine("", "END OF HEADER");
    }
    io::RINEXReader reader;
    ASSERT_TRUE(reader.open(path.string()));
    io::RINEXReader::RINEXHeader header;
    ASSERT_TRUE(reader.readHeader(header));
    EXPECT_EQ(header.glonass_frequency_channels.at(kSatellite), -4);
    ASSERT_EQ(header.glonass_frequency_channel_entries.size(), 2U);
    EXPECT_EQ(header.glonass_frequency_channel_entries[0].channel, -4);
    EXPECT_EQ(header.glonass_frequency_channel_entries[1].channel, -4);
    EXPECT_EQ(header.glonass_frequency_channel_malformed_entries, 0U);
    reader.close();
    std::filesystem::remove(path);
}
