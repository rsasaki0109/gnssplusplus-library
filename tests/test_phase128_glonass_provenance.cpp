#include <gtest/gtest.h>

#include <libgnss++/algorithms/fgo_config.hpp>
#include <libgnss++/algorithms/phase128_glonass_provenance.hpp>
#include <libgnss++/core/glonass_provenance.hpp>
#include <libgnss++/io/rinex.hpp>

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

namespace {

using namespace libgnss;
using namespace libgnss::phase128_glonass;

const SatelliteId kSatellite(GNSSSystem::GLONASS, 7);
const GNSSTime kQueryTime(2200, 1000.0);

Ephemeris ephemerisAt(int channel, bool canonical = true) {
    Ephemeris ephemeris;
    ephemeris.satellite = kSatellite;
    ephemeris.toe = kQueryTime;
    ephemeris.toc = kQueryTime;
    ephemeris.tof = kQueryTime;
    ephemeris.glonass_frequency_channel = channel;
    ephemeris.glonass_frequency_channel_present = true;
    ephemeris.glonass_canonical_geph_data_valid = canonical;
    ephemeris.valid = true;
    return ephemeris;
}

NavigationData navigationWith(std::initializer_list<Ephemeris> records) {
    NavigationData navigation;
    for (const auto& record : records) navigation.addEphemeris(record);
    return navigation;
}

std::string headerLine(const std::string& content, const std::string& label) {
    std::string line = content;
    if (line.size() < 60U) line.append(60U - line.size(), ' ');
    return line + label + "\n";
}

std::filesystem::path writeHeaderFixture(const std::string& body) {
    const auto path = std::filesystem::temp_directory_path() /
                      "gnsspp_phase128_header_fixture.rnx";
    std::ofstream output(path);
    output << headerLine("     3.03           O                   G", "RINEX VERSION / TYPE")
           << body
           << headerLine("", "END OF HEADER");
    return path;
}

}  // namespace

TEST(Phase128GlonassProvenance, CanonicalDataKeepsFieldPositionsAndNormalizesEncodedFcn) {
    std::array<double, 15> data{};
    data.fill(1.0);
    data[10] = 252.0;  // 252 - 256 == -4, the source encoded form.
    const auto accepted = decodeCanonicalGlonassGeph(data);
    ASSERT_TRUE(accepted.accepted);
    EXPECT_EQ(accepted.frequency_channel, -4);
    EXPECT_EQ(accepted.reject_reason,
              GlonassCanonicalRecordRejectReason::None);

    data[9] = std::numeric_limits<double>::quiet_NaN();
    const auto nonfinite = decodeCanonicalGlonassGeph(data);
    EXPECT_FALSE(nonfinite.accepted);
    EXPECT_EQ(nonfinite.reject_reason,
              GlonassCanonicalRecordRejectReason::NonFiniteField);

    std::vector<double> short_record(14U, 1.0);
    const auto short_result = decodeCanonicalGlonassGeph(short_record);
    EXPECT_FALSE(short_result.accepted);
    EXPECT_EQ(short_result.reject_reason,
              GlonassCanonicalRecordRejectReason::FieldCount);
}

TEST(Phase128GlonassProvenance, FcnIntegralAndSignedRangeAreFailClosed) {
    std::array<double, 15> data{};
    data.fill(1.0);
    data[10] = 128.0;  // Not an encoded signed value; outside [-7,6].
    EXPECT_EQ(decodeCanonicalGlonassGeph(data).reject_reason,
              GlonassCanonicalRecordRejectReason::FcnOutOfRange);
    data[10] = 252.5;
    EXPECT_EQ(decodeCanonicalGlonassGeph(data).reject_reason,
              GlonassCanonicalRecordRejectReason::FcnNonIntegral);
    data[10] = -8.0;
    EXPECT_EQ(decodeCanonicalGlonassGeph(data).reject_reason,
              GlonassCanonicalRecordRejectReason::FcnOutOfRange);
}

TEST(Phase128GlonassProvenance, HeaderAbsentAndValidEmptyContinueToBroadcastGeph) {
    const auto navigation = navigationWith({ephemerisAt(-4)});
    const auto absent = resolve(kSatellite, kQueryTime, navigation,
                                HeaderStatus::Absent, {});
    ASSERT_TRUE(absent.accepted);
    EXPECT_EQ(absent.channel, -4);
    EXPECT_EQ(absent.source, phase127_glonass::ChannelSource::BroadcastEphemeris);

    const auto empty = resolve(kSatellite, kQueryTime, navigation,
                               HeaderStatus::ValidEmpty, {});
    ASSERT_TRUE(empty.accepted);
    EXPECT_EQ(empty.channel, -4);
}

TEST(Phase128GlonassProvenance, MalformedHeaderAndCanonicalRecordFailPerRecord) {
    const auto navigation = navigationWith({ephemerisAt(-4)});
    const auto malformed = resolve(kSatellite, kQueryTime, navigation,
                                   HeaderStatus::Malformed, {});
    EXPECT_FALSE(malformed.accepted);
    EXPECT_EQ(malformed.diagnostics.failure_code, "header-fcn-malformed");

    const auto invalid_navigation = navigationWith({ephemerisAt(-4, false)});
    const auto invalid = resolve(kSatellite, kQueryTime, invalid_navigation,
                                 HeaderStatus::Absent, {});
    EXPECT_FALSE(invalid.accepted);
    EXPECT_EQ(invalid.diagnostics.failure_code, "geph-canonical-invalid");
}

TEST(Phase128GlonassProvenance, AnnotationIgnoresPhoneCarrierFrequencyMetadata) {
    const auto navigation = navigationWith({ephemerisAt(3)});
    Observation observation(kSatellite, SignalType::GLO_L1CA);
    observation.has_source_carrier_frequency_hz = true;
    observation.source_carrier_frequency_hz = 1.0e9;  // Never an FCN source.
    const auto result = resolveAndAnnotate(observation, kQueryTime, navigation,
                                           HeaderStatus::Absent, {});
    ASSERT_TRUE(result.accepted);
    EXPECT_TRUE(observation.has_glonass_frequency_channel);
    EXPECT_EQ(observation.glonass_frequency_channel, 3);
    EXPECT_DOUBLE_EQ(observation.source_carrier_frequency_hz, 1.0e9);
}

TEST(Phase128GlonassProvenance, RINEXHeaderStatesAreDistinct) {
    const auto absent_path = writeHeaderFixture("");
    io::RINEXReader absent_reader;
    ASSERT_TRUE(absent_reader.open(absent_path.string()));
    io::RINEXReader::RINEXHeader absent_header;
    ASSERT_TRUE(absent_reader.readHeader(absent_header));
    EXPECT_EQ(absent_header.glonass_frequency_channel_header_status,
              GlonassFrequencyChannelHeaderStatus::Absent);
    absent_reader.close();
    std::filesystem::remove(absent_path);

    const auto empty_path = writeHeaderFixture(
        headerLine("", "GLONASS SLOT / FRQ #"));
    io::RINEXReader empty_reader;
    ASSERT_TRUE(empty_reader.open(empty_path.string()));
    io::RINEXReader::RINEXHeader empty_header;
    ASSERT_TRUE(empty_reader.readHeader(empty_header));
    EXPECT_EQ(empty_header.glonass_frequency_channel_header_status,
              GlonassFrequencyChannelHeaderStatus::ValidEmpty);
    empty_reader.close();
    std::filesystem::remove(empty_path);

    std::string entries(60U, ' ');
    entries.replace(4U, 6U, "R07 -4");
    const auto entries_path = writeHeaderFixture(
        headerLine(entries, "GLONASS SLOT / FRQ #"));
    io::RINEXReader entries_reader;
    ASSERT_TRUE(entries_reader.open(entries_path.string()));
    io::RINEXReader::RINEXHeader entries_header;
    ASSERT_TRUE(entries_reader.readHeader(entries_header));
    EXPECT_EQ(entries_header.glonass_frequency_channel_header_status,
              GlonassFrequencyChannelHeaderStatus::Entries);
    ASSERT_EQ(entries_header.glonass_frequency_channel_entries.size(), 1U);
    EXPECT_EQ(entries_header.glonass_frequency_channel_entries.front().channel, -4);
    entries_reader.close();
    std::filesystem::remove(entries_path);

    std::string malformed(60U, ' ');
    malformed[4] = 'X';
    const auto malformed_path = writeHeaderFixture(
        headerLine(malformed, "GLONASS SLOT / FRQ #"));
    io::RINEXReader malformed_reader;
    ASSERT_TRUE(malformed_reader.open(malformed_path.string()));
    io::RINEXReader::RINEXHeader malformed_header;
    ASSERT_TRUE(malformed_reader.readHeader(malformed_header));
    EXPECT_EQ(malformed_header.glonass_frequency_channel_header_status,
              GlonassFrequencyChannelHeaderStatus::Malformed);
    EXPECT_GT(malformed_header.glonass_frequency_channel_malformed_entries, 0U);
    malformed_reader.close();
    std::filesystem::remove(malformed_path);
}

TEST(Phase128GlonassProvenance, ConfigSelectorDefaultsOff) {
    fgo::Config config;
    EXPECT_FALSE(config.use_native_phase128_glonass_provenance_parser_admission);
}
