#include <gtest/gtest.h>
#include <libgnss++/io/rinex.hpp>
#include <libgnss++/io/rinex4.hpp>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>

using namespace libgnss;

namespace {

std::string rinexHeaderLine(const std::string& content, const std::string& label) {
    std::string line = content;
    if (line.size() < 60) {
        line.append(60 - line.size(), ' ');
    }
    line += label;
    return line + "\n";
}

std::string rinexObsField(const std::string& value,
                          char lli = ' ',
                          char signal_strength = ' ') {
    std::string field;
    if (value.size() < 14) {
        field.append(14 - value.size(), ' ');
    }
    field += value;
    field += lli;
    field += signal_strength;
    return field;
}

std::string rinex4GpsLnavBody() {
    // RINEX 4.02 Table A12-compatible GPS LNAV body.  The data-record
    // header is deliberately kept separate so tests exercise the new
    // dispatch boundary rather than a RINEX 3-style first-line heuristic.
    return R"(G04 2019 03 14 04 00 00 1.330170780420e-04 7.275957614183e-12 0.000000000000e+00
     9.800000000000e+01-1.718750000000e+00 4.639836124941e-09 2.148941747752e+00
    -1.881271600723e-07 3.355251392350e-04 8.245930075645e-06 5.153800453186e+03
     3.600000000000e+05-1.676380634308e-08 5.171400020311e-01 1.490116119385e-08
     9.601921900531e-01 2.187187500000e+02-1.736906885738e+00-8.044977962767e-09
    -2.932264997750e-10 1.000000000000e+00 2.044000000000e+03 0.000000000000e+00
     4.000000000000e+00 6.300000000000e+01-8.847564458847e-09 8.660000000000e+02
     3.553500000000e+05 4.000000000000e+00
)";
}

std::string rinex4GalileoBody(int data_source_code) {
    std::string body = rinex4GpsLnavBody();
    body.replace(0, 3, "E04");
    size_t line_start = 0;
    for (int line = 0; line < 5; ++line) {
        line_start = body.find('\n', line_start) + 1;
    }
    std::ostringstream source_stream;
    source_stream << ' ' << std::scientific << std::setprecision(12)
                  << static_cast<double>(data_source_code);
    const std::string source_field = source_stream.str();
    body.replace(line_start + 23, source_field.size(), source_field);
    return body;
}

std::vector<std::string> rinex4GlonassCdmaBody(bool blank_delay = false) {
    std::vector<std::string> body = {
        "R26 2024 02 03 00 15 00-1.605716170161e-05 1.652011860642e-12-2.081668171172e-17",
        "     1.812154053020e+04-2.071979139000e+00 5.729816621169e-10 1.000000000000e+00",
        "    -2.325615360260e+03 1.285475494340e+00-1.047737896442e-09 1.000000000000e+00",
        "     1.781341854668e+04 2.280306640081e+00-9.458744898438e-10 0.000000000000e+00",
        "     2.000000000000e+00 1.000000000000e+01 7.500000000000e-01 7.500000000000e-01",
        "     0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00",
        "     0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00",
        "     0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00",
        "     1.500000000000e+01 5.000000000000e+00                    5.184000000000e+05",
    };
    if (blank_delay) {
        body[3].resize(61);
    }
    return body;
}

void writeRinex4GlonassCdmaBody(std::ofstream& file,
                                bool blank_delay = false) {
    for (const auto& line : rinex4GlonassCdmaBody(blank_delay)) {
        file << line << '\n';
    }
}

std::vector<std::string> rinex4GlonassFdmaBody() {
    std::istringstream stream(rinex4GpsLnavBody());
    std::vector<std::string> body;
    std::string line;
    for (int i = 0; i < 4 && std::getline(stream, line); ++i) {
        if (i == 0) {
            line.replace(0, 3, "R26");
        }
        body.push_back(line);
    }
    return body;
}

void writeRinex4GlonassFdmaBody(std::ofstream& file) {
    for (const auto& line : rinex4GlonassFdmaBody()) {
        file << line << '\n';
    }
}

std::string rinex4SystemNumber(double value) {
    std::ostringstream stream;
    stream << std::scientific << std::setprecision(12) << std::setw(19)
           << value;
    return stream.str();
}

std::string rinex4SystemDate(int year, int month, int day,
                             int hour, int minute, int second) {
    std::ostringstream stream;
    stream << "    " << std::setfill('0') << std::setw(4) << year << ' '
           << std::setw(2) << month << ' ' << std::setw(2) << day << ' '
           << std::setw(2) << hour << ' ' << std::setw(2) << minute << ' '
           << std::setw(2) << second;
    return stream.str();
}

std::string rinex4SystemDateNumbers(const std::vector<double>& values) {
    std::string line = rinex4SystemDate(2024, 2, 3, 4, 5, 6);
    line += ' ';
    for (const double value : values) {
        line += rinex4SystemNumber(value);
    }
    return line;
}

std::string rinex4SystemNumbers(const std::vector<double>& values) {
    std::string line = "    ";
    for (const double value : values) {
        line += rinex4SystemNumber(value);
    }
    return line;
}

std::string rinex4StoEpochLine(const std::string& correction_type,
                               const std::string& sbas_id = {},
                               const std::string& utc_id = {}) {
    std::string line = rinex4SystemDate(2024, 2, 3, 4, 5, 6);
    line += ' ';
    line += correction_type;
    line.resize(24 + 18, ' ');
    line += ' ';
    line += sbas_id;
    line.resize(24 + 2 * 19 - 1, ' ');
    line += ' ';
    line += utc_id;
    line.resize(24 + 3 * 19 - 1, ' ');
    return line;
}

std::string rinex4EopBodyLine1(bool malformed_spare) {
    std::string line = "    ";
    line += malformed_spare ? std::string(19, 'X') : std::string(19, ' ');
    line += rinex4SystemNumber(0.4);
    line += rinex4SystemNumber(0.5);
    line += rinex4SystemNumber(0.6);
    return line;
}

Ephemeris makeGpsEphemeris() {
    Ephemeris eph;
    eph.satellite = SatelliteId(GNSSSystem::GPS, 3);
    eph.toc = GNSSTime(2200, 345600.0);
    eph.toe = GNSSTime(2200, 345600.0);
    eph.tof = GNSSTime(2200, 345570.0);
    eph.toes = 345600.0;
    eph.af0 = 1.25e-4;
    eph.af1 = -2.0e-12;
    eph.af2 = 0.0;
    eph.iode = 12;
    eph.crs = 88.0;
    eph.delta_n = 4.3e-9;
    eph.m0 = 0.12;
    eph.cuc = 1.2e-6;
    eph.e = 0.01;
    eph.cus = -2.2e-6;
    eph.sqrt_a = 5153.7954775;
    eph.cic = 3.1e-8;
    eph.omega0 = 1.1;
    eph.cis = -4.1e-8;
    eph.i0 = 0.94;
    eph.crc = 250.0;
    eph.omega = 0.52;
    eph.omega_dot = -8.1e-9;
    eph.idot = 1.1e-10;
    eph.i_dot = eph.idot;
    eph.week = 2200;
    eph.sv_accuracy = 2.4;
    eph.sv_health = 0.0;
    eph.health = 0;
    eph.tgd = -2.3e-8;
    eph.iodc = 25;
    eph.valid = true;
    return eph;
}

Ephemeris makeGlonassEphemeris() {
    Ephemeris eph;
    eph.satellite = SatelliteId(GNSSSystem::GLONASS, 7);
    eph.toc = GNSSTime(2200, 345600.0);
    eph.toe = GNSSTime(2200, 345600.0);
    eph.tof = GNSSTime(2200, 345570.0);
    eph.week = 2200;
    eph.glonass_taun = -1.8e-4;
    eph.glonass_gamn = 2.5e-9;
    eph.glonass_position = Vector3d(1.91e7, -1.34e7, 2.12e7);
    eph.glonass_velocity = Vector3d(-1200.0, 1800.0, 900.0);
    eph.glonass_acceleration = Vector3d(0.15, -0.12, 0.09);
    eph.glonass_frequency_channel = -4;
    eph.glonass_age = 3;
    eph.health = 0;
    eph.valid = true;
    return eph;
}

Ephemeris makeBeiDouEphemeris() {
    Ephemeris eph;
    eph.satellite = SatelliteId(GNSSSystem::BeiDou, 12);
    eph.week = 844;
    eph.toc = GNSSTime(2200, 345614.0);
    eph.toe = GNSSTime(2200, 345614.0);
    eph.tof = GNSSTime(2200, 345584.0);
    eph.toes = 345600.0;
    eph.af0 = 2.5e-4;
    eph.af1 = -4.0e-12;
    eph.af2 = 0.0;
    eph.iode = 17;
    eph.iodc = 9;
    eph.crs = 64.0;
    eph.delta_n = 5.2e-9;
    eph.m0 = 0.24;
    eph.cuc = 1.5e-6;
    eph.e = 0.012;
    eph.cus = -1.8e-6;
    eph.sqrt_a = 5282.6256;
    eph.cic = 2.5e-8;
    eph.omega0 = 1.25;
    eph.cis = -3.2e-8;
    eph.i0 = 0.97;
    eph.crc = 180.0;
    eph.omega = 0.61;
    eph.omega_dot = -7.5e-9;
    eph.idot = 9.0e-11;
    eph.i_dot = eph.idot;
    eph.sv_accuracy = 3.0;
    eph.sv_health = 0.0;
    eph.health = 0;
    eph.tgd = -1.2e-8;
    eph.tgd_secondary = 2.3e-8;
    eph.valid = true;
    return eph;
}

Ephemeris makeGalileoEphemeris() {
    Ephemeris eph;
    eph.satellite = SatelliteId(GNSSSystem::Galileo, 5);
    eph.week = 2200;
    eph.toc = GNSSTime(2200, 345600.0);
    eph.toe = GNSSTime(2200, 345600.0);
    eph.tof = GNSSTime(2200, 345570.0);
    eph.toes = 345600.0;
    eph.af0 = 1.7e-4;
    eph.af1 = -3.0e-12;
    eph.af2 = 0.0;
    eph.iode = 44;
    eph.iodc = 44;
    eph.crs = 120.0;
    eph.delta_n = 3.5e-9;
    eph.m0 = 0.37;
    eph.cuc = 1.0e-6;
    eph.e = 0.015;
    eph.cus = -1.5e-6;
    eph.sqrt_a = 5440.588203;
    eph.cic = 2.4e-8;
    eph.omega0 = 1.9;
    eph.cis = -3.1e-8;
    eph.i0 = 0.98;
    eph.crc = 210.0;
    eph.omega = 0.73;
    eph.omega_dot = -5.9e-9;
    eph.idot = 8.0e-11;
    eph.i_dot = eph.idot;
    eph.sv_accuracy = 3.0;
    eph.sv_health = 0.0;
    eph.health = 0;
    eph.tgd = -4.5e-9;
    eph.tgd_secondary = 1.1e-8;
    eph.valid = true;
    return eph;
}

}  // namespace

TEST(RINEXReaderTest, ParsesTimeOfFirstObsHeader) {
    const auto temp_path = std::filesystem::temp_directory_path() / "libgnss_rinex_first_obs_test.obs";
    std::filesystem::remove(temp_path);

    {
        std::ofstream file(temp_path);
        ASSERT_TRUE(file.is_open());
        file << rinexHeaderLine("     3.04           OBSERVATION DATA    G                   ",
                                "RINEX VERSION / TYPE");
        file << rinexHeaderLine("  2024    8   22    3   14  15.5000000     GPS         ",
                                "TIME OF FIRST OBS");
        file << rinexHeaderLine("", "END OF HEADER");
    }

    io::RINEXReader reader;
    ASSERT_TRUE(reader.open(temp_path.string()));

    io::RINEXReader::RINEXHeader header;
    ASSERT_TRUE(reader.readHeader(header));

    EXPECT_EQ(header.first_obs.week, 2328);
    EXPECT_NEAR(header.first_obs.tow, 357255.5, 1e-9);

    reader.close();
    std::filesystem::remove(temp_path);
}

TEST(RINEXReaderTest, ParsesRinex4EpochPrecisionClockAndGpsQzssRows) {
    const auto temp_path =
        std::filesystem::temp_directory_path() / "libgnss_rinex4_observation_dispatch.obs";
    std::filesystem::remove(temp_path);

    {
        std::ofstream file(temp_path);
        ASSERT_TRUE(file.is_open());
        file << rinexHeaderLine(
            "     4.02           OBSERVATION DATA    M",
            "RINEX VERSION / TYPE");
        file << rinexHeaderLine(
            "G    4 C1C L1C C2W L2W",
            "SYS / # / OBS TYPES");
        file << rinexHeaderLine("", "END OF HEADER");
        std::string epoch = "> 2024 08 03 09 51 20.1234567  0  2";
        epoch.append(41 - epoch.size(), ' ');
        epoch += "-0.123456789012 12345\n";
        file << epoch;
        file << "G04" << rinexObsField("22011162.552")
             << rinexObsField("115669443.467")
             << rinexObsField("22022262.423")
             << rinexObsField("120222443.467") << "\n";
        file << "J01" << rinexObsField("22011163.552")
             << rinexObsField("115669444.467")
             << rinexObsField("22022263.423")
             << rinexObsField("120222444.467") << "\n";
    }

    io::RINEXReader reader;
    ASSERT_TRUE(reader.open(temp_path.string()));
    io::RINEXReader::RINEXHeader header;
    ASSERT_TRUE(reader.readHeader(header));
    EXPECT_DOUBLE_EQ(header.version, 4.02);
    EXPECT_TRUE(reader.isRinex4());

    ObservationData epoch;
    ASSERT_TRUE(reader.readObservationEpoch(epoch));
    EXPECT_NEAR(epoch.receiver_clock_bias, -0.123456789012, 1e-15);
    EXPECT_EQ(epoch.time.week, 2325);
    EXPECT_NEAR(epoch.time.tow, 553880.1234567123, 1e-9);
    ASSERT_EQ(epoch.observations.size(), 4U);
    EXPECT_EQ(epoch.observations[0].satellite, SatelliteId(GNSSSystem::GPS, 4));
    EXPECT_EQ(epoch.observations[2].satellite, SatelliteId(GNSSSystem::QZSS, 1));
    const auto* gps_l1 = epoch.getObservation(
        SatelliteId(GNSSSystem::GPS, 4), SignalType::GPS_L1CA);
    ASSERT_NE(gps_l1, nullptr);
    EXPECT_TRUE(gps_l1->has_pseudorange);

    reader.close();
    std::filesystem::remove(temp_path);
}

TEST(RINEXReaderTest, AccumulatesRinex4ObservationTypeContinuation) {
    const auto temp_path =
        std::filesystem::temp_directory_path() / "libgnss_rinex4_obs_types_continuation.obs";
    std::filesystem::remove(temp_path);
    {
        std::ofstream file(temp_path);
        ASSERT_TRUE(file.is_open());
        file << rinexHeaderLine(
            "     4.02           OBSERVATION DATA    M", "RINEX VERSION / TYPE");
        file << rinexHeaderLine(
            "G   14 C1C L1C C2W L2W C5Q L5Q D5Q S5Q C1L L1L D1L S1L C2L",
            "SYS / # / OBS TYPES");
        file << rinexHeaderLine("       S2L", "SYS / # / OBS TYPES");
        file << rinexHeaderLine("", "END OF HEADER");
        file << "> 2024 08 03 09 51 20.0000000  0  2\n";
        const std::vector<std::string> values = {
            "22011162.552", "115669443.467", "22022262.423", "120222443.467",
            "22033362.552", "115669543.467", "22044462.423", "120222543.467",
            "22055562.552", "115669643.467", "22066662.423", "120222643.467",
            "22077762.552", "115669743.467"};
        std::string gps_row = "G04";
        std::string qzss_row = "J01";
        for (const auto& value : values) {
            gps_row += rinexObsField(value);
            qzss_row += rinexObsField(value);
        }
        file << gps_row << "\n" << qzss_row << "\n";
    }

    io::RINEXReader reader;
    ASSERT_TRUE(reader.open(temp_path.string()));
    io::RINEXReader::RINEXHeader header;
    ASSERT_TRUE(reader.readHeader(header));
    ASSERT_EQ(header.system_obs_types['G'].size(), 14U);
    EXPECT_EQ(header.system_obs_types['G'].back(), "S2L");
    EXPECT_EQ(header.observation_types.size(), 14U);
    ObservationData epoch;
    ASSERT_TRUE(reader.readObservationEpoch(epoch));
    ASSERT_EQ(epoch.observations.size(), 4U);
    EXPECT_EQ(epoch.observations[0].satellite, SatelliteId(GNSSSystem::GPS, 4));
    EXPECT_EQ(epoch.observations[2].satellite, SatelliteId(GNSSSystem::QZSS, 1));
    reader.close();
    std::filesystem::remove(temp_path);
}

TEST(RINEXReaderTest, SkipsRinex4EventAndCycleSlipRecordsBeforeNextEpoch) {
    const auto temp_path =
        std::filesystem::temp_directory_path() / "libgnss_rinex4_events.obs";
    std::filesystem::remove(temp_path);
    {
        std::ofstream file(temp_path);
        ASSERT_TRUE(file.is_open());
        file << rinexHeaderLine(
            "     4.02           OBSERVATION DATA    M", "RINEX VERSION / TYPE");
        file << rinexHeaderLine("G    4 C1C L1C C2W L2W", "SYS / # / OBS TYPES");
        file << rinexHeaderLine("", "END OF HEADER");
        file << "> 2024 08 03 09 51 20.0000000  0  0\n";
        file << ">                              2 1\n";
        file << rinexHeaderLine("event comment", "COMMENT");
        file << ">                              6 1\n";
        file << "G04" << rinexObsField("99999999.000")
             << rinexObsField("1.000") << rinexObsField("2.000")
             << rinexObsField("3.000") << "\n";
        file << "> 2024 08 03 09 51 21.0000000  0  1\n";
        file << "G04" << rinexObsField("22011162.552")
             << rinexObsField("115669443.467")
             << rinexObsField("22022262.423")
             << rinexObsField("120222443.467") << "\n";
    }

    io::RINEXReader reader;
    ASSERT_TRUE(reader.open(temp_path.string()));
    io::RINEXReader::RINEXHeader header;
    ASSERT_TRUE(reader.readHeader(header));
    ObservationData empty_epoch;
    ASSERT_TRUE(reader.readObservationEpoch(empty_epoch));
    EXPECT_TRUE(empty_epoch.isEmpty());
    ObservationData epoch;
    ASSERT_TRUE(reader.readObservationEpoch(epoch));
    EXPECT_EQ(epoch.time.week, 2325);
    EXPECT_NEAR(epoch.time.tow, 553881.0, 1e-9);
    ASSERT_EQ(epoch.observations.size(), 2U);
    EXPECT_EQ(epoch.observations.front().satellite, SatelliteId(GNSSSystem::GPS, 4));
    reader.close();
    std::filesystem::remove(temp_path);
}

TEST(RINEXReaderTest, FeedsRinex4HeaderEventRecordsIntoHeaderParser) {
    const auto temp_path =
        std::filesystem::temp_directory_path() / "libgnss_rinex4_header_event.obs";
    std::filesystem::remove(temp_path);
    {
        std::ofstream file(temp_path);
        ASSERT_TRUE(file.is_open());
        file << rinexHeaderLine(
            "     4.02           OBSERVATION DATA    M", "RINEX VERSION / TYPE");
        file << rinexHeaderLine("G    4 C1C L1C C2W L2W", "SYS / # / OBS TYPES");
        file << rinexHeaderLine("", "END OF HEADER");
        file << ">                              4 1\n";
        file << rinexHeaderLine("G    4 C1C L1C C5Q L5Q", "SYS / # / OBS TYPES");
        file << "> 2024 08 03 09 51 22.0000000  0  1\n";
        file << "G04" << rinexObsField("22011162.552")
             << rinexObsField("115669443.467")
             << rinexObsField("22022262.423")
             << rinexObsField("120222443.467") << "\n";
    }

    io::RINEXReader reader;
    ASSERT_TRUE(reader.open(temp_path.string()));
    io::RINEXReader::RINEXHeader header;
    ASSERT_TRUE(reader.readHeader(header));
    ObservationData epoch;
    ASSERT_TRUE(reader.readObservationEpoch(epoch));
    ASSERT_EQ(epoch.observations.size(), 2U);
    reader.close();
    std::filesystem::remove(temp_path);
}

TEST(RINEXReaderTest, RejectsTruncatedRinex4ObservationEpoch) {
    const auto temp_path =
        std::filesystem::temp_directory_path() / "libgnss_rinex4_truncated.obs";
    std::filesystem::remove(temp_path);
    {
        std::ofstream file(temp_path);
        ASSERT_TRUE(file.is_open());
        file << rinexHeaderLine(
            "     4.02           OBSERVATION DATA    M", "RINEX VERSION / TYPE");
        file << rinexHeaderLine("G    4 C1C L1C C2W L2W", "SYS / # / OBS TYPES");
        file << rinexHeaderLine("", "END OF HEADER");
        file << "> 2024 08 03 09 51 23.0000000  0  2\n";
        file << "G04" << rinexObsField("22011162.552")
             << rinexObsField("115669443.467")
             << rinexObsField("22022262.423")
             << rinexObsField("120222443.467") << "\n";
    }

    io::RINEXReader reader;
    ASSERT_TRUE(reader.open(temp_path.string()));
    io::RINEXReader::RINEXHeader header;
    ASSERT_TRUE(reader.readHeader(header));
    ObservationData epoch;
    EXPECT_FALSE(reader.readObservationEpoch(epoch));
    EXPECT_TRUE(epoch.isEmpty());
    reader.close();
    std::filesystem::remove(temp_path);
}

TEST(RINEXReaderTest, RejectsMalformedRinex4EpochTokenWidthsAndCounts) {
    io::rinex4::ObservationEpochHeader epoch;
    EXPECT_FALSE(io::rinex4::parseObservationEpochHeader(
        "> 0001 08 03 09 51 23.0000000  0  1", epoch));
    EXPECT_FALSE(io::rinex4::parseObservationEpochHeader(
        "> 2024 8 03 09 51 23.0000000  0  1", epoch));
    EXPECT_FALSE(io::rinex4::parseObservationEpochHeader(
        "> 2024 08 03 09 51 23.0000000  0  1000", epoch));
    EXPECT_FALSE(io::rinex4::parseObservationEpochHeader(
        ">                              2 1000", epoch));
}

TEST(RINEXReaderTest, RejectsCompactRinexSuffixesButNotSimilarNames) {
    const auto base = std::filesystem::temp_directory_path();
    const auto crx = base / "libgnss_rinex4_policy.CRX";
    const auto crx_gz = base / "libgnss_rinex4_policy.CRX.GZ";
    const auto similar = base / "libgnss_rinex4_policy.crx.tmp";
    std::filesystem::remove(crx);
    std::filesystem::remove(crx_gz);
    std::filesystem::remove(similar);
    {
        std::ofstream(crx) << "not decoded\n";
        std::ofstream(crx_gz) << "not decoded\n";
        std::ofstream(similar) << "not compact\n";
    }

    io::RINEXReader reader;
    testing::internal::CaptureStderr();
    EXPECT_FALSE(reader.open(crx.string()));
    EXPECT_FALSE(reader.open(crx_gz.string()));
    const std::string diagnostic = testing::internal::GetCapturedStderr();
    EXPECT_NE(diagnostic.find("CompactRINEX input is not supported natively"),
              std::string::npos);
    EXPECT_TRUE(reader.open(similar.string()));
    reader.close();
    std::filesystem::remove(crx);
    std::filesystem::remove(crx_gz);
    std::filesystem::remove(similar);
}

TEST(RINEXReaderTest, ParsesRinex4NavigationMessageTypesAndRejectsExtraMetadata) {
    io::rinex4::NavigationRecordHeader header;
    ASSERT_TRUE(io::rinex4::parseNavigationRecordHeader(
        "> EPH E04 FNAV", header));
    EXPECT_EQ(header.navigation_message_type, NavigationMessageType::FNAV);
    EXPECT_STREQ(io::rinex4::navigationMessageTypeName(
                     header.navigation_message_type), "FNAV");
    EXPECT_EQ(io::rinex4::navigationMessageTypeFromRinexToken("inav"),
              NavigationMessageType::Unknown);
    EXPECT_FALSE(io::rinex4::parseNavigationRecordHeader(
        "> EPH E04 inav", header));
    EXPECT_FALSE(io::rinex4::parseNavigationRecordHeader(
        "> EPH E04 FNAV SUBTYPE", header));
    EXPECT_FALSE(io::rinex4::parseNavigationRecordHeader(
        "> EPH E04 FNAV EXTRA", header));
    EXPECT_TRUE(io::rinex4::parseNavigationRecordHeader(
        "> STO R FDMA TEST", header));
    EXPECT_FALSE(io::rinex4::parseNavigationRecordHeader(
        "> STO R FDMA TEST EXTRA", header));
}

TEST(RINEXReaderTest, DispatchesRinex4LnavAndSkipsUnsupportedRecordBoundary) {
    const auto temp_path =
        std::filesystem::temp_directory_path() / "libgnss_rinex4_navigation_dispatch.nav";
    std::filesystem::remove(temp_path);

    {
        std::ofstream file(temp_path);
        ASSERT_TRUE(file.is_open());
        file << rinexHeaderLine(
            "     4.02           NAVIGATION DATA     M",
            "RINEX VERSION / TYPE");
        file << rinexHeaderLine("", "END OF HEADER");
        file << "> STO R FDMA\n";
        file << "this system record is intentionally unsupported\n";
        file << "> EPH I05 LNAV\n";
        file << "this syntactically valid but unsupported EPH is skipped\n";
        file << "> EPH I05 L1NV\n";
        file << "this NavIC L1NV body is intentionally unsupported\n";
        file << "> EPH R01 L1OC\n";
        file << "this GLONASS L1OC body is intentionally unsupported\n";
        file << "> EPH R01 L3OC\n";
        file << "this GLONASS L3OC body is intentionally unsupported\n";
        file << "> EPH G04 LNAV\n";
        file << rinex4GpsLnavBody();
        std::string bds_d1_body = rinex4GpsLnavBody();
        bds_d1_body.replace(0, 3, "C01");
        file << "> EPH C01 D1\n";
        file << bds_d1_body;
        file << "> EPH G04 CNAV\n";
        file << "this EPH message type is deliberately unsupported\n";
    }

    io::RINEXReader reader;
    ASSERT_TRUE(reader.open(temp_path.string()));
    io::RINEXReader::RINEXHeader header;
    ASSERT_TRUE(reader.readHeader(header));
    EXPECT_TRUE(reader.isRinex4());

    NavigationData nav_data;
    testing::internal::CaptureStderr();
    ASSERT_TRUE(reader.readNavigationData(nav_data));
    const std::string diagnostic = testing::internal::GetCapturedStderr();
    EXPECT_NE(diagnostic.find("Skipping malformed RINEX 4 STO body"),
              std::string::npos);
    EXPECT_TRUE(reader.rinex4SystemData().system_time_offsets.empty());
    EXPECT_NE(diagnostic.find("Skipping unsupported RINEX 4 EPH I05 LNAV"),
              std::string::npos);
    EXPECT_NE(diagnostic.find("Skipping unsupported RINEX 4 EPH I05 L1NV"),
              std::string::npos);
    EXPECT_NE(diagnostic.find("Skipping malformed RINEX 4 GLONASS L1OC body"),
              std::string::npos);
    EXPECT_NE(diagnostic.find("Skipping malformed RINEX 4 GLONASS L3OC body"),
              std::string::npos);
    EXPECT_NE(diagnostic.find("Skipping unsupported RINEX 4 EPH G04 CNAV"),
              std::string::npos);

    const auto it = nav_data.ephemeris_data.find(SatelliteId(GNSSSystem::GPS, 4));
    ASSERT_NE(it, nav_data.ephemeris_data.end());
    ASSERT_EQ(it->second.size(), 1U);
    EXPECT_EQ(it->second.front().satellite, SatelliteId(GNSSSystem::GPS, 4));
    EXPECT_EQ(it->second.front().navigation_message_type,
              NavigationMessageType::LNAV);
    EXPECT_EQ(it->second.front().week, 2044);
    EXPECT_NEAR(it->second.front().toes, 360000.0, 1e-6);

    const auto bds_it = nav_data.ephemeris_data.find(
        SatelliteId(GNSSSystem::BeiDou, 1));
    ASSERT_NE(bds_it, nav_data.ephemeris_data.end());
    ASSERT_EQ(bds_it->second.size(), 1U);
    EXPECT_EQ(bds_it->second.front().navigation_message_type,
              NavigationMessageType::D1);

    reader.close();
    std::filesystem::remove(temp_path);
}

TEST(RINEXReaderTest, ParsesRinex4GlonassCdmaAndRecoversAtBoundaries) {
    const auto temp_path =
        std::filesystem::temp_directory_path() / "libgnss_rinex4_glonass_cdma.nav";
    std::filesystem::remove(temp_path);
    {
        std::ofstream file(temp_path);
        ASSERT_TRUE(file.is_open());
        file << rinexHeaderLine(
            "     4.02           NAVIGATION DATA     M",
            "RINEX VERSION / TYPE");
        file << rinexHeaderLine("", "END OF HEADER");

        file << "> EPH R26 L1OC\n";
        writeRinex4GlonassCdmaBody(file);

        file << "> EPH R26 FDMA\n";
        writeRinex4GlonassFdmaBody(file);

        file << "> EPH R26 L1OC\n";
        const auto truncated = rinex4GlonassCdmaBody();
        for (size_t i = 0; i + 1 < truncated.size(); ++i) {
            file << truncated[i] << '\n';
        }

        file << "> EPH R26 L3OC\n";
        writeRinex4GlonassCdmaBody(file, true);

        file << "> EPH R26 L3OC\n";
        auto bad_spare = rinex4GlonassCdmaBody();
        bad_spare[8].replace(42, 19, std::string(19, 'X'));
        for (const auto& line : bad_spare) {
            file << line << '\n';
        }

        file << "> EPH G04 LNAV\n";
        file << rinex4GpsLnavBody();
    }

    io::RINEXReader reader;
    ASSERT_TRUE(reader.open(temp_path.string()));
    io::RINEXReader::RINEXHeader header;
    ASSERT_TRUE(reader.readHeader(header));
    NavigationData nav_data;
    testing::internal::CaptureStderr();
    ASSERT_TRUE(reader.readNavigationData(nav_data));
    const std::string diagnostic = testing::internal::GetCapturedStderr();
    EXPECT_NE(diagnostic.find("Skipping malformed RINEX 4 GLONASS L1OC body"),
              std::string::npos);
    EXPECT_NE(diagnostic.find("Skipping malformed RINEX 4 GLONASS L3OC body"),
              std::string::npos);

    const auto glonass_it = nav_data.ephemeris_data.find(
        SatelliteId(GNSSSystem::GLONASS, 26));
    ASSERT_NE(glonass_it, nav_data.ephemeris_data.end());
    ASSERT_EQ(glonass_it->second.size(), 3U);
    const Ephemeris* l1oc = nullptr;
    const Ephemeris* l3oc = nullptr;
    const Ephemeris* fdma = nullptr;
    for (const auto& eph : glonass_it->second) {
        if (eph.navigation_message_type == NavigationMessageType::L1OC) {
            l1oc = &eph;
        } else if (eph.navigation_message_type == NavigationMessageType::L3OC) {
            l3oc = &eph;
        } else if (eph.navigation_message_type == NavigationMessageType::FDMA) {
            fdma = &eph;
        }
    }
    ASSERT_NE(l1oc, nullptr);
    ASSERT_NE(l3oc, nullptr);
    ASSERT_NE(fdma, nullptr);
    EXPECT_FALSE(fdma->glonass_cdma_data.has_value());
    ASSERT_TRUE(l1oc->glonass_cdma_data.has_value());
    ASSERT_TRUE(l3oc->glonass_cdma_data.has_value());
    EXPECT_NEAR(l1oc->glonass_taun, 1.605716170161e-05, 1e-18);
    EXPECT_NEAR(l1oc->glonass_position.x(), 1.812154053020e+07, 1e-6);
    EXPECT_NEAR(l1oc->glonass_velocity.y(), 1.285475494340e+03, 1e-9);
    EXPECT_EQ(l1oc->health, 1);
    EXPECT_FALSE(l1oc->valid);
    EXPECT_EQ(l1oc->glonass_cdma_data->source_flags, 10);
    EXPECT_DOUBLE_EQ(l1oc->glonass_cdma_data->aode, 0.75);
    EXPECT_DOUBLE_EQ(l1oc->glonass_cdma_data->aodc, 0.75);
    ASSERT_TRUE(l1oc->glonass_cdma_data->tgd_l2ocp.has_value());
    EXPECT_DOUBLE_EQ(*l1oc->glonass_cdma_data->tgd_l2ocp, 0.0);
    EXPECT_FALSE(l3oc->glonass_cdma_data->isc_l3ocp.has_value());
    EXPECT_DOUBLE_EQ(l3oc->glonass_cdma_data->transmission_time_utc_week,
                     518400.0);
    EXPECT_EQ(l1oc->toc.week, l1oc->tof.week);
    EXPECT_NEAR(l1oc->tof.tow - l1oc->toc.tow, -900.0, 1e-6);

    const auto gps_it = nav_data.ephemeris_data.find(
        SatelliteId(GNSSSystem::GPS, 4));
    ASSERT_NE(gps_it, nav_data.ephemeris_data.end());
    ASSERT_EQ(gps_it->second.size(), 1U);
    reader.close();
    std::filesystem::remove(temp_path);
}

TEST(RINEXReaderTest, GlonassCdmaParserIsTransactionalAndStrict) {
    io::rinex4::NavigationRecordHeader header;
    ASSERT_TRUE(io::rinex4::parseNavigationRecordHeader(
        "> EPH R26 L1OC", header));

    const auto valid = rinex4GlonassCdmaBody();
    io::rinex4::GlonassCdmaEphemerisRecord parsed;
    ASSERT_TRUE(io::rinex4::parseGlonassCdmaEphemerisRecord(
        header, valid, parsed));
    EXPECT_EQ(parsed.source_flags, 10);
    EXPECT_DOUBLE_EQ(parsed.aode, 0.75);
    EXPECT_DOUBLE_EQ(parsed.aodc, 0.75);

    parsed.minus_tau_n = 42.0;
    auto truncated = valid;
    truncated.pop_back();
    EXPECT_FALSE(io::rinex4::parseGlonassCdmaEphemerisRecord(
        header, truncated, parsed));
    EXPECT_DOUBLE_EQ(parsed.minus_tau_n, 42.0);

    auto bad_spare = valid;
    bad_spare[8].replace(42, 19, std::string(19, 'X'));
    EXPECT_FALSE(io::rinex4::parseGlonassCdmaEphemerisRecord(
        header, bad_spare, parsed));
    EXPECT_DOUBLE_EQ(parsed.minus_tau_n, 42.0);

    auto bad_integer = valid;
    bad_integer[1].replace(61, 19, rinex4SystemNumber(2.0));
    EXPECT_FALSE(io::rinex4::parseGlonassCdmaEphemerisRecord(
        header, bad_integer, parsed));
    EXPECT_DOUBLE_EQ(parsed.minus_tau_n, 42.0);

    auto bad_prefix = valid;
    bad_prefix[1][0] = 'X';
    EXPECT_FALSE(io::rinex4::parseGlonassCdmaEphemerisRecord(
        header, bad_prefix, parsed));
    EXPECT_DOUBLE_EQ(parsed.minus_tau_n, 42.0);

    auto bad_numeric = valid;
    bad_numeric[1].replace(4, 19, std::string(16, ' ') + "NaN");
    EXPECT_FALSE(io::rinex4::parseGlonassCdmaEphemerisRecord(
        header, bad_numeric, parsed));
    EXPECT_DOUBLE_EQ(parsed.minus_tau_n, 42.0);

    auto extra_body = valid;
    extra_body.push_back(valid.back());
    EXPECT_FALSE(io::rinex4::parseGlonassCdmaEphemerisRecord(
        header, extra_body, parsed));
    EXPECT_DOUBLE_EQ(parsed.minus_tau_n, 42.0);

    auto shortened_optional = rinex4GlonassCdmaBody(true);
    io::rinex4::NavigationRecordHeader l3_header;
    ASSERT_TRUE(io::rinex4::parseNavigationRecordHeader(
        "> EPH R26 L3OC", l3_header));
    ASSERT_TRUE(io::rinex4::parseGlonassCdmaEphemerisRecord(
        l3_header, shortened_optional, parsed));
    EXPECT_FALSE(parsed.isc_l3ocp.has_value());
}

TEST(RINEXReaderTest, PreservesRinex4GalileoMessageProvenanceAndRejectsContradiction) {
    const auto temp_path =
        std::filesystem::temp_directory_path() / "libgnss_rinex4_galileo_metadata.nav";
    std::filesystem::remove(temp_path);
    {
        std::ofstream file(temp_path);
        ASSERT_TRUE(file.is_open());
        file << rinexHeaderLine(
            "     4.02           NAVIGATION DATA     M",
            "RINEX VERSION / TYPE");
        file << rinexHeaderLine("", "END OF HEADER");
        file << "> EPH E04 FNAV\n";
        file << rinex4GalileoBody((1 << 8) + (1 << 1));
        file << "> EPH E04 FNAV\n";
        file << rinex4GalileoBody((1 << 9) | 1);
        file << "> EPH E04 INAV\n";
        file << rinex4GalileoBody((1 << 9) | 1);
    }

    io::RINEXReader reader;
    ASSERT_TRUE(reader.open(temp_path.string()));
    io::RINEXReader::RINEXHeader header;
    ASSERT_TRUE(reader.readHeader(header));
    NavigationData nav_data;
    testing::internal::CaptureStderr();
    ASSERT_TRUE(reader.readNavigationData(nav_data));
    const std::string diagnostic = testing::internal::GetCapturedStderr();
    EXPECT_NE(diagnostic.find("body data-source clock bit contradicts header"),
              std::string::npos);

    const auto it = nav_data.ephemeris_data.find(
        SatelliteId(GNSSSystem::Galileo, 4));
    ASSERT_NE(it, nav_data.ephemeris_data.end());
    ASSERT_EQ(it->second.size(), 2U);
    bool found_fnav = false;
    bool found_inav = false;
    for (const auto& eph : it->second) {
        found_fnav = found_fnav ||
                     eph.navigation_message_type == NavigationMessageType::FNAV;
        found_inav = found_inav ||
                     eph.navigation_message_type == NavigationMessageType::INAV;
    }
    EXPECT_TRUE(found_fnav);
    EXPECT_TRUE(found_inav);
    reader.close();
    std::filesystem::remove(temp_path);
}

TEST(RINEXReaderTest, ParsesRinex4SystemRecordsAndSkipsInvalidBoundaries) {
    const auto temp_path =
        std::filesystem::temp_directory_path() / "libgnss_rinex4_system_records.nav";
    std::filesystem::remove(temp_path);
    {
        std::ofstream file(temp_path);
        ASSERT_TRUE(file.is_open());
        file << rinexHeaderLine(
            "     4.02           NAVIGATION DATA     M",
            "RINEX VERSION / TYPE");
        file << rinexHeaderLine("", "END OF HEADER");

        file << "> STO G24 CNVX\n";
        file << rinex4StoEpochLine("GPUT", "", "UTC(USNO)") << '\n';
        file << rinex4SystemNumbers({253224.0, 9.0e-10, -1.0e-14, 0.0}) << '\n';

        file << "> STO G24 CNVX BAD\n";
        file << rinex4StoEpochLine("GPUT", "", "UTC(USNO)") << '\n';
        file << rinex4SystemNumbers({1.0, 2.0, 3.0, 4.0}) << '\n';

        file << "> EOP G24 CNVX\n";
        file << rinex4SystemDateNumbers({0.2, -0.0002, 0.0}) << '\n';
        file << rinex4EopBodyLine1(false) << '\n';
        file << rinex4SystemNumbers({253248.0, -0.17, -0.00007, 0.0}) << '\n';

        file << "> EOP G24 CNVX\n";
        file << rinex4SystemDateNumbers({0.2, -0.0002, 0.0}) << '\n';
        file << rinex4EopBodyLine1(true) << '\n';
        file << rinex4SystemNumbers({253248.0, -0.17, -0.00007, 0.0}) << '\n';

        file << "> ION J01 CNVX WIDE\n";
        file << rinex4SystemDateNumbers({1.0, 2.0, 3.0}) << '\n';
        file << rinex4SystemNumbers({4.0, 5.0, 6.0, 7.0}) << '\n';
        file << rinex4SystemNumbers({8.0}) << '\n';

        file << "> ION J01 CNVX BAD\n";
        file << rinex4SystemDateNumbers({1.0, 2.0, 3.0}) << '\n';
        file << rinex4SystemNumbers({4.0, 5.0, 6.0, 7.0}) << '\n';
        file << rinex4SystemNumbers({8.0}) << '\n';

        file << "> ION E13 IFNV\n";
        file << rinex4SystemDateNumbers({54.5, 0.35, 0.013}) << '\n';
        file << rinex4SystemNumbers({0.0}) << '\n';

        file << "> ION C03 CNVX\n";
        file << rinex4SystemDateNumbers({1.0, 2.0, 3.0}) << '\n';
        file << rinex4SystemNumbers({4.0, 5.0, 6.0, 7.0}) << '\n';
        file << rinex4SystemNumbers({8.0, 9.0}) << '\n';

        file << "> ION C03 D1D2\n";
        file << rinex4SystemDateNumbers({10.0, 11.0, 12.0}) << '\n';
        file << rinex4SystemNumbers({13.0, 14.0, 15.0, 16.0}) << '\n';
        file << rinex4SystemNumbers({17.0}) << '\n';

        file << "> ION I10 L1NV KLOB\n";
        file << rinex4SystemDateNumbers({1.0}) << '\n';
        file << rinex4SystemNumbers({2.0, 3.0, 4.0, 5.0}) << '\n';
        file << rinex4SystemNumbers({6.0, 7.0, 8.0, 9.0}) << '\n';
        file << rinex4SystemNumbers({10.0, 11.0, 12.0, 13.0}) << '\n';

        file << "> ION I10 L1NV NEQN\n";
        file << rinex4SystemDateNumbers({2.0}) << '\n';
        for (int line = 0; line < 6; ++line) {
            file << rinex4SystemNumbers({1.0 + line, 2.0 + line,
                                          3.0 + line, 0.0}) << '\n';
        }

        file << "> ION R26 LXOC\n";
        file << rinex4SystemDateNumbers({1.0, 2.0, 3.0}) << '\n';

        file << "> ION G14 CNVX\n";
        file << rinex4SystemDateNumbers({21.0, 22.0, 23.0}) << '\n';
        file << rinex4SystemNumbers({24.0, 25.0, 26.0, 27.0}) << '\n';
        file << rinex4SystemNumbers({28.0}) << '\n';

        file << "> EPH G04 LNAV\n";
        file << rinex4GpsLnavBody();
    }

    io::RINEXReader reader;
    ASSERT_TRUE(reader.open(temp_path.string()));
    io::RINEXReader::RINEXHeader header;
    ASSERT_TRUE(reader.readHeader(header));
    NavigationData nav_data;
    testing::internal::CaptureStderr();
    ASSERT_TRUE(reader.readNavigationData(nav_data));
    const std::string diagnostic = testing::internal::GetCapturedStderr();
    EXPECT_NE(diagnostic.find("Skipping malformed RINEX 4 EOP body"),
              std::string::npos);
    EXPECT_NE(diagnostic.find("Skipping malformed or unsupported RINEX 4 ION body"),
              std::string::npos);

    const auto& system_data = reader.rinex4SystemData();
    ASSERT_EQ(system_data.system_time_offsets.size(), 1U);
    EXPECT_EQ(system_data.system_time_offsets.front().correction_type, "GPUT");
    EXPECT_EQ(system_data.system_time_offsets.front().utc_id, "UTC(USNO)");
    EXPECT_NEAR(system_data.system_time_offsets.front().transmission_time,
                253224.0, 1e-9);

    ASSERT_EQ(system_data.earth_orientation_parameters.size(), 1U);
    EXPECT_NEAR(system_data.earth_orientation_parameters.front().x_p, 0.2, 1e-12);
    EXPECT_NEAR(system_data.earth_orientation_parameters.front().delta_ut1,
                -0.17, 1e-12);

    ASSERT_EQ(system_data.ionosphere_records.size(), 8U);
    EXPECT_TRUE(std::holds_alternative<io::rinex4::KlobucharIonosphere>(
        system_data.ionosphere_records[0].payload));
    EXPECT_TRUE(std::holds_alternative<io::rinex4::NequickGIonosphere>(
        system_data.ionosphere_records[1].payload));
    EXPECT_TRUE(std::holds_alternative<io::rinex4::BdgimIonosphere>(
        system_data.ionosphere_records[2].payload));
    EXPECT_TRUE(std::holds_alternative<io::rinex4::NavicKlobucharIonosphere>(
        system_data.ionosphere_records[4].payload));
    EXPECT_TRUE(std::holds_alternative<io::rinex4::NavicNequickNIonosphere>(
        system_data.ionosphere_records[5].payload));
    EXPECT_TRUE(std::holds_alternative<io::rinex4::GlonassCdmaIonosphere>(
        system_data.ionosphere_records[6].payload));
    EXPECT_TRUE(std::holds_alternative<io::rinex4::KlobucharIonosphere>(
        system_data.ionosphere_records[3].payload));
    const auto& qzss_klob = std::get<io::rinex4::KlobucharIonosphere>(
        system_data.ionosphere_records[0].payload);
    EXPECT_DOUBLE_EQ(qzss_klob.alpha[0], 1.0);
    EXPECT_DOUBLE_EQ(qzss_klob.beta[3], 8.0);
    EXPECT_EQ(std::get<io::rinex4::NequickGIonosphere>(
                  system_data.ionosphere_records[1].payload).ai[0],
              54.5);
    EXPECT_DOUBLE_EQ(std::get<io::rinex4::BdgimIonosphere>(
                         system_data.ionosphere_records[2].payload).alpha[8],
                     9.0);
    EXPECT_DOUBLE_EQ(std::get<io::rinex4::KlobucharIonosphere>(
                         system_data.ionosphere_records[3].payload).alpha[0],
                     10.0);
    EXPECT_DOUBLE_EQ(std::get<io::rinex4::NavicKlobucharIonosphere>(
                         system_data.ionosphere_records[4].payload).issue_of_data,
                     1.0);
    EXPECT_EQ(std::get<io::rinex4::NavicNequickNIonosphere>(
                  system_data.ionosphere_records[5].payload).disturbance_flags[0],
              0);
    EXPECT_DOUBLE_EQ(std::get<io::rinex4::GlonassCdmaIonosphere>(
                         system_data.ionosphere_records[6].payload).c_f10_7,
                     2.0);
    EXPECT_TRUE(std::holds_alternative<io::rinex4::KlobucharIonosphere>(
        system_data.ionosphere_records[7].payload));
    EXPECT_DOUBLE_EQ(std::get<io::rinex4::KlobucharIonosphere>(
                         system_data.ionosphere_records[7].payload).alpha[0],
                     21.0);

    const auto eph_it = nav_data.ephemeris_data.find(
        SatelliteId(GNSSSystem::GPS, 4));
    ASSERT_NE(eph_it, nav_data.ephemeris_data.end());
    ASSERT_EQ(eph_it->second.size(), 1U);
    reader.close();
    std::filesystem::remove(temp_path);
}

TEST(RINEXReaderTest, SystemRecordParsersAreTransactionalAndStrict) {
    io::rinex4::NavigationRecordHeader sto_header;
    ASSERT_TRUE(io::rinex4::parseNavigationRecordHeader(
        "> STO G24 CNVX", sto_header));
    io::rinex4::SystemTimeOffsetRecord sto;
    sto.a0 = 42.0;
    const std::vector<std::string> malformed_sto = {
        rinex4StoEpochLine("GPUT", "", "UTC(USNO)"),
        rinex4SystemNumbers({1.0, 2.0, 3.0})};
    EXPECT_FALSE(io::rinex4::parseSystemTimeOffsetRecord(
        sto_header, malformed_sto, sto));
    EXPECT_DOUBLE_EQ(sto.a0, 42.0);

    std::string malformed_prefix_sto = rinex4SystemNumbers({1.0, 2.0, 3.0, 4.0});
    malformed_prefix_sto[0] = 'X';
    EXPECT_FALSE(io::rinex4::parseSystemTimeOffsetRecord(
        sto_header,
        {rinex4StoEpochLine("GPUT", "", "UTC(USNO)"), malformed_prefix_sto},
        sto));

    const std::vector<std::string> gps_code_for_glonass = {
        rinex4StoEpochLine("GPUT", "", "UTC(USNO)"),
        rinex4SystemNumbers({1.0, 2.0, 3.0, 4.0})};
    io::rinex4::NavigationRecordHeader glonass_sto_header;
    ASSERT_TRUE(io::rinex4::parseNavigationRecordHeader(
        "> STO R FDMA", glonass_sto_header));
    EXPECT_FALSE(io::rinex4::parseSystemTimeOffsetRecord(
        glonass_sto_header, gps_code_for_glonass, sto));
    const std::vector<std::string> valid_glonass_sto = {
        rinex4StoEpochLine("GLUT", "", "UTC(SU)"),
        rinex4SystemNumbers({1.0, 2.0, 3.0, 4.0})};
    EXPECT_TRUE(io::rinex4::parseSystemTimeOffsetRecord(
        glonass_sto_header, valid_glonass_sto, sto));

    io::rinex4::NavigationRecordHeader sbas_sto_header;
    ASSERT_TRUE(io::rinex4::parseNavigationRecordHeader(
        "> STO S SBAS", sbas_sto_header));
    const std::vector<std::string> sbas_sto_with_glonass_utc = {
        rinex4StoEpochLine("SBUT", "GAGAN", "UTC(SU)"),
        rinex4SystemNumbers({1.0, 2.0, 3.0, 4.0})};
    EXPECT_FALSE(io::rinex4::parseSystemTimeOffsetRecord(
        sbas_sto_header, sbas_sto_with_glonass_utc, sto));
    const std::vector<std::string> valid_sbas_sto = {
        rinex4StoEpochLine("SBUT", "GAGAN", "UTC(NICT)"),
        rinex4SystemNumbers({1.0, 2.0, 3.0, 4.0})};
    EXPECT_TRUE(io::rinex4::parseSystemTimeOffsetRecord(
        sbas_sto_header, valid_sbas_sto, sto));

    io::rinex4::NavigationRecordHeader legacy_sto_header;
    ASSERT_TRUE(io::rinex4::parseNavigationRecordHeader(
        "> STO E LEG", legacy_sto_header));
    std::string short_legacy_sto = rinex4StoEpochLine("GAGP");
    short_legacy_sto.erase(short_legacy_sto.find_last_not_of(' ') + 1);
    EXPECT_TRUE(io::rinex4::parseSystemTimeOffsetRecord(
        legacy_sto_header,
        {short_legacy_sto, rinex4SystemNumbers({1.0, 2.0, 3.0, 4.0})},
        sto));

    io::rinex4::EarthOrientationRecord eop;
    io::rinex4::NavigationRecordHeader eop_gps_lnav_header;
    ASSERT_TRUE(io::rinex4::parseNavigationRecordHeader(
        "> EOP G24 LNAV", eop_gps_lnav_header));
    EXPECT_FALSE(io::rinex4::parseEarthOrientationRecord(
        eop_gps_lnav_header,
        {rinex4SystemDateNumbers({1.0, 2.0, 3.0}),
         rinex4EopBodyLine1(false), rinex4SystemNumbers({4.0, 5.0, 6.0, 7.0})},
        eop));

    io::rinex4::NavigationRecordHeader eop_navic_lnav_header;
    ASSERT_TRUE(io::rinex4::parseNavigationRecordHeader(
        "> EOP I10 LNAV", eop_navic_lnav_header));
    EXPECT_TRUE(io::rinex4::parseEarthOrientationRecord(
        eop_navic_lnav_header,
        {rinex4SystemDateNumbers({1.0, 2.0, 3.0}),
         rinex4EopBodyLine1(false), rinex4SystemNumbers({4.0, 5.0, 6.0, 7.0})},
        eop));

    io::rinex4::NavigationRecordHeader eop_header;
    ASSERT_TRUE(io::rinex4::parseNavigationRecordHeader(
        "> EOP G24 CNVX", eop_header));
    eop.x_p = 17.0;
    const std::vector<std::string> malformed_eop = {
        rinex4SystemDateNumbers({1.0, 2.0, 3.0}),
        rinex4EopBodyLine1(true),
        rinex4SystemNumbers({4.0, 5.0, 6.0, 7.0})};
    EXPECT_FALSE(io::rinex4::parseEarthOrientationRecord(
        eop_header, malformed_eop, eop));
    EXPECT_DOUBLE_EQ(eop.x_p, 17.0);

    io::rinex4::NavigationRecordHeader ion_header;
    ASSERT_TRUE(io::rinex4::parseNavigationRecordHeader(
        "> ION J01 CNVX WIDE", ion_header));
    const std::vector<std::string> malformed_ion = {
        rinex4SystemDateNumbers({1.0, 2.0, 3.0}),
        rinex4SystemNumbers({4.0, 5.0, 6.0, 7.0}),
        "    malformed"};
    io::rinex4::IonosphereRecord ion;
    EXPECT_FALSE(io::rinex4::parseIonosphereRecord(
        ion_header, malformed_ion, ion));

    io::rinex4::NavigationRecordHeader invalid_flags_header;
    ASSERT_TRUE(io::rinex4::parseNavigationRecordHeader(
        "> ION E13 IFNV", invalid_flags_header));
    EXPECT_FALSE(io::rinex4::parseIonosphereRecord(
        invalid_flags_header,
        {rinex4SystemDateNumbers({1.0, 2.0, 3.0}),
         rinex4SystemNumbers({32.0})},
        ion));
    EXPECT_FALSE(io::rinex4::parseIonosphereRecord(
        invalid_flags_header,
        {rinex4SystemDateNumbers({1.0, 2.0, 3.0}),
         rinex4SystemNumbers({1.5})},
        ion));

    io::rinex4::NavigationRecordHeader invalid_idf_header;
    ASSERT_TRUE(io::rinex4::parseNavigationRecordHeader(
        "> ION I10 L1NV NEQN", invalid_idf_header));
    std::vector<std::string> invalid_idf_body = {
        rinex4SystemDateNumbers({1.0})};
    for (int line = 0; line < 6; ++line) {
        invalid_idf_body.push_back(rinex4SystemNumbers(
            {1.0 + line, 2.0 + line, 3.0 + line,
             line == 0 ? 1.5 : 0.0}));
    }
    EXPECT_FALSE(io::rinex4::parseIonosphereRecord(
        invalid_idf_header, invalid_idf_body, ion));

    std::vector<std::string> high_idf_body = {
        rinex4SystemDateNumbers({1.0})};
    for (int line = 0; line < 6; ++line) {
        high_idf_body.push_back(rinex4SystemNumbers(
            {1.0 + line, 2.0 + line, 3.0 + line,
             line == 0 ? 32.0 : 0.0}));
    }
    EXPECT_TRUE(io::rinex4::parseIonosphereRecord(
        invalid_idf_header, high_idf_body, ion));
}

TEST(RINEXReaderTest, SystemRecordOnlyNavigationFileReturnsSuccess) {
    const auto temp_path =
        std::filesystem::temp_directory_path() / "libgnss_rinex4_system_only.nav";
    std::filesystem::remove(temp_path);
    {
        std::ofstream file(temp_path);
        ASSERT_TRUE(file.is_open());
        file << rinexHeaderLine(
            "     4.02           NAVIGATION DATA     M",
            "RINEX VERSION / TYPE");
        file << rinexHeaderLine("", "END OF HEADER");
        file << "> STO G24 CNVX\n";
        file << rinex4StoEpochLine("GPUT", "", "UTC(USNO)") << '\n';
        file << rinex4SystemNumbers({1.0, 2.0, 3.0, 4.0}) << '\n';
    }
    io::RINEXReader reader;
    ASSERT_TRUE(reader.open(temp_path.string()));
    io::RINEXReader::RINEXHeader header;
    ASSERT_TRUE(reader.readHeader(header));
    NavigationData nav_data;
    ASSERT_TRUE(reader.readNavigationData(nav_data));
    EXPECT_TRUE(nav_data.ephemeris_data.empty());
    ASSERT_EQ(reader.rinex4SystemData().system_time_offsets.size(), 1U);
    reader.close();
    std::filesystem::remove(temp_path);
}

TEST(RINEXWriterTest, WritesGpsNavigationMessageReadableByReader) {
    const auto temp_path = std::filesystem::temp_directory_path() / "libgnss_rinex_writer_test.nav";
    std::filesystem::remove(temp_path);

    io::RINEXWriter writer;
    io::RINEXReader::RINEXHeader header;
    header.version = 3.04;
    header.file_type = io::RINEXReader::FileType::NAVIGATION;
    header.satellite_system = "M";
    ASSERT_TRUE(writer.createNavigationFile(temp_path.string(), header));

    const Ephemeris expected = makeGpsEphemeris();
    ASSERT_TRUE(writer.writeNavigationMessage(expected));
    writer.close();

    io::RINEXReader reader;
    ASSERT_TRUE(reader.open(temp_path.string()));

    NavigationData nav_data;
    ASSERT_TRUE(reader.readNavigationData(nav_data));
    const auto it = nav_data.ephemeris_data.find(expected.satellite);
    ASSERT_NE(it, nav_data.ephemeris_data.end());
    ASSERT_EQ(it->second.size(), 1U);

    const auto& actual = it->second.front();
    EXPECT_EQ(actual.satellite.system, GNSSSystem::GPS);
    EXPECT_EQ(actual.satellite.prn, 3);
    EXPECT_EQ(actual.week, expected.week);
    EXPECT_NEAR(actual.toe.tow, expected.toe.tow, 1e-3);
    EXPECT_NEAR(actual.toes, expected.toes, 1e-6);
    EXPECT_NEAR(actual.af0, expected.af0, 1e-12);
    EXPECT_NEAR(actual.af1, expected.af1, 1e-18);
    EXPECT_NEAR(actual.crs, expected.crs, 1e-9);
    EXPECT_NEAR(actual.delta_n, expected.delta_n, 1e-18);
    EXPECT_NEAR(actual.m0, expected.m0, 1e-12);
    EXPECT_NEAR(actual.sqrt_a, expected.sqrt_a, 1e-9);
    EXPECT_NEAR(actual.omega0, expected.omega0, 1e-12);
    EXPECT_NEAR(actual.i0, expected.i0, 1e-12);
    EXPECT_NEAR(actual.omega_dot, expected.omega_dot, 1e-18);
    EXPECT_NEAR(actual.tgd, expected.tgd, 1e-18);
    EXPECT_EQ(actual.iodc, expected.iodc);
    EXPECT_EQ(actual.navigation_message_type, NavigationMessageType::Unknown);

    reader.close();
    std::filesystem::remove(temp_path);
}

TEST(RINEXWriterTest, WritesGlonassNavigationMessageReadableByReader) {
    const auto temp_path = std::filesystem::temp_directory_path() / "libgnss_rinex_writer_test_glo.nav";
    std::filesystem::remove(temp_path);

    io::RINEXWriter writer;
    io::RINEXReader::RINEXHeader header;
    header.version = 3.04;
    header.file_type = io::RINEXReader::FileType::NAVIGATION;
    header.satellite_system = "M";
    ASSERT_TRUE(writer.createNavigationFile(temp_path.string(), header));

    const Ephemeris expected = makeGlonassEphemeris();
    ASSERT_TRUE(writer.writeNavigationMessage(expected));

    writer.close();

    io::RINEXReader reader;
    ASSERT_TRUE(reader.open(temp_path.string()));

    NavigationData nav_data;
    ASSERT_TRUE(reader.readNavigationData(nav_data));
    const auto it = nav_data.ephemeris_data.find(expected.satellite);
    ASSERT_NE(it, nav_data.ephemeris_data.end());
    ASSERT_EQ(it->second.size(), 1U);

    const auto& actual = it->second.front();
    EXPECT_EQ(actual.satellite.system, GNSSSystem::GLONASS);
    EXPECT_EQ(actual.satellite.prn, expected.satellite.prn);
    EXPECT_EQ(actual.week, expected.week);
    EXPECT_NEAR(actual.glonass_taun, expected.glonass_taun, 1e-12);
    EXPECT_NEAR(actual.glonass_gamn, expected.glonass_gamn, 1e-18);
    EXPECT_NEAR((actual.glonass_position - expected.glonass_position).norm(), 0.0, 1e-3);
    EXPECT_NEAR((actual.glonass_velocity - expected.glonass_velocity).norm(), 0.0, 1e-6);
    EXPECT_NEAR((actual.glonass_acceleration - expected.glonass_acceleration).norm(), 0.0, 1e-6);
    EXPECT_EQ(actual.glonass_frequency_channel, expected.glonass_frequency_channel);
    EXPECT_EQ(actual.glonass_age, expected.glonass_age);

    reader.close();
    std::filesystem::remove(temp_path);
}

TEST(RINEXWriterTest, WritesBeiDouNavigationMessageReadableByReader) {
    const auto temp_path = std::filesystem::temp_directory_path() / "libgnss_rinex_writer_test_bds.nav";
    std::filesystem::remove(temp_path);

    io::RINEXWriter writer;
    io::RINEXReader::RINEXHeader header;
    header.version = 3.04;
    header.file_type = io::RINEXReader::FileType::NAVIGATION;
    header.satellite_system = "M";
    ASSERT_TRUE(writer.createNavigationFile(temp_path.string(), header));

    const Ephemeris expected = makeBeiDouEphemeris();
    ASSERT_TRUE(writer.writeNavigationMessage(expected));

    writer.close();

    io::RINEXReader reader;
    ASSERT_TRUE(reader.open(temp_path.string()));

    NavigationData nav_data;
    ASSERT_TRUE(reader.readNavigationData(nav_data));
    const auto it = nav_data.ephemeris_data.find(expected.satellite);
    ASSERT_NE(it, nav_data.ephemeris_data.end());
    ASSERT_EQ(it->second.size(), 1U);

    const auto& actual = it->second.front();
    EXPECT_EQ(actual.satellite.system, GNSSSystem::BeiDou);
    EXPECT_EQ(actual.satellite.prn, expected.satellite.prn);
    EXPECT_EQ(actual.week, expected.week);
    EXPECT_NEAR(actual.toe.tow, expected.toe.tow, 1e-3);
    EXPECT_NEAR(actual.toes, expected.toes, 1e-6);
    EXPECT_NEAR(actual.af0, expected.af0, 1e-12);
    EXPECT_NEAR(actual.af1, expected.af1, 1e-18);
    EXPECT_NEAR(actual.sqrt_a, expected.sqrt_a, 1e-9);
    EXPECT_NEAR(actual.omega0, expected.omega0, 1e-12);
    EXPECT_NEAR(actual.i0, expected.i0, 1e-12);
    EXPECT_NEAR(actual.tgd, expected.tgd, 1e-18);
    EXPECT_NEAR(actual.tgd_secondary, expected.tgd_secondary, 1e-18);
    EXPECT_EQ(actual.iodc, expected.iodc);

    reader.close();
    std::filesystem::remove(temp_path);
}

TEST(RINEXWriterTest, WritesGalileoNavigationMessageReadableByReader) {
    const auto temp_path = std::filesystem::temp_directory_path() / "libgnss_rinex_writer_test_gal.nav";
    std::filesystem::remove(temp_path);

    io::RINEXWriter writer;
    io::RINEXReader::RINEXHeader header;
    header.version = 3.04;
    header.file_type = io::RINEXReader::FileType::NAVIGATION;
    header.satellite_system = "M";
    ASSERT_TRUE(writer.createNavigationFile(temp_path.string(), header));

    const Ephemeris expected = makeGalileoEphemeris();
    ASSERT_TRUE(writer.writeNavigationMessage(expected));
    writer.close();

    io::RINEXReader reader;
    ASSERT_TRUE(reader.open(temp_path.string()));

    NavigationData nav_data;
    ASSERT_TRUE(reader.readNavigationData(nav_data));
    const auto it = nav_data.ephemeris_data.find(expected.satellite);
    ASSERT_NE(it, nav_data.ephemeris_data.end());
    ASSERT_EQ(it->second.size(), 1U);

    const auto& actual = it->second.front();
    EXPECT_EQ(actual.satellite.system, GNSSSystem::Galileo);
    EXPECT_EQ(actual.satellite.prn, expected.satellite.prn);
    EXPECT_EQ(actual.week, expected.week);
    EXPECT_NEAR(actual.toe.tow, expected.toe.tow, 1e-3);
    EXPECT_NEAR(actual.toes, expected.toes, 1e-6);
    EXPECT_NEAR(actual.af0, expected.af0, 1e-12);
    EXPECT_NEAR(actual.af1, expected.af1, 1e-18);
    EXPECT_NEAR(actual.sqrt_a, expected.sqrt_a, 1e-9);
    EXPECT_NEAR(actual.omega0, expected.omega0, 1e-12);
    EXPECT_NEAR(actual.i0, expected.i0, 1e-12);
    EXPECT_NEAR(actual.tgd, expected.tgd, 1e-18);
    EXPECT_NEAR(actual.tgd_secondary, expected.tgd_secondary, 1e-18);
    EXPECT_EQ(actual.iodc, expected.iodc);

    reader.close();
    std::filesystem::remove(temp_path);
}

TEST(RINEXWriterTest, ReadsObservationAntennaHeaderFields) {
    const auto temp_path =
        std::filesystem::temp_directory_path() / "libgnss_rinex_obs_antenna_header_test.obs";
    std::filesystem::remove(temp_path);

    std::ofstream output(temp_path);
    ASSERT_TRUE(output.is_open());
    output << "     3.04           O                   M                   RINEX VERSION / TYPE\n";
    output << "libgnss++           tests               20260327 000000 UTC PGM / RUN BY / DATE\n";
    output << "     3875000.0000     3325000.0000     4975000.0000                  APPROX POSITION XYZ\n";
    output << "12345               TEST-ANT                                ANT # / TYPE\n";
    output << "        1.2340        0.1230       -0.4560                  ANTENNA: DELTA H/E/N\n";
    output << "G    4 C1C L1C C2W L2W                                      SYS / # / OBS TYPES\n";
    output << "                                                            END OF HEADER\n";
    output.close();

    io::RINEXReader reader;
    ASSERT_TRUE(reader.open(temp_path.string()));

    io::RINEXReader::RINEXHeader header;
    ASSERT_TRUE(reader.readHeader(header));

    EXPECT_EQ(header.antenna_number, "12345");
    EXPECT_EQ(header.antenna_type, "TEST-ANT");
    EXPECT_NEAR(header.antenna_delta.x(), 0.1230, 1e-9);
    EXPECT_NEAR(header.antenna_delta.y(), -0.4560, 1e-9);
    EXPECT_NEAR(header.antenna_delta.z(), 1.2340, 1e-9);

    reader.close();
    std::filesystem::remove(temp_path);
}

TEST(RINEXReaderTest, KeepsRinex3TrackingCodeFieldsTogether) {
    const auto temp_path =
        std::filesystem::temp_directory_path() / "libgnss_rinex_obs_tracking_code_test.obs";
    std::filesystem::remove(temp_path);

    std::ofstream output(temp_path);
    ASSERT_TRUE(output.is_open());
    output << rinexHeaderLine(
        "     3.04           OBSERVATION DATA    M",
        "RINEX VERSION / TYPE");
    output << rinexHeaderLine(
        "G    6 C1C L1C D1C S1C C1W S1W",
        "SYS / # / OBS TYPES");
    output << rinexHeaderLine("", "END OF HEADER");
    output << "> 2024 08 03 09 51 20.0000000  0  1\n";
    output << "G24"
           << rinexObsField("22011162.552", ' ', '6')
           << rinexObsField("115669443.467", '0', '6')
           << rinexObsField("-2361.403", ' ', '6')
           << rinexObsField("41.156")
           << rinexObsField("22011162.423", ' ', '5')
           << rinexObsField("31.406")
           << "\n";
    output.close();

    io::RINEXReader reader;
    ASSERT_TRUE(reader.open(temp_path.string()));

    io::RINEXReader::RINEXHeader header;
    ASSERT_TRUE(reader.readHeader(header));

    ObservationData epoch;
    ASSERT_TRUE(reader.readObservationEpoch(epoch));

    const auto* gps_l1 = epoch.getObservation(
        SatelliteId(GNSSSystem::GPS, 24),
        SignalType::GPS_L1CA);
    ASSERT_NE(gps_l1, nullptr);
    EXPECT_TRUE(gps_l1->has_pseudorange);
    EXPECT_TRUE(gps_l1->has_carrier_phase);
    EXPECT_TRUE(gps_l1->has_doppler);
    EXPECT_NEAR(gps_l1->pseudorange, 22011162.552, 1e-3);
    EXPECT_NEAR(gps_l1->carrier_phase, 115669443.467, 1e-3);
    EXPECT_NEAR(gps_l1->doppler, -2361.403, 1e-3);
    EXPECT_NEAR(gps_l1->snr, 41.156, 1e-3);
    EXPECT_EQ(gps_l1->lli, 0);

    reader.close();
    std::filesystem::remove(temp_path);
}

TEST(RINEXReaderTest, PreservesExactGpsL2RinexObservationIdentity) {
    auto read_l2_epoch = [](const std::string& label,
                            const std::string& l2_obs_types,
                            const std::string& c2_value,
                            const std::string& l2_value) {
        const auto temp_path =
            std::filesystem::temp_directory_path() /
            ("libgnss_gps_l2_identity_" + label + ".obs");
        std::filesystem::remove(temp_path);

        std::ofstream output(temp_path);
        EXPECT_TRUE(output.is_open());
        output << rinexHeaderLine(
            "     3.04           OBSERVATION DATA    M",
            "RINEX VERSION / TYPE");
        output << rinexHeaderLine(
            "G    4 C1C L1C " + l2_obs_types,
            "SYS / # / OBS TYPES");
        output << rinexHeaderLine("", "END OF HEADER");
        output << "> 2024 08 03 09 51 20.0000000  0  1\n";
        output << "G24"
               << rinexObsField("22011162.552")
               << rinexObsField("115669443.467")
               << rinexObsField(c2_value)
               << rinexObsField(l2_value)
               << "\n";
        output.close();

        io::RINEXReader reader;
        EXPECT_TRUE(reader.open(temp_path.string()));
        io::RINEXReader::RINEXHeader header;
        EXPECT_TRUE(reader.readHeader(header));
        ObservationData epoch;
        EXPECT_TRUE(reader.readObservationEpoch(epoch));
        reader.close();
        std::filesystem::remove(temp_path);
        return epoch;
    };

    const SatelliteId gps_24(GNSSSystem::GPS, 24);

    const ObservationData w_epoch =
        read_l2_epoch("w", "C2W L2W", "22022262.423", "120222443.467");
    const auto* gps_l2w = w_epoch.getObservation(gps_24, SignalType::GPS_L2C);
    ASSERT_NE(gps_l2w, nullptr);
    EXPECT_NEAR(gps_l2w->pseudorange, 22022262.423, 1e-3);
    EXPECT_NEAR(gps_l2w->carrier_phase, 120222443.467, 1e-3);
    EXPECT_EQ(gps_l2w->pseudorange_observation_type, "C2W");
    EXPECT_EQ(gps_l2w->carrier_phase_observation_type, "L2W");
    EXPECT_EQ(gps_l2w->exactBiasObservationType(), "C2W");
    const auto* exact_l2w = w_epoch.getRinexTrackingObservation(gps_24, "2W");
    ASSERT_NE(exact_l2w, nullptr);
    EXPECT_NEAR(exact_l2w->pseudorange, 22022262.423, 1e-3);
    EXPECT_NEAR(exact_l2w->carrier_phase, 120222443.467, 1e-3);
    const auto* w_slot = w_epoch.getRinexFrequencySlot(GNSSSystem::GPS, 1);
    ASSERT_NE(w_slot, nullptr);
    EXPECT_EQ(*w_slot, "2W");

    const ObservationData x_epoch =
        read_l2_epoch("x", "C2X L2X", "22033362.423", "120333443.467");
    const auto* gps_l2x = x_epoch.getObservation(gps_24, SignalType::GPS_L2C);
    ASSERT_NE(gps_l2x, nullptr);
    EXPECT_NEAR(gps_l2x->pseudorange, 22033362.423, 1e-3);
    EXPECT_NEAR(gps_l2x->carrier_phase, 120333443.467, 1e-3);
    EXPECT_EQ(gps_l2x->pseudorange_observation_type, "C2X");
    EXPECT_EQ(gps_l2x->carrier_phase_observation_type, "L2X");
    EXPECT_EQ(gps_l2x->exactBiasObservationType(), "C2X");
    const auto* exact_l2x = x_epoch.getRinexTrackingObservation(gps_24, "2X");
    ASSERT_NE(exact_l2x, nullptr);
    EXPECT_NEAR(exact_l2x->pseudorange, 22033362.423, 1e-3);
    EXPECT_NEAR(exact_l2x->carrier_phase, 120333443.467, 1e-3);
    const auto* x_slot = x_epoch.getRinexFrequencySlot(GNSSSystem::GPS, 1);
    ASSERT_NE(x_slot, nullptr);
    EXPECT_EQ(*x_slot, "2X");
}

TEST(RINEXReaderTest, GpsL2HeaderSlotDoesNotFallbackWhenL2wIsMissing) {
    const auto temp_path = std::filesystem::temp_directory_path() /
        "libgnss_gps_l2_fixed_header_slot.obs";
    std::filesystem::remove(temp_path);

    std::ofstream output(temp_path);
    ASSERT_TRUE(output.is_open());
    output << rinexHeaderLine(
        "     3.04           OBSERVATION DATA    M",
        "RINEX VERSION / TYPE");
    output << rinexHeaderLine(
        "G    6 C1C L1C C2W L2W C2L L2L",
        "SYS / # / OBS TYPES");
    output << rinexHeaderLine("", "END OF HEADER");
    output << "> 2024 08 03 09 51 20.0000000  0  1\n";
    output << "G24"
           << rinexObsField("22011162.552")
           << rinexObsField("115669443.467")
           << rinexObsField("")
           << rinexObsField("")
           << rinexObsField("22033362.423")
           << rinexObsField("120333443.467")
           << "\n";
    output.close();

    io::RINEXReader reader;
    ASSERT_TRUE(reader.open(temp_path.string()));
    io::RINEXReader::RINEXHeader header;
    ASSERT_TRUE(reader.readHeader(header));
    ObservationData epoch;
    ASSERT_TRUE(reader.readObservationEpoch(epoch));

    const auto* slot = epoch.getRinexFrequencySlot(GNSSSystem::GPS, 1);
    ASSERT_NE(slot, nullptr);
    EXPECT_EQ(*slot, "2W");
    EXPECT_EQ(epoch.getRinexTrackingObservation(
                  SatelliteId(GNSSSystem::GPS, 24), "2W"),
              nullptr);
    EXPECT_NE(epoch.getRinexTrackingObservation(
                  SatelliteId(GNSSSystem::GPS, 24), "2L"),
              nullptr);

    reader.close();
    std::filesystem::remove(temp_path);
}

TEST(RINEXReaderTest, QzssSecondaryL5PreferenceSelectsL5Q) {
    const auto temp_path =
        std::filesystem::temp_directory_path() / "libgnss_qzss_l5_preference_test.obs";
    std::filesystem::remove(temp_path);

    std::ofstream output(temp_path);
    ASSERT_TRUE(output.is_open());
    output << rinexHeaderLine(
        "     3.04           OBSERVATION DATA    M",
        "RINEX VERSION / TYPE");
    output << rinexHeaderLine(
        "J    8 C1L L1L C2L L2L C5P L5P C5Q L5Q",
        "SYS / # / OBS TYPES");
    output << rinexHeaderLine("", "END OF HEADER");
    output << "> 2025 04 01 00 00 00.0000000  0  1\n";
    output << "J02"
           << rinexObsField("21000001.000")
           << rinexObsField("110000001.000")
           << rinexObsField("22000002.000")
           << rinexObsField("120000002.000")
           << rinexObsField("23000003.000")
           << rinexObsField("130000003.000")
           << rinexObsField("24000004.000")
           << rinexObsField("140000004.000")
           << "\n";
    output.close();

    auto read_epoch = [&](bool prefer_l5, bool preserve_additional_bands) {
        io::RINEXReader reader;
        reader.setQzssSecondaryL5Preference(prefer_l5);
        reader.setPreserveAdditionalFrequencyBands(preserve_additional_bands);
        EXPECT_TRUE(reader.open(temp_path.string()));
        io::RINEXReader::RINEXHeader header;
        EXPECT_TRUE(reader.readHeader(header));
        ObservationData epoch;
        EXPECT_TRUE(reader.readObservationEpoch(epoch));
        reader.close();
        return epoch;
    };

    const SatelliteId qzss_2(GNSSSystem::QZSS, 2);
    const ObservationData default_epoch = read_epoch(false, false);
    EXPECT_NE(default_epoch.getObservation(qzss_2, SignalType::QZS_L2C), nullptr);
    EXPECT_EQ(default_epoch.getObservation(qzss_2, SignalType::QZS_L5), nullptr);

    const ObservationData l5_epoch = read_epoch(true, false);
    EXPECT_EQ(l5_epoch.getObservation(qzss_2, SignalType::QZS_L2C), nullptr);
    const auto* qzss_l5 = l5_epoch.getObservation(qzss_2, SignalType::QZS_L5);
    ASSERT_NE(qzss_l5, nullptr);
    EXPECT_NEAR(qzss_l5->pseudorange, 24000004.000, 1e-3);
    EXPECT_NEAR(qzss_l5->carrier_phase, 140000004.000, 1e-3);
    EXPECT_EQ(qzss_l5->pseudorange_observation_type, "C5Q");
    EXPECT_EQ(qzss_l5->carrier_phase_observation_type, "L5Q");

    const ObservationData three_frequency_epoch = read_epoch(true, true);
    EXPECT_EQ(three_frequency_epoch.getObservations(qzss_2).size(), 3U);
    EXPECT_NE(
        three_frequency_epoch.getObservation(qzss_2, SignalType::QZS_L1CA),
        nullptr);
    EXPECT_NE(
        three_frequency_epoch.getObservation(qzss_2, SignalType::QZS_L5),
        nullptr);
    EXPECT_NE(
        three_frequency_epoch.getObservation(qzss_2, SignalType::QZS_L2C),
        nullptr);

    std::filesystem::remove(temp_path);
}

TEST(RINEXReaderTest, AdditionalFrequencyModePreservesFourGalileoBands) {
    const auto temp_path = std::filesystem::temp_directory_path() /
        "libgnss_galileo_four_frequency_test.obs";
    std::filesystem::remove(temp_path);

    std::ofstream output(temp_path);
    ASSERT_TRUE(output.is_open());
    output << rinexHeaderLine(
        "     3.04           OBSERVATION DATA    M",
        "RINEX VERSION / TYPE");
    output << rinexHeaderLine(
        "E    8 C1C L1C C5Q L5Q C7Q L7Q C6C L6C",
        "SYS / # / OBS TYPES");
    output << rinexHeaderLine("", "END OF HEADER");
    output << "> 2025 04 01 00 00 00.0000000  0  1\n";
    output << "E09"
           << rinexObsField("21000001.000")
           << rinexObsField("110000001.000")
           << rinexObsField("22000002.000")
           << rinexObsField("120000002.000")
           << rinexObsField("23000003.000")
           << rinexObsField("130000003.000")
           << rinexObsField("24000004.000")
           << rinexObsField("140000004.000")
           << "\n";
    output.close();

    const auto read_epoch = [&](bool preserve_additional_bands) {
        io::RINEXReader reader;
        reader.setPreserveAdditionalFrequencyBands(preserve_additional_bands);
        EXPECT_TRUE(reader.open(temp_path.string()));
        io::RINEXReader::RINEXHeader header;
        EXPECT_TRUE(reader.readHeader(header));
        ObservationData epoch;
        EXPECT_TRUE(reader.readObservationEpoch(epoch));
        reader.close();
        return epoch;
    };

    const SatelliteId galileo_9(GNSSSystem::Galileo, 9);
    const ObservationData default_epoch = read_epoch(false);
    EXPECT_EQ(default_epoch.getObservations(galileo_9).size(), 2U);
    EXPECT_NE(default_epoch.getObservation(galileo_9, SignalType::GAL_E1), nullptr);
    EXPECT_NE(default_epoch.getObservation(galileo_9, SignalType::GAL_E5A), nullptr);
    EXPECT_EQ(default_epoch.getObservation(galileo_9, SignalType::GAL_E5B), nullptr);
    EXPECT_EQ(default_epoch.getObservation(galileo_9, SignalType::GAL_E6), nullptr);

    const ObservationData four_frequency_epoch = read_epoch(true);
    EXPECT_EQ(four_frequency_epoch.getObservations(galileo_9).size(), 4U);
    EXPECT_NE(four_frequency_epoch.getObservation(galileo_9, SignalType::GAL_E1), nullptr);
    EXPECT_NE(four_frequency_epoch.getObservation(galileo_9, SignalType::GAL_E5A), nullptr);
    EXPECT_NE(four_frequency_epoch.getObservation(galileo_9, SignalType::GAL_E5B), nullptr);
    EXPECT_NE(four_frequency_epoch.getObservation(galileo_9, SignalType::GAL_E6), nullptr);

    std::filesystem::remove(temp_path);
}
