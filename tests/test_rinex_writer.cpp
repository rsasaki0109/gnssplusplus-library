#include <gtest/gtest.h>
#include <libgnss++/io/rinex.hpp>

#include <filesystem>
#include <fstream>

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

TEST(RINEXReaderTest, DetectsRinex4AndDoesNotUseRinex3EpochColumns) {
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
        // The extra second digits intentionally shift the RINEX 3 fixed
        // columns.  Phase 1 must reject this explicitly until token-based
        // RINEX 4 observation parsing is implemented.
        file << "> 2024 08 03 09 51 20.1234567890123  0  1\n";
        file << "G04" << rinexObsField("22011162.552")
             << rinexObsField("115669443.467")
             << rinexObsField("22022262.423")
             << rinexObsField("120222443.467") << "\n";
    }

    io::RINEXReader reader;
    ASSERT_TRUE(reader.open(temp_path.string()));
    io::RINEXReader::RINEXHeader header;
    ASSERT_TRUE(reader.readHeader(header));
    EXPECT_DOUBLE_EQ(header.version, 4.02);
    EXPECT_TRUE(reader.isRinex4());

    ObservationData epoch;
    testing::internal::CaptureStderr();
    EXPECT_FALSE(reader.readObservationEpoch(epoch));
    const std::string diagnostic = testing::internal::GetCapturedStderr();
    EXPECT_NE(diagnostic.find("RINEX 4 observation epochs are not supported yet"),
              std::string::npos);

    reader.close();
    std::filesystem::remove(temp_path);
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
    EXPECT_NE(diagnostic.find("Skipping unsupported RINEX 4 navigation record type: STO"),
              std::string::npos);
    EXPECT_NE(diagnostic.find("Skipping unsupported RINEX 4 EPH I05 LNAV"),
              std::string::npos);
    EXPECT_NE(diagnostic.find("Skipping unsupported RINEX 4 EPH G04 CNAV"),
              std::string::npos);

    const auto it = nav_data.ephemeris_data.find(SatelliteId(GNSSSystem::GPS, 4));
    ASSERT_NE(it, nav_data.ephemeris_data.end());
    ASSERT_EQ(it->second.size(), 1U);
    EXPECT_EQ(it->second.front().satellite, SatelliteId(GNSSSystem::GPS, 4));
    EXPECT_EQ(it->second.front().week, 2044);
    EXPECT_NEAR(it->second.front().toes, 360000.0, 1e-6);

    const auto bds_it = nav_data.ephemeris_data.find(
        SatelliteId(GNSSSystem::BeiDou, 1));
    ASSERT_NE(bds_it, nav_data.ephemeris_data.end());
    ASSERT_EQ(bds_it->second.size(), 1U);

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
