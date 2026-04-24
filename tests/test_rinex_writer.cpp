#include <gtest/gtest.h>
#include <libgnss++/io/rinex.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>

using namespace libgnss;

namespace {

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

std::string makeRinex3ObsTypesLine(char system,
                                   int total_count,
                                   const std::vector<std::string>& types) {
    std::ostringstream line_stream;
    if (system == ' ') {
        line_stream << "       ";
    } else {
        line_stream << system << "  " << std::setw(3) << total_count << ' ';
    }
    for (const auto& type : types) {
        line_stream << type << ' ';
    }

    std::string line = line_stream.str();
    if (line.size() < 60) {
        line.resize(60, ' ');
    }
    line += "SYS / # / OBS TYPES";
    return line;
}

std::string makeRinex3ObservationLine(const std::string& satellite,
                                      const std::vector<double>& values) {
    std::string line = satellite;
    for (const double value : values) {
        if (value == 0.0) {
            line += std::string(16, ' ');
            continue;
        }
        std::ostringstream value_stream;
        value_stream << std::fixed << std::setprecision(3) << std::setw(14) << value << "  ";
        line += value_stream.str();
    }
    return line;
}

}  // namespace

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


TEST(RINEXWriterTest, KeepsRinex3CodeAndCarrierOnSelectedTrackingCode) {
    const auto temp_path =
        std::filesystem::temp_directory_path() / "libgnss_rinex_obs_tracking_code_selection_test.obs";
    std::filesystem::remove(temp_path);

    std::ofstream output(temp_path);
    ASSERT_TRUE(output.is_open());
    output << "     3.04           O                   M                   RINEX VERSION / TYPE\n";
    output << "libgnss++           tests               20260421 000000 UTC PGM / RUN BY / DATE\n";
    output << makeRinex3ObsTypesLine('G', 9,
                                      {"C1C", "C1L", "C1W", "L1C", "L1L",
                                       "C2L", "C2W", "L2L", "L2W"})
           << "\n";
    output << "                                                            END OF HEADER\n";
    output << "> 2025 04 01 00 00 00.0000000  0  1\n";

    std::vector<double> gps_values(9, 0.0);
    gps_values[0] = 20200000.125;  // C1C: highest-priority GPS L1 code
    gps_values[1] = 20200010.000;  // C1L: must not overwrite C1C
    gps_values[2] = 20200020.000;  // C1W: must not overwrite C1C
    gps_values[3] = 106150000.250; // L1C: must match selected C1C
    gps_values[4] = 106150010.000; // L1L: must not mix with C1C
    gps_values[5] = 20200005.000;  // C2L: lower priority than C2W
    gps_values[6] = 20200006.500;  // C2W: selected GPS L2 code
    gps_values[7] = 82700000.000;  // L2L: must not mix with C2W
    gps_values[8] = 82700000.750;  // L2W: must match selected C2W
    output << makeRinex3ObservationLine("G01", gps_values) << "\n";
    output.close();

    io::RINEXReader reader;
    ASSERT_TRUE(reader.open(temp_path.string()));

    io::RINEXReader::RINEXHeader header;
    ASSERT_TRUE(reader.readHeader(header));

    ObservationData epoch;
    ASSERT_TRUE(reader.readObservationEpoch(epoch));

    const SatelliteId gps_sat(GNSSSystem::GPS, 1);
    const Observation* gps_l1 = epoch.getObservation(gps_sat, SignalType::GPS_L1CA);
    ASSERT_NE(gps_l1, nullptr);
    EXPECT_TRUE(gps_l1->has_pseudorange);
    EXPECT_TRUE(gps_l1->has_carrier_phase);
    EXPECT_NEAR(gps_l1->pseudorange, 20200000.125, 1e-6);
    EXPECT_NEAR(gps_l1->carrier_phase, 106150000.250, 1e-6);

    const Observation* gps_l2 = epoch.getObservation(gps_sat, SignalType::GPS_L2P);
    ASSERT_NE(gps_l2, nullptr);
    EXPECT_EQ(epoch.getObservation(gps_sat, SignalType::GPS_L2C), nullptr);
    EXPECT_TRUE(gps_l2->has_pseudorange);
    EXPECT_TRUE(gps_l2->has_carrier_phase);
    EXPECT_EQ(gps_l2->observation_code, "C2W");
    EXPECT_NEAR(gps_l2->pseudorange, 20200006.500, 1e-6);
    EXPECT_NEAR(gps_l2->carrier_phase, 82700000.750, 1e-6);

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


TEST(RINEXWriterTest, ReadsRinex3ContinuedObservationTypesAndValues) {
    const auto temp_path =
        std::filesystem::temp_directory_path() / "libgnss_rinex_obs_types_continuation_test.obs";
    std::filesystem::remove(temp_path);

    std::ofstream output(temp_path);
    ASSERT_TRUE(output.is_open());
    output << "     3.04           O                   M                   RINEX VERSION / TYPE\n";
    output << "libgnss++           tests               20260421 000000 UTC PGM / RUN BY / DATE\n";
    output << makeRinex3ObsTypesLine('G', 15,
                                      {"C1C", "L1C", "D1C", "S1C", "C2W", "D2W", "S2W",
                                       "C5Q", "L5Q", "D5Q", "S5Q", "C1L", "L1L"})
           << "\n";
    output << makeRinex3ObsTypesLine(' ', 15, {"L2W", "S1W"}) << "\n";
    output << makeRinex3ObsTypesLine('C', 15,
                                      {"C2I", "L2I", "D2I", "S2I", "C6I", "L6I", "D6I",
                                       "S6I", "C7I", "D7I", "S7I", "C1P", "L1P"})
           << "\n";
    output << makeRinex3ObsTypesLine(' ', 15, {"L7I", "S1P"}) << "\n";
    output << "                                                            END OF HEADER\n";
    output << "> 2025 04 01 00 00 00.0000000  0  2\n";

    std::vector<double> gps_values(15, 0.0);
    gps_values[0] = 20200000.125;
    gps_values[1] = 106150000.250;
    gps_values[4] = 20200005.500;
    gps_values[13] = 82700000.750;
    output << makeRinex3ObservationLine("G01", gps_values) << "\n";

    std::vector<double> bds_values(15, 0.0);
    bds_values[0] = 21400000.125;
    bds_values[1] = 112000000.250;
    bds_values[8] = 21400005.500;
    bds_values[13] = 86000000.750;
    output << makeRinex3ObservationLine("C07", bds_values) << "\n";
    output.close();

    io::RINEXReader reader;
    ASSERT_TRUE(reader.open(temp_path.string()));

    io::RINEXReader::RINEXHeader header;
    ASSERT_TRUE(reader.readHeader(header));

    ASSERT_EQ(header.system_obs_types['G'].size(), 15U);
    ASSERT_EQ(header.system_obs_types['C'].size(), 15U);
    EXPECT_EQ(header.system_obs_types['G'][13], "L2W");
    EXPECT_EQ(header.system_obs_types['C'][13], "L7I");
    EXPECT_NE(std::find(header.observation_types.begin(), header.observation_types.end(), "L2W"),
              header.observation_types.end());

    ObservationData epoch;
    ASSERT_TRUE(reader.readObservationEpoch(epoch));

    const SatelliteId gps_sat(GNSSSystem::GPS, 1);
    const Observation* gps_l2 = epoch.getObservation(gps_sat, SignalType::GPS_L2P);
    ASSERT_NE(gps_l2, nullptr);
    EXPECT_EQ(epoch.getObservation(gps_sat, SignalType::GPS_L2C), nullptr);
    EXPECT_TRUE(gps_l2->has_pseudorange);
    EXPECT_TRUE(gps_l2->has_carrier_phase);
    EXPECT_EQ(gps_l2->observation_code, "C2W");
    EXPECT_NEAR(gps_l2->pseudorange, 20200005.500, 1e-6);
    EXPECT_NEAR(gps_l2->carrier_phase, 82700000.750, 1e-6);

    const SatelliteId bds_sat(GNSSSystem::BeiDou, 7);
    const Observation* bds_b2i = epoch.getObservation(bds_sat, SignalType::BDS_B2I);
    ASSERT_NE(bds_b2i, nullptr);
    EXPECT_TRUE(bds_b2i->has_pseudorange);
    EXPECT_TRUE(bds_b2i->has_carrier_phase);
    EXPECT_NEAR(bds_b2i->pseudorange, 21400005.500, 1e-6);
    EXPECT_NEAR(bds_b2i->carrier_phase, 86000000.750, 1e-6);

    reader.close();
    std::filesystem::remove(temp_path);
}
