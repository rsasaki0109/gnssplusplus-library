#include <gtest/gtest.h>
#include <libgnss++/io/rinex.hpp>

#include <filesystem>

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
