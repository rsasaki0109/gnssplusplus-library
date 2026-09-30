// RINEX 4 navigation reader coverage with synthetic mixed-constellation
// files.  Every record below is generated for this test from the RINEX 4.02
// navigation message field layouts (no third-party navigation data is
// embedded): GPS/QZSS LNAV and CNAV, Galileo I/NAV and F/NAV, BeiDou D1 and
// B-CNAV1/2/3, GLONASS FDMA (with the RINEX 4 fifth orbit line) and SBAS.

#include <gtest/gtest.h>
#include <libgnss++/io/rinex.hpp>
#include <libgnss++/io/rinex4.hpp>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace libgnss;

namespace {

std::string headerLine(const std::string& content, const std::string& label) {
    std::string line = content;
    line.resize(60, ' ');
    return line + label + "\n";
}

std::string field(double value) {
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%19.12E", value);
    return buffer;
}

// First record line: satellite, epoch (20 columns) and three D19.12 fields.
std::string epochLine(const std::string& satellite,
                      const std::string& epoch,
                      double a,
                      double b,
                      double c) {
    return satellite + " " + epoch + field(a) + field(b) + field(c) + "\n";
}

// Broadcast orbit line: 4X then up to four D19.12 fields.
std::string orbitLine(const std::vector<double>& values) {
    std::string line = "    ";
    for (const double value : values) {
        line += field(value);
    }
    return line + "\n";
}

// GPS LNAV body (2025-03-01 12:00:00 GPST = week 2355, TOW 561600).
std::string gpsLnavBody(const std::string& satellite) {
    return epochLine(satellite, "2025 03 01 12 00 00",
                     1.234567890123e-04, 1.136868377216e-12, 0.0) +
           orbitLine({45.0, -35.21875, 4.512345678901e-09, 1.234567890123e+00}) +
           orbitLine({-1.806020736694e-06, 6.543210987654e-03, 8.123621344566e-06,
                      5.153650000000e+03}) +
           orbitLine({561600.0, 1.117587089539e-08, -2.345678901234e+00,
                      5.587935447693e-08}) +
           orbitLine({9.612345678901e-01, 234.5625, 1.234567890123e+00,
                      -7.891234567890e-09}) +
           orbitLine({2.345678901234e-10, 1.0, 2355.0, 0.0}) +
           orbitLine({2.0, 0.0, -1.024454832077e-08, 45.0}) +
           orbitLine({554418.0, 4.0});
}

// GPS/QZSS CNAV body: nine lines, ADOT / delta-n0-dot / URAI fields where
// LNAV carries IODE / week / accuracy.  Read as LNAV these would produce a
// week of 3 and an IODE of 0.25, which must never reach NavigationData.
std::string cnavBody(const std::string& satellite) {
    return epochLine(satellite, "2025 03 01 12 00 00",
                     1.234000000000e-04, 1.100000000000e-12, 0.0) +
           orbitLine({0.25, -35.0, 4.5e-09, 1.2}) +
           orbitLine({-1.8e-06, 6.5e-03, 8.1e-06, 5.15365e+03}) +
           orbitLine({561600.0, 1.1e-08, -2.3, 5.6e-08}) +
           orbitLine({9.6e-01, 234.0, 1.2, -7.9e-09}) +
           orbitLine({2.3e-10, -1.0e-13, 3.0, 2.0}) +
           orbitLine({1.0, 0.0, -1.0e-08, 1.0}) +
           orbitLine({1.5e-09, -2.0e-09, 3.0e-09, 4.0e-09}) +
           orbitLine({554418.0, 561600.0});
}

// Galileo body; data_source selects I/NAV (bits 0/2 + 9) or F/NAV (1 + 8).
std::string galileoBody(const std::string& satellite, double data_source, double bgd_b) {
    return epochLine(satellite, "2025 03 01 12 00 00",
                     -3.456789012345e-04, -8.412825991400e-12, 0.0) +
           orbitLine({87.0, -120.5, 3.123456789012e-09, -2.123456789012e+00}) +
           orbitLine({-5.498528480530e-06, 2.345678901234e-04, 4.965439438820e-06,
                      5.440612345678e+03}) +
           orbitLine({561600.0, 3.725290298462e-09, 1.678901234567e+00,
                      -2.607703208923e-08}) +
           orbitLine({9.612345678901e-01, 243.5625, -1.234567890123e+00,
                      -5.567088463215e-09}) +
           orbitLine({-4.114456743811e-10, data_source, 2355.0}) +
           orbitLine({3.12, 0.0, -4.190951585770e-09, bgd_b}) +
           orbitLine({561000.0});
}

// BeiDou D1 body; epoch and TOE are BDT (week 999 = GPS week 2355).
std::string beidouD1Body(const std::string& satellite) {
    return epochLine(satellite, "2025 03 01 11 59 46",
                     2.345678901234e-04, 3.552713678801e-14, 0.0) +
           orbitLine({1.0, 12.5, 4.012345678901e-09, 2.345678901234e+00}) +
           orbitLine({6.123456789012e-07, 1.234567890123e-03, 7.123456789012e-06,
                      5.282612345678e+03}) +
           orbitLine({561586.0, -2.793967723846e-08, 1.234567890123e+00,
                      1.024454832077e-08}) +
           orbitLine({9.612345678901e-01, 150.0, -2.345678901234e+00,
                      -6.789012345678e-09}) +
           orbitLine({1.234567890123e-10, 0.0, 999.0, 0.0}) +
           orbitLine({2.0, 0.0, 2.300000000000e-09, -1.000000000000e-09}) +
           orbitLine({561580.0, 1.0});
}

// BeiDou B-CNAV bodies: CNV1/CNV2 have ten lines, CNV3 nine.
std::string beidouCnavBody(const std::string& satellite, int lines) {
    std::string body = epochLine(satellite, "2025 03 01 11 59 46",
                                 2.3e-04, 3.5e-14, 0.0);
    for (int line = 1; line < lines; ++line) {
        body += orbitLine({0.5 * line, -3.0, 1.0e-09, 2.0});
    }
    return body;
}

// GLONASS FDMA body with the RINEX 4 fifth orbit line (status flags,
// L1/L2 group delay difference, URAI, health flags).  Position in km,
// velocity km/s, acceleration km/s^2; TauN is written as -TauN.
std::string glonassFdmaBody(const std::string& satellite,
                            const std::string& epoch_utc,
                            double z_km,
                            double vz_km_s) {
    return epochLine(satellite, epoch_utc,
                     -(-4.567890123456e-05), 9.094947017729e-13, 561582.0) +
           orbitLine({12000.0, 1.5, 9.313225746155e-10, 0.0}) +
           orbitLine({-18000.0, 1.5, -1.862645149231e-09, -2.0}) +
           orbitLine({z_km, vz_km_s, 0.0, 0.0}) +
           orbitLine({1.83e+02, 3.725290298462e-09, 3.0, 0.0});
}

std::string sbasBody(const std::string& satellite) {
    return epochLine(satellite, "2025 03 01 11 59 44",
                     1.234567890123e-08, 0.0, 561584.0) +
           orbitLine({-33000.0, 0.0, 0.0, 0.0}) +
           orbitLine({24000.0, 0.0, 0.0, 32.0}) +
           orbitLine({50.0, 0.0, 0.0, 12.0});
}

// Nominal GLONASS state: |r| = 25510 km, velocity orthogonal to r.
constexpr double kGlonassZKm = 13518.879391428862;
constexpr double kGlonassVzKmPerS = 0.6657356530383807;

std::filesystem::path writeTempFile(const std::string& name, const std::string& content) {
    const auto path = std::filesystem::temp_directory_path() / name;
    std::filesystem::remove(path);
    std::ofstream file(path, std::ios::binary);
    file << content;
    return path;
}

std::string rinex4Header() {
    return headerLine("     4.02           NAVIGATION DATA     M", "RINEX VERSION / TYPE") +
           headerLine("    18", "LEAP SECONDS") +
           headerLine("", "END OF HEADER");
}

std::string rinex3Header() {
    return headerLine("     3.05           N: GNSS NAV DATA    M: MIXED", "RINEX VERSION / TYPE") +
           headerLine("", "END OF HEADER");
}

// Four-line GLONASS body as written by RINEX 3.
std::string glonassRinex3Body(const std::string& satellite, const std::string& epoch_utc) {
    const std::string body =
        glonassFdmaBody(satellite, epoch_utc, kGlonassZKm, kGlonassVzKmPerS);
    size_t end = 0;
    for (int line = 0; line < 4; ++line) {
        end = body.find('\n', end) + 1;
    }
    return body.substr(0, end);
}

struct ReadResult {
    bool ok = false;
    NavigationData nav;
    std::string diagnostics;
};

ReadResult readNavigation(const std::filesystem::path& path) {
    ReadResult result;
    io::RINEXReader reader;
    if (!reader.open(path.string())) {
        return result;
    }
    testing::internal::CaptureStderr();
    result.ok = reader.readNavigationData(result.nav);
    result.diagnostics = testing::internal::GetCapturedStderr();
    reader.close();
    return result;
}

const std::vector<Ephemeris>* ephemerides(const NavigationData& nav, GNSSSystem system, int prn) {
    const auto it = nav.ephemeris_data.find(SatelliteId(system, static_cast<uint8_t>(prn)));
    return it == nav.ephemeris_data.end() ? nullptr : &it->second;
}

bool contains(const std::string& text, const std::string& needle) {
    return text.find(needle) != std::string::npos;
}

}  // namespace

TEST(Rinex4NavigationTest, ReadsEveryConstellationAndKeepsMessageTypesApart) {
    std::string content = rinex4Header();
    content += "> EPH G07 LNAV\n" + gpsLnavBody("G07");
    content += "> EPH G07 CNAV\n" + cnavBody("G07");
    content += "> EPH J02 LNAV\n" + gpsLnavBody("J02");
    content += "> EPH J02 CNAV\n" + cnavBody("J02");
    content += "> EPH E11 INAV\n" + galileoBody("E11", 517.0, -4.656612873077e-09);
    content += "> EPH E11 FNAV\n" + galileoBody("E11", 258.0, 0.0);
    content += "> EPH C25 CNV1\n" + beidouCnavBody("C25", 10);
    content += "> EPH C25 CNV2\n" + beidouCnavBody("C25", 10);
    content += "> EPH C25 CNV3\n" + beidouCnavBody("C25", 9);
    content += "> EPH C25 D1\n" + beidouD1Body("C25");
    content += "> EPH R05 FDMA\n" +
               glonassFdmaBody("R05", "2025 03 01 11 45 00", kGlonassZKm, kGlonassVzKmPerS);
    content += "> EPH S27 SBAS\n" + sbasBody("S27");
    const auto path = writeTempFile("libgnss_rinex4_nav_all_types.rnx", content);

    const ReadResult result = readNavigation(path);
    ASSERT_TRUE(result.ok) << result.diagnostics;

    // CNAV / B-CNAV records are reported and skipped, never parsed as LNAV/D1.
    for (const char* skipped : {"G07 CNAV", "J02 CNAV", "C25 CNV1", "C25 CNV2", "C25 CNV3"}) {
        EXPECT_TRUE(contains(result.diagnostics,
                             std::string("Skipping unsupported RINEX 4 EPH ") + skipped))
            << skipped << "\n" << result.diagnostics;
    }

    for (const GNSSSystem system : {GNSSSystem::GPS, GNSSSystem::QZSS}) {
        const auto* list = ephemerides(result.nav, system, system == GNSSSystem::GPS ? 7 : 2);
        ASSERT_NE(list, nullptr);
        ASSERT_EQ(list->size(), 1U);
        const Ephemeris& eph = list->front();
        EXPECT_EQ(eph.navigation_message_type, NavigationMessageType::LNAV);
        EXPECT_EQ(eph.week, 2355);
        EXPECT_EQ(eph.iode, 45);
        EXPECT_EQ(eph.iodc, 45);
        EXPECT_DOUBLE_EQ(eph.toes, 561600.0);
        EXPECT_EQ(eph.toc, GNSSTime(2355, 561600.0));
        EXPECT_DOUBLE_EQ(eph.sqrt_a, 5153.65);
        EXPECT_DOUBLE_EQ(eph.af0, 1.234567890123e-04);
        EXPECT_DOUBLE_EQ(eph.tgd, -1.024454832077e-08);
    }

    const auto* galileo = ephemerides(result.nav, GNSSSystem::Galileo, 11);
    ASSERT_NE(galileo, nullptr);
    ASSERT_EQ(galileo->size(), 2U);
    int inav = 0;
    int fnav = 0;
    for (const auto& eph : *galileo) {
        EXPECT_EQ(eph.week, 2355);
        EXPECT_EQ(eph.toe, GNSSTime(2355, 561600.0));
        EXPECT_EQ(eph.iode, 87);
        EXPECT_DOUBLE_EQ(eph.tgd, -4.190951585770e-09);
        if (eph.navigation_message_type == NavigationMessageType::INAV) {
            ++inav;
            EXPECT_EQ(eph.data_source_code, 517);
            EXPECT_DOUBLE_EQ(eph.tgd_secondary, -4.656612873077e-09);
        } else if (eph.navigation_message_type == NavigationMessageType::FNAV) {
            ++fnav;
            EXPECT_EQ(eph.data_source_code, 258);
        }
    }
    EXPECT_EQ(inav, 1);
    EXPECT_EQ(fnav, 1);

    const auto* beidou = ephemerides(result.nav, GNSSSystem::BeiDou, 25);
    ASSERT_NE(beidou, nullptr);
    ASSERT_EQ(beidou->size(), 1U);
    EXPECT_EQ(beidou->front().navigation_message_type, NavigationMessageType::D1);
    // BDT week 999 / 561586 s is GPST week 2355 / 561600 s.
    EXPECT_EQ(beidou->front().toe, GNSSTime(2355, 561600.0));
    EXPECT_EQ(beidou->front().toc, GNSSTime(2355, 561600.0));
    EXPECT_DOUBLE_EQ(beidou->front().tgd, 2.3e-09);
    EXPECT_DOUBLE_EQ(beidou->front().tgd_secondary, -1.0e-09);

    const auto* glonass = ephemerides(result.nav, GNSSSystem::GLONASS, 5);
    ASSERT_NE(glonass, nullptr);
    ASSERT_EQ(glonass->size(), 1U);
    const Ephemeris& glo = glonass->front();
    EXPECT_EQ(glo.navigation_message_type, NavigationMessageType::FDMA);
    // 11:45:00 UTC + 18 s leap seconds = GPST week 2355, 560718 s.
    EXPECT_EQ(glo.toe, GNSSTime(2355, 560718.0));
    EXPECT_NEAR(glo.glonass_position.x(), 12000.0e3, 1e-6);
    EXPECT_NEAR(glo.glonass_position.z(), kGlonassZKm * 1e3, 1e-3);
    EXPECT_NEAR(glo.glonass_velocity.z(), kGlonassVzKmPerS * 1e3, 1e-6);
    EXPECT_DOUBLE_EQ(glo.glonass_taun, -4.567890123456e-05);
    EXPECT_TRUE(glo.glonass_frequency_channel_present);
    EXPECT_EQ(glo.glonass_frequency_channel, -2);

    const auto* sbas = ephemerides(result.nav, GNSSSystem::SBAS, 127);
    ASSERT_NE(sbas, nullptr);
    ASSERT_EQ(sbas->size(), 1U);
    EXPECT_NEAR(sbas->front().glonass_position.x(), -33000.0e3, 1e-6);

    std::filesystem::remove(path);
}

TEST(Rinex4NavigationTest, MatchesRinex3SatelliteStatesForSharedMessageTypes) {
    // The same broadcast records written as RINEX 3.05 and RINEX 4.02 must
    // give bit-identical satellite states.
    const std::string gps = gpsLnavBody("G07");
    const std::string qzss = gpsLnavBody("J02");
    const std::string inav = galileoBody("E11", 517.0, -4.656612873077e-09);
    const std::string bds = beidouD1Body("C25");
    const std::string glo = glonassRinex3Body("R05", "2025 03 01 11 45 00");

    const auto path3 = writeTempFile("libgnss_rinex3_nav_equivalence.rnx",
                                     rinex3Header() + gps + qzss + inav + bds + glo);
    const auto path4 = writeTempFile(
        "libgnss_rinex4_nav_equivalence.rnx",
        rinex4Header() + "> EPH G07 LNAV\n" + gps + "> EPH J02 LNAV\n" + qzss +
            "> EPH E11 INAV\n" + inav + "> EPH C25 D1\n" + bds + "> EPH R05 FDMA\n" +
            glonassFdmaBody("R05", "2025 03 01 11 45 00", kGlonassZKm, kGlonassVzKmPerS));

    const ReadResult v3 = readNavigation(path3);
    const ReadResult v4 = readNavigation(path4);
    ASSERT_TRUE(v3.ok) << v3.diagnostics;
    ASSERT_TRUE(v4.ok) << v4.diagnostics;

    const std::vector<std::pair<SatelliteId, GNSSTime>> queries = {
        {SatelliteId(GNSSSystem::GPS, 7), GNSSTime(2355, 562000.0)},
        {SatelliteId(GNSSSystem::QZSS, 2), GNSSTime(2355, 562000.0)},
        {SatelliteId(GNSSSystem::Galileo, 11), GNSSTime(2355, 562000.0)},
        {SatelliteId(GNSSSystem::BeiDou, 25), GNSSTime(2355, 562000.0)},
        {SatelliteId(GNSSSystem::GLONASS, 5), GNSSTime(2355, 561000.0)},
    };
    for (const auto& [satellite, time] : queries) {
        Vector3d pos3, vel3, pos4, vel4;
        double clk3 = 0.0, drift3 = 0.0, clk4 = 0.0, drift4 = 0.0;
        ASSERT_TRUE(v3.nav.calculateSatelliteState(satellite, time, pos3, vel3, clk3, drift3))
            << satellite.toString();
        ASSERT_TRUE(v4.nav.calculateSatelliteState(satellite, time, pos4, vel4, clk4, drift4))
            << satellite.toString();
        EXPECT_EQ(pos3, pos4) << satellite.toString();
        EXPECT_EQ(vel3, vel4) << satellite.toString();
        EXPECT_EQ(clk3, clk4) << satellite.toString();
        EXPECT_GT(pos4.norm(), 2.0e7) << satellite.toString();
    }

    std::filesystem::remove(path3);
    std::filesystem::remove(path4);
}

TEST(Rinex4NavigationTest, RejectsGlonassRecordsWhoseStateVectorIsNotOnAnOrbit) {
    // A record whose third orbit line (Z, Vz, Az) belongs to another frame
    // than the X/Y lines: the state vector is far off the GLONASS orbit
    // shell and must not reach NavigationData.
    std::string content = rinex4Header();
    content += "> EPH R05 FDMA\n" +
               glonassFdmaBody("R05", "2025 03 01 11 45 00", kGlonassZKm, kGlonassVzKmPerS);
    content += "> EPH R05 FDMA\n" +
               glonassFdmaBody("R05", "2025 03 01 12 15 00", -2500.0, 3.4);
    content += "> EPH G07 LNAV\n" + gpsLnavBody("G07");
    const auto path = writeTempFile("libgnss_rinex4_nav_glonass_state.rnx", content);

    const ReadResult result = readNavigation(path);
    ASSERT_TRUE(result.ok) << result.diagnostics;
    EXPECT_TRUE(contains(result.diagnostics,
                         "Skipping RINEX 4 GLONASS EPH R05 FDMA 2025 03 01 12 15 00"))
        << result.diagnostics;

    const auto* glonass = ephemerides(result.nav, GNSSSystem::GLONASS, 5);
    ASSERT_NE(glonass, nullptr);
    ASSERT_EQ(glonass->size(), 1U);
    EXPECT_EQ(glonass->front().toe, GNSSTime(2355, 560718.0));
    ASSERT_NE(ephemerides(result.nav, GNSSSystem::GPS, 7), nullptr);

    std::filesystem::remove(path);
}

TEST(Rinex4NavigationTest, GlonassStatePlausibilityBounds) {
    const Vector3d position(12000.0e3, -18000.0e3, kGlonassZKm * 1e3);
    const Vector3d velocity(1.5e3, 1.5e3, kGlonassVzKmPerS * 1e3);
    EXPECT_TRUE(io::rinex4::isPlausibleGlonassFdmaState(position, velocity));

    // Radius outside the GLONASS shell (25510 km +/- 490 km).
    EXPECT_FALSE(io::rinex4::isPlausibleGlonassFdmaState(position * 0.97, velocity));
    EXPECT_FALSE(io::rinex4::isPlausibleGlonassFdmaState(position * 1.03, velocity));
    // On the shell but with a 500 m/s radial velocity component.
    EXPECT_FALSE(io::rinex4::isPlausibleGlonassFdmaState(
        position, velocity + position.normalized() * 500.0));
    // Non-finite values.
    EXPECT_FALSE(io::rinex4::isPlausibleGlonassFdmaState(
        Vector3d(std::nan(""), 0.0, 0.0), velocity));
}
