// RINEX 4 navigation reader coverage with synthetic mixed-constellation
// files.  Every record below is generated for this test from the RINEX 4.02
// navigation message field layouts (no third-party navigation data is
// embedded): GPS/QZSS LNAV and CNAV, Galileo I/NAV and F/NAV, BeiDou D1 and
// B-CNAV1/2/3, GLONASS FDMA (with the RINEX 4 fifth orbit line) and SBAS.

#include <gtest/gtest.h>
#include <libgnss++/io/rinex.hpp>
#include <libgnss++/io/rinex4.hpp>

#include <algorithm>
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
// week of 3 and an IODE of 0.25; they belong in cnav_ephemeris_data only.
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
           orbitLine({554418.0, 2355.0});
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

// BeiDou B-CNAV body; epoch and toe are BDT (BDT week 999 = GPS week 2355).
// CNV1 / CNV2 have ten lines, CNV3 nine.  Blank columns are the spare / other
// signal fields; the short lines exercise writers that drop trailing blanks.
std::string beidouCnavBody(const std::string& satellite, const std::string& message) {
    const std::string blank(19, ' ');
    std::string body =
        epochLine(satellite, "2025 03 01 11 59 46", -9.4e-04, 3.7e-13, 0.0) +
        orbitLine({-2.28e-03, -146.88, 3.5e-09, 0.698}) +       // ADOT Crs dn0 M0
        orbitLine({-7.16e-06, 6.75e-04, 1.89e-06, 5282.63}) +    // Cuc e Cus sqrtA
        orbitLine({561586.0, 5.12e-08, 1.874, 5.68e-08}) +       // toe Cic OMEGA0 Cis
        orbitLine({0.9846, 334.996, -0.4104, -6.94e-09}) +       // i0 Crc omega OMEGAdot
        orbitLine({-1.925e-10, 1.578e-14, 3.0, 561586.0}) +      // IDOT dn0dot SatType t_op
        orbitLine({0.0, -5.0, 0.0, -1.0});                       // SISAI oe ocb oc1 oc2
    if (message == "CNV1") {
        body += "    " + field(-8.7e-10) + blank + field(9.5e-09) + field(1.2e-09) + "\n";
    } else if (message == "CNV2") {
        body += "    " + blank + field(-2.7e-09) + field(9.5e-09) + field(1.2e-09) + "\n";
    }
    if (message == "CNV3") {
        body += orbitLine({3.0, 0.0, 0.0, 1.9e-09}) + orbitLine({561000.0});
    } else {
        body += orbitLine({3.0, 0.0, 0.0, 12.0}) +
                "    " + field(561000.0) + blank + blank + field(12.0) + "\n";
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
    content += "> EPH C25 CNV1\n" + beidouCnavBody("C25", "CNV1");
    content += "> EPH C25 CNV2\n" + beidouCnavBody("C25", "CNV2");
    content += "> EPH C25 CNV3\n" + beidouCnavBody("C25", "CNV3");
    content += "> EPH C25 D1\n" + beidouD1Body("C25");
    content += "> EPH R05 FDMA\n" +
               glonassFdmaBody("R05", "2025 03 01 11 45 00", kGlonassZKm, kGlonassVzKmPerS);
    content += "> EPH S27 SBAS\n" + sbasBody("S27");
    const auto path = writeTempFile("libgnss_rinex4_nav_all_types.rnx", content);

    const ReadResult result = readNavigation(path);
    ASSERT_TRUE(result.ok) << result.diagnostics;

    // Every record is read; none is reported as skipped.
    EXPECT_FALSE(contains(result.diagnostics, "Skipping")) << result.diagnostics;

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

    // CNAV / B-CNAV records live in cnav_ephemeris_data, never next to the
    // LNAV / D1 records above.
    for (const GNSSSystem system : {GNSSSystem::GPS, GNSSSystem::QZSS}) {
        const auto it = result.nav.cnav_ephemeris_data.find(
            SatelliteId(system, system == GNSSSystem::GPS ? 7 : 2));
        ASSERT_NE(it, result.nav.cnav_ephemeris_data.end());
        ASSERT_EQ(it->second.size(), 1U);
        const Ephemeris& eph = it->second.front();
        EXPECT_EQ(eph.navigation_message_type, NavigationMessageType::CNAV);
        // CNAV has no toe field: toe = toc.
        EXPECT_EQ(eph.toc, GNSSTime(2355, 561600.0));
        EXPECT_EQ(eph.toe, GNSSTime(2355, 561600.0));
        EXPECT_EQ(eph.tof, GNSSTime(2355, 554418.0));
        EXPECT_DOUBLE_EQ(eph.toes, 561600.0);
        EXPECT_EQ(eph.week, 2355);
        EXPECT_EQ(eph.iode, 0);
        EXPECT_DOUBLE_EQ(eph.sqrt_a, 5153.65);
        EXPECT_DOUBLE_EQ(eph.delta_n, 4.5e-09);
        EXPECT_DOUBLE_EQ(eph.tgd, -1.0e-08);
        EXPECT_EQ(eph.health, 0);
        ASSERT_TRUE(eph.cnav_data.has_value());
        const CnavNavigationData& cnav = *eph.cnav_data;
        EXPECT_DOUBLE_EQ(cnav.a_dot, 0.25);
        EXPECT_DOUBLE_EQ(cnav.delta_n0_dot, -1.0e-13);
        EXPECT_DOUBLE_EQ(cnav.t_op, 561600.0);
        EXPECT_EQ(cnav.urai_ned0, 3);
        EXPECT_EQ(cnav.urai_ned1, 2);
        EXPECT_EQ(cnav.urai_ed, 1);
        EXPECT_EQ(cnav.urai_ned2, 1);
        EXPECT_EQ(cnav.wn_op, 2355);
        EXPECT_FALSE(cnav.integrity_flags.has_value());
        ASSERT_TRUE(cnav.isc_l1ca.has_value());
        EXPECT_DOUBLE_EQ(*cnav.isc_l1ca, 1.5e-09);
        EXPECT_DOUBLE_EQ(*cnav.isc_l5q5, 4.0e-09);
        EXPECT_FALSE(cnav.isc_l1cd.has_value());
    }

    const auto bds_cnav = result.nav.cnav_ephemeris_data.find(SatelliteId(GNSSSystem::BeiDou, 25));
    ASSERT_NE(bds_cnav, result.nav.cnav_ephemeris_data.end());
    ASSERT_EQ(bds_cnav->second.size(), 3U);
    for (const Ephemeris& eph : bds_cnav->second) {
        const NavigationMessageType type = eph.navigation_message_type;
        SCOPED_TRACE(io::rinex4::navigationMessageTypeName(type));
        // BDT 2025-03-01 11:59:46 (week 999, 561586 s) is GPST week 2355, 561600 s.
        EXPECT_EQ(eph.toc, GNSSTime(2355, 561600.0));
        EXPECT_EQ(eph.toe, GNSSTime(2355, 561600.0));
        EXPECT_EQ(eph.tof, GNSSTime(2355, 561014.0));
        EXPECT_DOUBLE_EQ(eph.toes, 561586.0);
        EXPECT_EQ(eph.week, 999);
        ASSERT_TRUE(eph.cnav_data.has_value());
        const CnavNavigationData& cnav = *eph.cnav_data;
        EXPECT_DOUBLE_EQ(cnav.a_dot, -2.28e-03);
        EXPECT_DOUBLE_EQ(cnav.delta_n0_dot, 1.578e-14);
        EXPECT_DOUBLE_EQ(cnav.t_op, 561586.0);
        EXPECT_EQ(cnav.satellite_type, 3);
        EXPECT_EQ(cnav.sisai_ocb, -5);
        EXPECT_EQ(cnav.sisai_oc2, -1);
        EXPECT_EQ(cnav.sismai, 3);
        EXPECT_EQ(cnav.integrity_flags, 0);
        if (type == NavigationMessageType::CNV3) {
            EXPECT_DOUBLE_EQ(eph.tgd, 1.9e-09);
            EXPECT_EQ(eph.iode, 0);
            EXPECT_FALSE(cnav.isc_b1cd.has_value());
            EXPECT_FALSE(cnav.isc_b2ad.has_value());
            continue;
        }
        EXPECT_DOUBLE_EQ(eph.tgd, 9.5e-09);
        EXPECT_DOUBLE_EQ(eph.tgd_secondary, 1.2e-09);
        EXPECT_EQ(eph.iode, 12);
        EXPECT_EQ(eph.iodc, 12);
        if (type == NavigationMessageType::CNV1) {
            ASSERT_TRUE(cnav.isc_b1cd.has_value());
            EXPECT_DOUBLE_EQ(*cnav.isc_b1cd, -8.7e-10);
            EXPECT_FALSE(cnav.isc_b2ad.has_value());
        } else {
            EXPECT_EQ(type, NavigationMessageType::CNV2);
            ASSERT_TRUE(cnav.isc_b2ad.has_value());
            EXPECT_DOUBLE_EQ(*cnav.isc_b2ad, -2.7e-09);
            EXPECT_FALSE(cnav.isc_b1cd.has_value());
        }
    }

    // Explicit selection by message type and IODE.
    const SatelliteId c25(GNSSSystem::BeiDou, 25);
    const GNSSTime query(2355, 562000.0);
    for (const auto type : {NavigationMessageType::CNV1, NavigationMessageType::CNV2,
                            NavigationMessageType::CNV3}) {
        const Ephemeris* eph = result.nav.getCnavEphemeris(c25, query, type);
        ASSERT_NE(eph, nullptr);
        EXPECT_EQ(eph->navigation_message_type, type);
    }
    EXPECT_NE(result.nav.getCnavEphemeris(c25, query, NavigationMessageType::CNV1, 12), nullptr);
    EXPECT_EQ(result.nav.getCnavEphemeris(c25, query, NavigationMessageType::CNV1, 13), nullptr);
    EXPECT_EQ(result.nav.getCnavEphemeris(c25, query, NavigationMessageType::D1), nullptr);
    EXPECT_EQ(result.nav.getCnavEphemeris(c25, GNSSTime(2356, 561600.0),
                                          NavigationMessageType::CNV1),
              nullptr);
    EXPECT_EQ(result.nav.getCnavEphemeris(SatelliteId(GNSSSystem::GPS, 7), query,
                                          NavigationMessageType::CNV1),
              nullptr);
    // The legacy selection never returns a CNAV record.
    EXPECT_EQ(result.nav.getEphemeris(c25, query)->navigation_message_type,
              NavigationMessageType::D1);

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

TEST(Rinex4NavigationTest, CnavRateTermsChangeOnlyRadiusAndMeanMotion) {
    // A circular orbit with no harmonic corrections isolates the two CNAV
    // terms: ADOT scales the radius by A0 + ADOT tk, delta-n0-dot advances
    // the argument of latitude by delta-n0-dot tk^2 / 2.
    Ephemeris eph;
    eph.satellite = SatelliteId(GNSSSystem::GPS, 7);
    eph.toe = GNSSTime(2355, 561600.0);
    eph.toc = eph.toe;
    eph.toes = 561600.0;
    eph.sqrt_a = 5153.65;
    eph.e = 0.0;
    eph.i0 = 0.96;
    eph.omega0 = 1.2;
    eph.omega = 0.4;
    eph.m0 = 0.3;
    eph.delta_n = 4.5e-09;
    eph.omega_dot = -7.9e-09;
    eph.valid = true;

    const double tk = 3600.0;
    const GNSSTime time = eph.toe + tk;
    Vector3d legacy_pos, legacy_vel;
    double legacy_clk = 0.0, legacy_drift = 0.0;
    ASSERT_TRUE(eph.calculateSatelliteState(time, legacy_pos, legacy_vel, legacy_clk, legacy_drift));

    // Zero rates: identical to the legacy computation.
    Ephemeris zero = eph;
    zero.cnav_data = CnavNavigationData{};
    Vector3d pos, vel;
    double clk = 0.0, drift = 0.0;
    ASSERT_TRUE(zero.calculateSatelliteState(time, pos, vel, clk, drift));
    EXPECT_EQ(pos, legacy_pos);
    EXPECT_EQ(clk, legacy_clk);

    Ephemeris adot = zero;
    adot.cnav_data->a_dot = 0.25;
    ASSERT_TRUE(adot.calculateSatelliteState(time, pos, vel, clk, drift));
    EXPECT_NEAR(pos.norm() - legacy_pos.norm(), 0.25 * tk, 1e-6);
    EXPECT_NEAR(std::atan2(pos.cross(legacy_pos).norm(), pos.dot(legacy_pos)), 0.0, 1e-12);

    Ephemeris dndot = zero;
    dndot.cnav_data->delta_n0_dot = -1.0e-13;
    ASSERT_TRUE(dndot.calculateSatelliteState(time, pos, vel, clk, drift));
    EXPECT_NEAR(pos.norm(), legacy_pos.norm(), 1e-6);
    const double angle = std::atan2(pos.cross(legacy_pos).norm(), pos.dot(legacy_pos));
    EXPECT_NEAR(angle, 0.5 * 1.0e-13 * tk * tk, 1e-12);
}

TEST(Rinex4NavigationTest, CnavSatelliteStatesMatchCssrlib) {
    // Reference values: cssrlib (MIT) rnxdec.decode_nav + ephemeris.eph2pos
    // on these same synthetic records, toe -1 h and +1.5 h.
    const auto path = writeTempFile(
        "libgnss_rinex4_nav_cnav_cssrlib.rnx",
        rinex4Header() + "> EPH G07 CNAV\n" + cnavBody("G07") + "> EPH C25 CNV1\n" +
            beidouCnavBody("C25", "CNV1"));
    const ReadResult result = readNavigation(path);
    ASSERT_TRUE(result.ok) << result.diagnostics;

    struct Expected {
        SatelliteId satellite;
        NavigationMessageType type;
        double dt;
        Vector3d position;
        double clock;
    };
    const std::vector<Expected> expected = {
        {SatelliteId(GNSSSystem::GPS, 7), NavigationMessageType::CNAV, -3600.0,
         Vector3d(-16514324.467706, 1087778.919311, 20599956.345308), 1.233866935400717e-04},
        {SatelliteId(GNSSSystem::GPS, 7), NavigationMessageType::CNAV, 5400.0,
         Vector3d(-24810074.002564, -9599879.005788, -1262486.485955), 1.233923675452283e-04},
        {SatelliteId(GNSSSystem::BeiDou, 25), NavigationMessageType::CNV1, -3600.0,
         Vector3d(9262061.753319, -25898344.438093, -4608463.356392), -9.400016631368170e-04},
        {SatelliteId(GNSSSystem::BeiDou, 25), NavigationMessageType::CNV1, 5400.0,
         Vector3d(9969602.072164, -16933349.478383, 19810593.302954), -9.399995705322118e-04},
    };
    for (const auto& item : expected) {
        SCOPED_TRACE(item.satellite.toString() + " dt " + std::to_string(item.dt));
        const GNSSTime time = GNSSTime(2355, 561600.0) + item.dt;
        const Ephemeris* eph = result.nav.getCnavEphemeris(item.satellite, time, item.type);
        ASSERT_NE(eph, nullptr);
        Vector3d pos, vel;
        double clk = 0.0, drift = 0.0;
        ASSERT_TRUE(eph->calculateSatelliteState(time, pos, vel, clk, drift));
        EXPECT_LT((pos - item.position).norm(), 1e-4);
        EXPECT_NEAR(clk, item.clock, 1e-15);
    }

    std::filesystem::remove(path);
}

TEST(Rinex4NavigationTest, BeiDouSourceBCnav1SelectsBCnav1ByIode) {
    std::string content = rinex4Header();
    content += "> EPH C25 D1\n" + beidouD1Body("C25");
    content += "> EPH C25 CNV1\n" + beidouCnavBody("C25", "CNV1");
    content += "> EPH C25 CNV2\n" + beidouCnavBody("C25", "CNV2");
    const auto path = writeTempFile("libgnss_rinex4_nav_bds_source.rnx", content);
    ReadResult result = readNavigation(path);
    ASSERT_TRUE(result.ok) << result.diagnostics;
    NavigationData& nav = result.nav;
    const SatelliteId c25(GNSSSystem::BeiDou, 25);
    const GNSSTime time(2355, 562000.0);

    // Default: the D1 record; an SSR IODE is matched against its toe index.
    ASSERT_NE(nav.getEphemeris(c25, time), nullptr);
    EXPECT_EQ(nav.getEphemeris(c25, time)->navigation_message_type, NavigationMessageType::D1);
    EXPECT_EQ(nav.getEphemeris(c25, time, 12), nullptr);

    nav.setBeiDouEphemerisSource(NavigationData::BeiDouEphemerisSource::BCnav1);
    ASSERT_NE(nav.getEphemeris(c25, time), nullptr);
    EXPECT_EQ(nav.getEphemeris(c25, time)->navigation_message_type, NavigationMessageType::CNV1);
    const Ephemeris* matched = nav.getEphemeris(c25, time, 12);
    ASSERT_NE(matched, nullptr);
    EXPECT_EQ(matched->navigation_message_type, NavigationMessageType::CNV1);
    EXPECT_EQ(nav.getEphemeris(c25, time, 13), nullptr);
    // The satellite state comes from the B-CNAV1 record, CNAV terms included.
    Vector3d pos, vel, expected_pos, expected_vel;
    double clk = 0.0, drift = 0.0, expected_clk = 0.0, expected_drift = 0.0;
    ASSERT_TRUE(nav.calculateSatelliteState(c25, time, pos, vel, clk, drift, 12));
    ASSERT_TRUE(matched->calculateSatelliteState(time, expected_pos, expected_vel, expected_clk,
                                                 expected_drift));
    EXPECT_EQ(pos, expected_pos);
    EXPECT_EQ(clk, expected_clk);

    // GPS and the other systems are unaffected.
    nav.setBeiDouEphemerisSource(NavigationData::BeiDouEphemerisSource::D1D2);
    EXPECT_EQ(nav.getEphemeris(c25, time)->navigation_message_type, NavigationMessageType::D1);
    std::filesystem::remove(path);
}
