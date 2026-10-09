// RINEX observation-code selection regression tests.
//
//  * Among several tracking attributes of one band (GPS 2W/2L/2X, Galileo
//    5X/5I/5Q, BeiDou 2I/2Q/2X ...) the observation that is kept follows a
//    fixed per-system priority (RTKLIB demo5 `codepris`), not the order in
//    which the header lists the observation types.  A rover and a base whose
//    receivers declare the codes in a different order must therefore pair on
//    the same tracking code.
//  * Band 8 (Galileo E5 AltBOC, BeiDou B2a+B2b, both 1191.795 MHz) has no
//    SignalType and must be skipped instead of being filed under E5b / B2a
//    with the wrong wavelength.
#include <gtest/gtest.h>

#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/signal_policy.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/io/rinex.hpp>

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace libgnss;

namespace {

using CodeValues = std::map<std::string, double>;

std::string hdr(const std::string& content, const std::string& label) {
    std::string line = content;
    if (line.size() < 60) line.append(60 - line.size(), ' ');
    return line + label + "\n";
}

std::string field(double value) {
    if (value == 0.0) return std::string(16, ' ');
    char buf[24];
    std::snprintf(buf, sizeof(buf), "%14.3f  ", value);
    return buf;
}

// One RINEX 3.04 file: a single system, one epoch, one satellite.
std::string makeRinex(char system, int prn, const std::vector<std::string>& types,
                      const CodeValues& values) {
    std::string out;
    out += hdr("     3.04           OBSERVATION DATA    M", "RINEX VERSION / TYPE");
    out += hdr("unit test", "PGM / RUN BY / DATE");
    out += hdr("TEST", "MARKER NAME");
    out += hdr("  -3957000.0000  3310000.0000  3737000.0000", "APPROX POSITION XYZ");
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%c  %3d", system, static_cast<int>(types.size()));
    std::string line = buf;
    for (const auto& type : types) line += " " + type;
    out += hdr(line, "SYS / # / OBS TYPES");
    out += hdr("  2024     1     1     0     0    0.0000000     GPS", "TIME OF FIRST OBS");
    out += hdr("", "END OF HEADER");
    out += "> 2024 01 01 00 00  0.0000000  0  1\n";
    std::snprintf(buf, sizeof(buf), "%c%02d", system, prn);
    std::string row = buf;
    for (const auto& type : types) {
        const auto it = values.find(type);
        row += field(it == values.end() ? 0.0 : it->second);
    }
    out += row + "\n";
    return out;
}

ObservationData readOne(const std::string& text, bool preserve_bands) {
    static int counter = 0;
    const auto path = std::filesystem::temp_directory_path() /
                      ("libgnss_prio_" + std::to_string(++counter) + ".obs");
    {
        std::ofstream file(path, std::ios::binary);
        file << text;
    }
    ObservationData epoch;
    io::RINEXReader reader;
    reader.setPreserveAdditionalFrequencyBands(preserve_bands);
    io::RINEXReader::RINEXHeader header;
    if (reader.open(path.string()) && reader.readHeader(header)) {
        reader.readObservationEpoch(epoch);
    }
    std::error_code ec;
    std::filesystem::remove(path, ec);
    return epoch;
}

// signal -> "<pseudorange type>/<phase type>" for every emitted observation.
std::map<SignalType, std::string> chosen(const ObservationData& epoch) {
    std::map<SignalType, std::string> result;
    for (const auto& obs : epoch.observations) {
        result[obs.signal] =
            obs.pseudorange_observation_type + "/" + obs.carrier_phase_observation_type;
    }
    return result;
}

// Every code gets a distinct pseudorange and phase so a wrong pick is visible.
CodeValues valuesFor(const std::vector<std::string>& codes) {
    CodeValues values;
    double k = 1.0;
    for (const auto& code : codes) {
        values["C" + code] = 20000000.0 + 1000.0 * k;
        values["L" + code] = 100000000.0 + 1000.0 * k;
        values["S" + code] = 30.0 + k;
        k += 1.0;
    }
    return values;
}

std::vector<std::string> headerTypes(const std::vector<std::string>& codes, bool reverse) {
    std::vector<std::string> ordered = codes;
    if (reverse) std::reverse(ordered.begin(), ordered.end());
    std::vector<std::string> types;
    for (const auto& code : ordered) types.push_back("C" + code);
    for (const auto& code : ordered) types.push_back("L" + code);
    for (const auto& code : ordered) types.push_back("S" + code);
    return types;
}

// Reads the same measurements with the codes declared in every cyclic
// rotation and the reversed order of the header, in default and preserve-bands
// mode, and checks the chosen codes never change.  Returns the (single)
// outcome of the default-mode and preserve-mode reads.
struct Outcome {
    std::map<SignalType, std::string> normal;
    std::map<SignalType, std::string> preserved;
};

Outcome readAllOrders(char system, int prn, const std::vector<std::string>& codes) {
    const CodeValues values = valuesFor(codes);
    Outcome first;
    bool have_first = false;
    for (int reverse = 0; reverse < 2; ++reverse) {
        for (size_t rotation = 0; rotation < codes.size(); ++rotation) {
            std::vector<std::string> rotated = codes;
            std::rotate(rotated.begin(), rotated.begin() + static_cast<long>(rotation),
                        rotated.end());
            const std::string text =
                makeRinex(system, prn, headerTypes(rotated, reverse != 0), values);
            Outcome current;
            current.normal = chosen(readOne(text, false));
            current.preserved = chosen(readOne(text, true));
            if (!have_first) {
                first = current;
                have_first = true;
            } else {
                EXPECT_EQ(current.normal, first.normal)
                    << system << " header rotation " << rotation << " reverse " << reverse;
                EXPECT_EQ(current.preserved, first.preserved)
                    << system << " header rotation " << rotation << " reverse " << reverse;
            }
        }
    }
    return first;
}

std::string pair(const std::string& code) { return "C" + code + "/L" + code; }

}  // namespace

TEST(RinexCodePriorityTest, GpsL2PrefersWOverLXSRegardlessOfHeaderOrder) {
    const auto out = readAllOrders('G', 5, {"2L", "2W", "2X", "2S"});
    ASSERT_EQ(out.normal.count(SignalType::GPS_L2C), 1U);
    EXPECT_EQ(out.normal.at(SignalType::GPS_L2C), pair("2W"));
    EXPECT_EQ(out.preserved.at(SignalType::GPS_L2C), pair("2W"));
}

TEST(RinexCodePriorityTest, GpsL2WithoutWFollowsDemo5Order) {
    // demo5 GPS L2 "CPYWMNDLSX": L before S before X.
    const auto out = readAllOrders('G', 5, {"2X", "2S", "2L"});
    EXPECT_EQ(out.normal.at(SignalType::GPS_L2C), pair("2L"));
    const auto out2 = readAllOrders('G', 5, {"2X", "2S"});
    EXPECT_EQ(out2.normal.at(SignalType::GPS_L2C), pair("2S"));
}

TEST(RinexCodePriorityTest, GpsL1PrefersCOverPAndW) {
    const auto out = readAllOrders('G', 5, {"1W", "1C", "1X"});
    EXPECT_EQ(out.normal.at(SignalType::GPS_L1CA), pair("1C"));
}

TEST(RinexCodePriorityTest, GpsL2FallsBackWhenPreferredCodeIsMissingInTheEpoch) {
    // Header declares 2W but this satellite only has 2L data.
    const std::vector<std::string> types = {"C2W", "C2L", "L2W", "L2L"};
    const CodeValues values = {{"C2L", 22000000.0}, {"L2L", 110000000.0}};
    const auto got = chosen(readOne(makeRinex('G', 7, types, values), false));
    EXPECT_EQ(got.at(SignalType::GPS_L2C), pair("2L"));
}

TEST(RinexCodePriorityTest, GpsL5PrefersIOverQOverX) {
    const auto out = readAllOrders('G', 5, {"5X", "5Q", "5I"});
    EXPECT_EQ(out.preserved.at(SignalType::GPS_L5), pair("5I"));
}

TEST(RinexCodePriorityTest, GalileoE1E5aE5bPriority) {
    const auto e1 = readAllOrders('E', 11, {"1X", "1B", "1C"});
    EXPECT_EQ(e1.normal.at(SignalType::GAL_E1), pair("1C"));  // "CABXZ"

    const auto e5a = readAllOrders('E', 11, {"5Q", "5I", "5X"});
    EXPECT_EQ(e5a.normal.at(SignalType::GAL_E5A), pair("5X"));  // "XIQ"
    EXPECT_EQ(e5a.preserved.at(SignalType::GAL_E5A), pair("5X"));

    // E5b only shows up as a separate observation in preserve-bands mode.
    const auto e5b = readAllOrders('E', 11, {"7Q", "7X", "7I"});
    EXPECT_EQ(e5b.preserved.at(SignalType::GAL_E5B), pair("7X"));
}

TEST(RinexCodePriorityTest, BeiDouB1IB2IB1CB2aPriority) {
    const auto b1i = readAllOrders('C', 20, {"2X", "2Q", "2I"});
    EXPECT_EQ(b1i.normal.at(SignalType::BDS_B1I), pair("2I"));  // "IQX"

    const auto b2i = readAllOrders('C', 20, {"7P", "7X", "7Q", "7I", "7D"});
    EXPECT_EQ(b2i.normal.at(SignalType::BDS_B2I), pair("7I"));  // "IQXDPZ"
    const auto b2b = readAllOrders('C', 20, {"7P", "7Z", "7D"});
    EXPECT_EQ(b2b.normal.at(SignalType::BDS_B2I), pair("7D"));

    const auto b2a = readAllOrders('C', 20, {"5X", "5P", "5D"});
    EXPECT_EQ(b2a.preserved.at(SignalType::BDS_B2A), pair("5D"));  // "DPX"

    const auto b1c = readAllOrders('C', 20, {"1X", "1P", "1D"});
    EXPECT_EQ(b1c.normal.at(SignalType::BDS_B1C), pair("1D"));  // "DPXSLZAN"
}

TEST(RinexCodePriorityTest, GlonassPrefersCaOverP) {
    const auto g1 = readAllOrders('R', 3, {"1P", "1C"});
    EXPECT_EQ(g1.normal.at(SignalType::GLO_L1CA), pair("1C"));
    EXPECT_EQ(g1.normal.count(SignalType::GLO_L1P), 0U);
    const auto g2 = readAllOrders('R', 3, {"2P", "2C"});
    EXPECT_EQ(g2.normal.at(SignalType::GLO_L2CA), pair("2C"));
    EXPECT_EQ(g2.normal.count(SignalType::GLO_L2P), 0U);
}

TEST(RinexCodePriorityTest, QzssPriority) {
    const auto l1 = readAllOrders('J', 2, {"1L", "1X", "1C", "1S"});
    EXPECT_EQ(l1.normal.at(SignalType::QZS_L1CA), pair("1C"));  // "CLSXZBE"
    const auto l2 = readAllOrders('J', 2, {"2X", "2S", "2L"});
    EXPECT_EQ(l2.normal.at(SignalType::QZS_L2C), pair("2L"));  // "LSX"
    const auto l5 = readAllOrders('J', 2, {"5X", "5Q", "5I"});
    EXPECT_EQ(l5.preserved.at(SignalType::QZS_L5), pair("5I"));  // "IQXDPZ"
}

TEST(RinexCodePriorityTest, SingleCodePerBandStillSelected) {
    // Unchanged behaviour: one code per band is simply used.
    const auto out = readAllOrders('G', 5, {"2W"});
    EXPECT_EQ(out.normal.at(SignalType::GPS_L2C), pair("2W"));
}

TEST(RinexCodePriorityTest, UnlistedAttributeLosesToEveryListedOne) {
    // 'Q' is not in GPS L2's priority string; it only wins when alone, and
    // never depends on the header order.
    const auto both = readAllOrders('G', 5, {"2Q", "2X"});
    EXPECT_EQ(both.normal.at(SignalType::GPS_L2C), pair("2X"));
    const auto alone = readAllOrders('G', 5, {"2Q"});
    EXPECT_EQ(alone.normal.at(SignalType::GPS_L2C), pair("2Q"));
}

TEST(RinexCodePriorityTest, GalileoBand8IsSkippedNotFiledUnderE5b) {
    const std::vector<std::string> codes = {"1C", "5Q", "7Q", "8Q"};
    const CodeValues values = valuesFor(codes);
    for (bool preserve : {false, true}) {
        const auto epoch =
            readOne(makeRinex('E', 11, headerTypes(codes, false), values), preserve);
        std::set<SignalType> signals;
        for (const auto& obs : epoch.observations) {
            EXPECT_TRUE(signals.insert(obs.signal).second)
                << "duplicate (sat, signal) preserve=" << preserve;
            EXPECT_NE(obs.pseudorange_observation_type, "C8Q");
            EXPECT_NE(obs.carrier_phase_observation_type, "L8Q");
            // Every kept observation carries its own band's frequency.
            if (obs.signal == SignalType::GAL_E5B) {
                EXPECT_EQ(obs.pseudorange_observation_type, "C7Q");
                EXPECT_NEAR(signalFrequencyHz(obs), 1207.140e6, 1.0);
            }
        }
        EXPECT_EQ(signals.count(SignalType::GAL_E1), 1U);
        EXPECT_EQ(signals.count(SignalType::GAL_E5A), 1U);
        EXPECT_EQ(signals.count(SignalType::GAL_E5B), preserve ? 1U : 0U);
    }
}

TEST(RinexCodePriorityTest, GalileoBand8AloneProducesNoSecondObservation) {
    const std::vector<std::string> codes = {"1C", "8X"};
    const CodeValues values = valuesFor(codes);
    for (bool preserve : {false, true}) {
        const auto epoch =
            readOne(makeRinex('E', 11, headerTypes(codes, false), values), preserve);
        ASSERT_EQ(epoch.observations.size(), 1U) << "preserve=" << preserve;
        EXPECT_EQ(epoch.observations[0].signal, SignalType::GAL_E1);
    }
}

TEST(RinexCodePriorityTest, BeiDouBand8IsSkippedNotFiledUnderB2a) {
    const std::vector<std::string> codes = {"2I", "5X", "7I", "8X"};
    const CodeValues values = valuesFor(codes);
    for (bool preserve : {false, true}) {
        const auto epoch =
            readOne(makeRinex('C', 20, headerTypes(codes, false), values), preserve);
        std::set<SignalType> signals;
        for (const auto& obs : epoch.observations) {
            EXPECT_TRUE(signals.insert(obs.signal).second)
                << "duplicate (sat, signal) preserve=" << preserve;
            EXPECT_NE(obs.pseudorange_observation_type, "C8X");
            EXPECT_NE(obs.carrier_phase_observation_type, "L8X");
            if (obs.signal == SignalType::BDS_B2A) {
                EXPECT_EQ(obs.pseudorange_observation_type, "C5X");
                EXPECT_NEAR(signalFrequencyHz(obs), 1176.45e6, 1.0);
            }
        }
        EXPECT_EQ(signals.count(SignalType::BDS_B1I), 1U);
        EXPECT_EQ(signals.count(SignalType::BDS_B2I), 1U);
        EXPECT_EQ(signals.count(SignalType::BDS_B2A), preserve ? 1U : 0U);
    }
}

TEST(RinexCodePriorityTest, Band8DoesNotMapToAnySignal) {
    SignalType signal = SignalType::SIGNAL_TYPE_COUNT;
    EXPECT_FALSE(signal_policy::trySignalForObservationType(GNSSSystem::Galileo, "C8Q", signal));
    EXPECT_FALSE(signal_policy::trySignalForObservationType(GNSSSystem::BeiDou, "C8X", signal));
    // The neighbouring bands keep their mapping.
    EXPECT_TRUE(signal_policy::trySignalForObservationType(GNSSSystem::Galileo, "C7Q", signal));
    EXPECT_EQ(signal, SignalType::GAL_E5B);
    EXPECT_TRUE(signal_policy::trySignalForObservationType(GNSSSystem::BeiDou, "C5P", signal));
    EXPECT_EQ(signal, SignalType::BDS_B2A);
}

TEST(RinexCodePriorityTest, TrackingAttributeRankFollowsDemo5Strings) {
    using signal_policy::trackingAttributeRank;
    EXPECT_LT(trackingAttributeRank(GNSSSystem::GPS, 2, 'W'),
              trackingAttributeRank(GNSSSystem::GPS, 2, 'L'));
    EXPECT_LT(trackingAttributeRank(GNSSSystem::GPS, 2, 'S'),
              trackingAttributeRank(GNSSSystem::GPS, 2, 'X'));
    EXPECT_EQ(trackingAttributeRank(GNSSSystem::GPS, 2, 'Q'),
              signal_policy::kUnlistedTrackingRank);
    EXPECT_LT(trackingAttributeRank(GNSSSystem::BeiDou, 1, 'D'),
              trackingAttributeRank(GNSSSystem::BeiDou, 1, 'X'));
    EXPECT_EQ(trackingAttributeRank(GNSSSystem::Galileo, 8, 'X'),
              signal_policy::kUnlistedTrackingRank);
}
