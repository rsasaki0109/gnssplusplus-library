// RINEX 2.x observation reader regression tests.
//
// One test (or small group) per audited defect:
//   2a  "# / TYPES OF OBSERV" with more than 9 types (continuation records)
//   2b  more than 12 satellites per epoch (satellite list continuation)
//   2c  satellite system letters other than G/R/E/C
//   2d  epoch flags 2-6 (special / cycle-slip records)
//   2e  blank and right-trimmed observation rows
//   2f  deterministic C1/P1 and C2/P2 selection
#include <gtest/gtest.h>

#include <libgnss++/core/observation.hpp>
#include <libgnss++/io/rinex.hpp>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

using namespace libgnss;

namespace {

using Value = std::optional<double>;

std::string headerLine(const std::string& content, const std::string& label) {
    std::string line = content;
    if (line.size() < 60) line.append(60 - line.size(), ' ');
    return line + label + "\n";
}

// RINEX 2.x header with the "# / TYPES OF OBSERV" record split at 9 types.
std::string rinex2Header(const std::vector<std::string>& types,
                         const std::string& version = "2.11") {
    std::string out;
    out += headerLine("     " + version + "           OBSERVATION DATA    M (MIXED)",
                      "RINEX VERSION / TYPE");
    out += headerLine("unit test", "PGM / RUN BY / DATE");
    out += headerLine("TEST", "MARKER NAME");
    out += headerLine("  -3957000.0000  3310000.0000  3737000.0000", "APPROX POSITION XYZ");
    for (size_t i = 0; i < types.size(); i += 9) {
        char count[8];
        std::snprintf(count, sizeof(count), "%6d", static_cast<int>(types.size()));
        std::string content = (i == 0) ? std::string(count) : std::string(6, ' ');
        for (size_t j = i; j < types.size() && j < i + 9; ++j) {
            content += "    " + types[j];
        }
        out += headerLine(content, "# / TYPES OF OBSERV");
    }
    out += headerLine("  2024     1     1     0     0    0.0000000     GPS", "TIME OF FIRST OBS");
    out += headerLine("", "END OF HEADER");
    return out;
}

// Epoch record: 12 satellite ids on the first line, the rest on continuation
// lines.  `pad_to` right-pads every record; `clock` writes a receiver clock
// offset (F12.9) into cols 69-80 of the first record.
std::string epochRecord(int sec_of_minute,
                        int flag,
                        const std::vector<std::string>& sats,
                        size_t pad_to = 0,
                        std::optional<double> clock = std::nullopt,
                        int count = -1) {
    char head[64];
    std::snprintf(head, sizeof(head), " 24  1  1  0  0%11.7f  %d%3d",
                  static_cast<double>(sec_of_minute), flag,
                  count >= 0 ? count : static_cast<int>(sats.size()));
    std::string out;
    std::string line = head;
    for (size_t i = 0; i < sats.size() && i < 12; ++i) line += sats[i];
    if (clock) {
        line.resize(68, ' ');
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%12.9f", *clock);
        line += buf;
    } else if (pad_to > 0 && line.size() < pad_to) {
        line.resize(pad_to, ' ');
    }
    out += line + "\n";
    for (size_t i = 12; i < sats.size(); i += 12) {
        std::string cont(32, ' ');
        for (size_t j = i; j < sats.size() && j < i + 12; ++j) cont += sats[j];
        if (pad_to > 0 && cont.size() < pad_to) cont.resize(pad_to, ' ');
        out += cont + "\n";
    }
    return out;
}

std::string obsRows(const std::vector<Value>& values, bool trim_rows = false) {
    std::string out;
    for (size_t i = 0; i < values.size(); i += 5) {
        std::string row;
        for (size_t j = i; j < i + 5 && j < values.size(); ++j) {
            if (values[j]) {
                char buf[24];
                std::snprintf(buf, sizeof(buf), "%14.3f  ", *values[j]);
                row += buf;
            } else {
                row += std::string(16, ' ');
            }
        }
        if (trim_rows) {
            const size_t last = row.find_last_not_of(' ');
            row = (last == std::string::npos) ? std::string() : row.substr(0, last + 1);
        }
        out += row + "\n";
    }
    return out;
}

struct ReadResult {
    bool header_ok = false;
    io::RINEXReader::RINEXHeader header;
    std::vector<ObservationData> epochs;
};

ReadResult readText(const std::string& text,
                    const std::string& name,
                    bool preserve_bands = false) {
    const auto path = std::filesystem::temp_directory_path() / ("libgnss_rinex2_" + name + ".obs");
    {
        std::ofstream file(path, std::ios::binary);
        file << text;
    }
    ReadResult result;
    io::RINEXReader reader;
    reader.setPreserveAdditionalFrequencyBands(preserve_bands);
    if (reader.open(path.string())) {
        result.header_ok = reader.readHeader(result.header);
        ObservationData epoch;
        while (result.header_ok && reader.readObservationEpoch(epoch)) {
            result.epochs.push_back(epoch);
        }
    }
    std::error_code ec;
    std::filesystem::remove(path, ec);
    return result;
}

const Observation* findObs(const ObservationData& epoch,
                           GNSSSystem system,
                           int prn,
                           int band_rank /* n-th observation of the satellite: 0 = primary */) {
    const SatelliteId sat(system, static_cast<uint8_t>(prn));
    for (const auto& obs : epoch.observations) {
        if (obs.satellite == sat) {
            if (band_rank == 0) return &obs;
            --band_rank;
        }
    }
    return nullptr;
}

size_t countSystem(const ObservationData& epoch, GNSSSystem system) {
    size_t n = 0;
    for (const auto& obs : epoch.observations) {
        if (obs.satellite.system == system) ++n;
    }
    return n;
}

}  // namespace

// 2a ---------------------------------------------------------------------
TEST(Rinex2ReaderTest, ParsesMoreThanNineObservationTypesAcrossHeaderRecords) {
    const std::vector<std::string> types = {"C1", "L1", "L2", "P2", "S1", "S2",
                                            "D1", "D2", "C2", "P1", "L5", "C5"};
    std::string text = rinex2Header(types);
    text += epochRecord(0, 0, {"G01", "G02"});
    for (int g = 1; g <= 2; ++g) {
        std::vector<Value> v;
        for (size_t i = 0; i < types.size(); ++i) {
            v.push_back(20000000.0 + g * 1000.0 + static_cast<double>(i) * 100.0);
        }
        text += obsRows(v);
    }

    const ReadResult r = readText(text, "types12");
    ASSERT_TRUE(r.header_ok);
    EXPECT_EQ(r.header.observation_types, types);
    ASSERT_EQ(r.epochs.size(), 1U);

    // Both satellites decode: values sit on the third row (L5/C5) too.
    const Observation* g1 = findObs(r.epochs[0], GNSSSystem::GPS, 1, 0);
    const Observation* g2 = findObs(r.epochs[0], GNSSSystem::GPS, 2, 0);
    ASSERT_NE(g1, nullptr);
    ASSERT_NE(g2, nullptr);
    EXPECT_DOUBLE_EQ(g1->pseudorange, 20001000.0);        // C1 (index 0)
    EXPECT_DOUBLE_EQ(g1->carrier_phase, 20001100.0);      // L1 (index 1)
    EXPECT_DOUBLE_EQ(g2->pseudorange, 20002000.0);

    // A file with a 10th+ type must also expose the types located on the
    // continuation record (L5/C5 are types 11 and 12).
    const ReadResult r5 = readText(text, "types12_bands", true);
    ASSERT_EQ(r5.epochs.size(), 1U);
    bool saw_l5 = false;
    for (const auto& obs : r5.epochs[0].observations) {
        if (obs.satellite == SatelliteId(GNSSSystem::GPS, 1) &&
            obs.carrier_phase_observation_type == "L5") {
            saw_l5 = true;
            EXPECT_DOUBLE_EQ(obs.carrier_phase, 20001000.0 + 1000.0);   // index 10
            EXPECT_DOUBLE_EQ(obs.pseudorange, 20001000.0 + 1100.0);     // index 11
        }
    }
    EXPECT_TRUE(saw_l5);
}

TEST(Rinex2ReaderTest, ParsesExactlyNineAndTwentyOneObservationTypes) {
    std::vector<std::string> nine = {"C1", "L1", "S1", "P2", "L2", "S2", "D1", "D2", "C2"};
    ReadResult r9 = readText(rinex2Header(nine), "types9");
    ASSERT_TRUE(r9.header_ok);
    EXPECT_EQ(r9.header.observation_types, nine);

    std::vector<std::string> many;
    for (const char* t : {"C1", "L1", "D1", "S1", "P1", "C2", "L2", "D2", "S2", "P2", "C5",
                          "L5", "D5", "S5", "C6", "L6", "D6", "S6", "C7", "L7", "D7"}) {
        many.emplace_back(t);
    }
    ReadResult r21 = readText(rinex2Header(many), "types21");
    ASSERT_TRUE(r21.header_ok);
    EXPECT_EQ(r21.header.observation_types, many);
}

// 2b ---------------------------------------------------------------------
namespace {

std::string fourteenSatelliteFile(size_t pad_to, std::optional<double> clock) {
    const std::vector<std::string> types = {"C1", "L1", "P2", "L2"};
    std::vector<std::string> sats;
    for (int i = 1; i <= 14; ++i) {
        char id[8];
        std::snprintf(id, sizeof(id), "G%02d", i);
        sats.emplace_back(id);
    }
    std::string text = rinex2Header(types);
    text += epochRecord(0, 0, sats, pad_to, clock);
    for (int i = 1; i <= 14; ++i) {
        text += obsRows({20000000.0 + i * 1000.0, 100000000.0 + i, 20000100.0 + i, 100000007.0});
    }
    return text;
}

void expectFourteenGpsSatellites(const ReadResult& r) {
    ASSERT_TRUE(r.header_ok);
    ASSERT_EQ(r.epochs.size(), 1U);
    const ObservationData& epoch = r.epochs[0];
    EXPECT_EQ(countSystem(epoch, GNSSSystem::GPS), 14U * 2U);  // L1 + L2 per satellite
    for (int prn = 1; prn <= 14; ++prn) {
        const Observation* o = findObs(epoch, GNSSSystem::GPS, prn, 0);
        ASSERT_NE(o, nullptr) << "G" << prn;
        EXPECT_DOUBLE_EQ(o->pseudorange, 20000000.0 + prn * 1000.0) << "G" << prn;
    }
    // No phantom satellites from misreading the continuation row.
    EXPECT_EQ(findObs(epoch, GNSSSystem::GPS, 0, 0), nullptr);
    EXPECT_EQ(findObs(epoch, GNSSSystem::GPS, 23, 0), nullptr);
}

}  // namespace

TEST(Rinex2ReaderTest, ReadsSatelliteListContinuationWithUnpaddedFirstLine) {
    expectFourteenGpsSatellites(readText(fourteenSatelliteFile(0, std::nullopt), "sat14_plain"));
}

TEST(Rinex2ReaderTest, ReadsSatelliteListContinuationWithPaddedFirstLine) {
    expectFourteenGpsSatellites(readText(fourteenSatelliteFile(80, std::nullopt), "sat14_pad"));
}

TEST(Rinex2ReaderTest, ReadsSatelliteListContinuationWithReceiverClockOffset) {
    expectFourteenGpsSatellites(readText(fourteenSatelliteFile(0, 0.123456789), "sat14_clk"));
}

TEST(Rinex2ReaderTest, ReadsTwentyFiveSatellitesAcrossThreeListRecords) {
    std::vector<std::string> sats;
    for (int i = 1; i <= 25; ++i) {
        char id[8];
        std::snprintf(id, sizeof(id), "G%02d", i);
        sats.emplace_back(id);
    }
    std::string text = rinex2Header({"C1", "L1"});
    text += epochRecord(0, 0, sats, 80);
    for (int i = 1; i <= 25; ++i) text += obsRows({20000000.0 + i, 100000000.0 + i});
    text += epochRecord(30, 0, {"G01"});
    text += obsRows({21000000.0, 110000000.0});

    const ReadResult r = readText(text, "sat25");
    ASSERT_EQ(r.epochs.size(), 2U);
    for (int prn : {1, 12, 13, 24, 25}) {
        const Observation* o = findObs(r.epochs[0], GNSSSystem::GPS, prn, 0);
        ASSERT_NE(o, nullptr) << prn;
        EXPECT_DOUBLE_EQ(o->pseudorange, 20000000.0 + prn);
    }
    EXPECT_NEAR(r.epochs[1].time - r.epochs[0].time, 30.0, 1e-6);
}

// 2c ---------------------------------------------------------------------
TEST(Rinex2ReaderTest, MapsSystemLettersWithoutAliasingToGps) {
    // C1/L1 are also valid band-1 observations for QZSS; SBAS has no mapped
    // signal in this library, so its rows must be consumed and dropped.
    std::string text = rinex2Header({"C1", "L1"});
    text += epochRecord(0, 0, {"G20", "S20", "R05", "E11", "J02", "C06", "X07", " 03", "I01"});
    text += obsRows({20000020.0, 100000020.0});  // G20
    text += obsRows({20000120.0, 100000120.0});  // S20
    text += obsRows({20000005.0, 100000005.0});  // R05
    text += obsRows({20000011.0, 100000011.0});  // E11
    text += obsRows({20000002.0, 100000002.0});  // J02
    text += obsRows({20000006.0, 100000006.0});  // C06
    text += obsRows({20009999.0, 100009999.0});  // X07 (unknown letter)
    text += obsRows({20000003.0, 100000003.0});  // blank = GPS 03
    text += obsRows({20000001.0, 100000001.0});  // I01 (not a RINEX 2.x system)
    text += epochRecord(30, 0, {"G20"});
    text += obsRows({20001020.0, 100001020.0});

    const ReadResult r = readText(text, "sysids");
    ASSERT_EQ(r.epochs.size(), 2U);
    const ObservationData& epoch = r.epochs[0];

    const Observation* g20 = findObs(epoch, GNSSSystem::GPS, 20, 0);
    ASSERT_NE(g20, nullptr);
    EXPECT_DOUBLE_EQ(g20->pseudorange, 20000020.0);  // not overwritten by S20
    EXPECT_EQ(countSystem(epoch, GNSSSystem::GPS), 2U);  // G20 and blank-letter 03
    const Observation* g03 = findObs(epoch, GNSSSystem::GPS, 3, 0);
    ASSERT_NE(g03, nullptr);
    EXPECT_DOUBLE_EQ(g03->pseudorange, 20000003.0);

    // J02 stays QZSS PRN 2 (as the RINEX 3 path stores it), never GPS 2.
    EXPECT_EQ(findObs(epoch, GNSSSystem::GPS, 2, 0), nullptr);
    const Observation* j02 = findObs(epoch, GNSSSystem::QZSS, 2, 0);
    ASSERT_NE(j02, nullptr);
    EXPECT_DOUBLE_EQ(j02->pseudorange, 20000002.0);

    const Observation* r05 = findObs(epoch, GNSSSystem::GLONASS, 5, 0);
    ASSERT_NE(r05, nullptr);
    EXPECT_DOUBLE_EQ(r05->pseudorange, 20000005.0);
    const Observation* e11 = findObs(epoch, GNSSSystem::Galileo, 11, 0);
    ASSERT_NE(e11, nullptr);
    EXPECT_DOUBLE_EQ(e11->pseudorange, 20000011.0);
    EXPECT_EQ(countSystem(epoch, GNSSSystem::BeiDou), 1U);

    // Unknown letters (X, I) never turn into GPS 07 / 01.
    EXPECT_EQ(findObs(epoch, GNSSSystem::GPS, 7, 0), nullptr);
    EXPECT_EQ(findObs(epoch, GNSSSystem::GPS, 1, 0), nullptr);
    // SBAS S20 is neither GPS 20 nor a second GPS observation set.
    EXPECT_EQ(countSystem(epoch, GNSSSystem::SBAS), 0U);

    // Rows were consumed for every listed satellite, so the next epoch is intact.
    const Observation* next = findObs(r.epochs[1], GNSSSystem::GPS, 20, 0);
    ASSERT_NE(next, nullptr);
    EXPECT_DOUBLE_EQ(next->pseudorange, 20001020.0);
}

// 2d ---------------------------------------------------------------------
TEST(Rinex2ReaderTest, SkipsEventFlag3NewSiteRecords) {
    std::string text = rinex2Header({"C1", "L1"});
    text += epochRecord(0, 0, {"G01"});
    text += obsRows({20000001.0, 100000001.0});
    // New site occupation: 2 header-style records follow, no satellites.
    text += epochRecord(15, 3, {}, 0, std::nullopt, 2);
    text += headerLine("NEWSITE", "MARKER NAME");
    text += headerLine("  -3957100.0000  3310100.0000  3737100.0000", "APPROX POSITION XYZ");
    text += epochRecord(30, 0, {"G02"});
    text += obsRows({20000002.0, 100000002.0});

    const ReadResult r = readText(text, "flag3");
    ASSERT_EQ(r.epochs.size(), 2U);
    EXPECT_NE(findObs(r.epochs[0], GNSSSystem::GPS, 1, 0), nullptr);
    EXPECT_EQ(r.epochs[1].observations.empty(), false);
    const Observation* g2 = findObs(r.epochs[1], GNSSSystem::GPS, 2, 0);
    ASSERT_NE(g2, nullptr);
    EXPECT_DOUBLE_EQ(g2->pseudorange, 20000002.0);
    EXPECT_NEAR(r.epochs[1].time - r.epochs[0].time, 30.0, 1e-6);
}

TEST(Rinex2ReaderTest, AppliesObservationTypeChangeFromEventFlag4HeaderRecords) {
    std::string text = rinex2Header({"C1", "L1"});
    text += epochRecord(0, 0, {"G01"});
    text += obsRows({20000001.0, 100000001.0});
    // Header information follows: new type list (L1 first, 3 types), plus a
    // comment record that must simply be skipped.
    text += epochRecord(15, 4, {}, 0, std::nullopt, 2);
    text += headerLine("     3    L1    C1    P2", "# / TYPES OF OBSERV");
    text += headerLine("antenna swapped", "COMMENT");
    text += epochRecord(30, 0, {"G01"});
    text += obsRows({100000002.0, 20000002.0, 20000102.0});

    const ReadResult r = readText(text, "flag4");
    ASSERT_EQ(r.epochs.size(), 2U);
    const Observation* before = findObs(r.epochs[0], GNSSSystem::GPS, 1, 0);
    ASSERT_NE(before, nullptr);
    EXPECT_DOUBLE_EQ(before->pseudorange, 20000001.0);
    const Observation* after = findObs(r.epochs[1], GNSSSystem::GPS, 1, 0);
    ASSERT_NE(after, nullptr);
    EXPECT_DOUBLE_EQ(after->pseudorange, 20000002.0);
    EXPECT_DOUBLE_EQ(after->carrier_phase, 100000002.0);
    const Observation* l2 = findObs(r.epochs[1], GNSSSystem::GPS, 1, 1);
    ASSERT_NE(l2, nullptr);
    EXPECT_DOUBLE_EQ(l2->pseudorange, 20000102.0);
}

TEST(Rinex2ReaderTest, SkipsEventFlag5ExternalEventAndFlag2Records) {
    std::string text = rinex2Header({"C1", "L1"});
    text += epochRecord(0, 0, {"G01"});
    text += obsRows({20000001.0, 100000001.0});
    text += epochRecord(10, 5, {}, 0, std::nullopt, 1);
    text += headerLine("trigger from event input", "COMMENT");
    text += epochRecord(20, 2, {}, 0, std::nullopt, 1);
    text += headerLine("start moving antenna", "COMMENT");
    text += epochRecord(30, 0, {"G01"});
    text += obsRows({20000031.0, 100000031.0});

    const ReadResult r = readText(text, "flag5");
    ASSERT_EQ(r.epochs.size(), 2U);
    EXPECT_NEAR(r.epochs[1].time - r.epochs[0].time, 30.0, 1e-6);
    const Observation* g1 = findObs(r.epochs[1], GNSSSystem::GPS, 1, 0);
    ASSERT_NE(g1, nullptr);
    EXPECT_DOUBLE_EQ(g1->pseudorange, 20000031.0);
}

TEST(Rinex2ReaderTest, ConsumesButDoesNotEmitFlag6CycleSlipRecords) {
    std::string text = rinex2Header({"C1", "L1"});
    text += epochRecord(0, 0, {"G01"});
    text += obsRows({20000001.0, 100000001.0});
    // Flag 6: satellite list + observation-format rows carrying slip info.
    text += epochRecord(15, 6, {"G01", "G02"});
    text += obsRows({std::nullopt, 1.0});
    text += obsRows({std::nullopt, 2.0});
    text += epochRecord(30, 0, {"G01"});
    text += obsRows({20000031.0, 100000031.0});

    const ReadResult r = readText(text, "flag6");
    ASSERT_EQ(r.epochs.size(), 2U);
    EXPECT_NEAR(r.epochs[1].time - r.epochs[0].time, 30.0, 1e-6);
    EXPECT_EQ(findObs(r.epochs[1], GNSSSystem::GPS, 2, 0), nullptr);
}

TEST(Rinex2ReaderTest, ResynchronisesOnTheNextEpochAfterGarbageRows) {
    std::string text = rinex2Header({"C1", "L1"});
    text += epochRecord(0, 0, {"G01"});
    text += obsRows({20000001.0, 100000001.0});
    text += "this line is not an epoch and not an observation row\n";
    text += "\n";
    text += epochRecord(30, 0, {"G01"});
    text += obsRows({20000031.0, 100000031.0});

    const ReadResult r = readText(text, "garbage");
    ASSERT_EQ(r.epochs.size(), 2U);
}

// 2e ---------------------------------------------------------------------
TEST(Rinex2ReaderTest, AcceptsBlankAndRightTrimmedObservationRows) {
    // A satellite with no observations at all (blank row), one with a
    // right-trimmed row, and one with a short row before a complete epoch.
    std::string text = rinex2Header({"C1", "L1", "P2", "L2"});
    text += epochRecord(0, 0, {"G01", "G02", "G03"});
    text += "\n";                                             // G01: fully blank row
    text += obsRows({20000002.0, std::nullopt}, true);        // G02: trimmed after C1
    text += obsRows({20000003.0, 100000003.0, 20000103.0, 100000007.0});
    text += epochRecord(30, 0, {"G01"});
    text += obsRows({20000031.0, 100000031.0, 20000131.0, 100000037.0});

    const ReadResult r = readText(text, "blank");
    ASSERT_TRUE(r.header_ok);
    ASSERT_EQ(r.epochs.size(), 2U);   // reading must not stop at the short rows

    EXPECT_EQ(findObs(r.epochs[0], GNSSSystem::GPS, 1, 0), nullptr);
    const Observation* g2 = findObs(r.epochs[0], GNSSSystem::GPS, 2, 0);
    ASSERT_NE(g2, nullptr);
    EXPECT_DOUBLE_EQ(g2->pseudorange, 20000002.0);
    EXPECT_FALSE(g2->has_carrier_phase);
    const Observation* g3 = findObs(r.epochs[0], GNSSSystem::GPS, 3, 0);
    ASSERT_NE(g3, nullptr);
    EXPECT_DOUBLE_EQ(g3->pseudorange, 20000003.0);
    EXPECT_DOUBLE_EQ(g3->carrier_phase, 100000003.0);

    const Observation* next = findObs(r.epochs[1], GNSSSystem::GPS, 1, 0);
    ASSERT_NE(next, nullptr);
    EXPECT_DOUBLE_EQ(next->pseudorange, 20000031.0);
}

TEST(Rinex2ReaderTest, ReadsCrlfFilesWithTrimmedRows) {
    std::string text = rinex2Header({"C1", "L1"});
    text += epochRecord(0, 0, {"G01"});
    text += obsRows({20000001.0, 100000001.0}, true);
    text += epochRecord(30, 0, {"G01"});
    text += obsRows({20000031.0, std::nullopt}, true);
    std::string crlf;
    for (char c : text) {
        if (c == '\n') crlf += '\r';
        crlf += c;
    }
    const ReadResult r = readText(crlf, "crlf");
    ASSERT_TRUE(r.header_ok);
    ASSERT_EQ(r.epochs.size(), 2U);
    const Observation* o = findObs(r.epochs[1], GNSSSystem::GPS, 1, 0);
    ASSERT_NE(o, nullptr);
    EXPECT_DOUBLE_EQ(o->pseudorange, 20000031.0);
    EXPECT_FALSE(o->has_carrier_phase);
    EXPECT_EQ(o->lli, 0);
}

// 2f ---------------------------------------------------------------------
namespace {

// Types listed in the given order; G01 carries distinct values per type.
double gpsPrimaryRange(const std::vector<std::string>& types,
                       const std::vector<Value>& values,
                       const std::string& name,
                       GNSSSystem system = GNSSSystem::GPS,
                       int band_rank = 0) {
    std::string text = rinex2Header(types);
    const char* id = system == GNSSSystem::GLONASS ? "R01" : "G01";
    text += epochRecord(0, 0, {id});
    text += obsRows(values);
    const ReadResult r = readText(text, name);
    EXPECT_EQ(r.epochs.size(), 1U);
    if (r.epochs.size() != 1U) return -1.0;
    const Observation* o = findObs(r.epochs[0], system, 1, band_rank);
    EXPECT_NE(o, nullptr);
    return o ? o->pseudorange : -1.0;
}

}  // namespace

TEST(Rinex2ReaderTest, PrefersC1OverP1RegardlessOfHeaderOrder) {
    const double c1 = 20000001.0;
    const double p1 = 20000002.0;
    EXPECT_DOUBLE_EQ(gpsPrimaryRange({"C1", "P1", "L1"}, {c1, p1, 1e8}, "c1p1"), c1);
    EXPECT_DOUBLE_EQ(gpsPrimaryRange({"P1", "C1", "L1"}, {p1, c1, 1e8}, "p1c1"), c1);
    // Fallback: P1 is used when C1 is absent for the epoch ...
    EXPECT_DOUBLE_EQ(gpsPrimaryRange({"C1", "P1", "L1"}, {std::nullopt, p1, 1e8}, "p1only"), p1);
    // ... and when C1 is not in the header at all.
    EXPECT_DOUBLE_EQ(gpsPrimaryRange({"P1", "L1"}, {p1, 1e8}, "p1nohdr"), p1);
}

TEST(Rinex2ReaderTest, PrefersP2OverC2ForGpsAndC2OverP2ForGlonass) {
    const double c2 = 20000011.0;
    const double p2 = 20000012.0;
    // Secondary band: observation rank 1 for the satellite.
    EXPECT_DOUBLE_EQ(gpsPrimaryRange({"C1", "L1", "C2", "P2", "L2"},
                                     {20000001.0, 1e8, c2, p2, 1e8}, "gps_c2p2",
                                     GNSSSystem::GPS, 1),
                     p2);
    EXPECT_DOUBLE_EQ(gpsPrimaryRange({"C1", "L1", "P2", "C2", "L2"},
                                     {20000001.0, 1e8, p2, c2, 1e8}, "gps_p2c2",
                                     GNSSSystem::GPS, 1),
                     p2);
    EXPECT_DOUBLE_EQ(gpsPrimaryRange({"C1", "L1", "C2", "P2", "L2"},
                                     {20000001.0, 1e8, c2, std::nullopt, 1e8}, "gps_c2only",
                                     GNSSSystem::GPS, 1),
                     c2);
    EXPECT_DOUBLE_EQ(gpsPrimaryRange({"C1", "L1", "C2", "P2", "L2"},
                                     {20000001.0, 1e8, c2, p2, 1e8}, "glo_c2p2",
                                     GNSSSystem::GLONASS, 1),
                     c2);
    EXPECT_DOUBLE_EQ(gpsPrimaryRange({"C1", "L1", "P2", "C2", "L2"},
                                     {20000001.0, 1e8, p2, c2, 1e8}, "glo_p2c2",
                                     GNSSSystem::GLONASS, 1),
                     c2);
}
