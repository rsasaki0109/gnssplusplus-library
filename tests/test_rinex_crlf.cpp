// RINEX 3.x line-ending and LLI/SSI robustness regression tests.
//
//  * CRLF files must decode exactly like LF files: header, observation and
//    navigation rows all lose their trailing '\r'.  Previously the '\r' of a
//    right-trimmed observation row landed in an LLI column and was decoded as
//    ('\r' - '0') = 221, setting the cycle-slip bit on every epoch.
//  * LLI is only defined for '0'..'7' and SSI for '1'..'9'; any other
//    character is ignored.
#include <gtest/gtest.h>

#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/io/rinex.hpp>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace libgnss;

namespace {

std::string hdr(const std::string& content, const std::string& label) {
    std::string line = content;
    if (line.size() < 60) line.append(60 - line.size(), ' ');
    return line + label + "\n";
}

std::string toCrlf(const std::string& text) {
    std::string out;
    for (char c : text) {
        if (c == '\n') out += '\r';
        out += c;
    }
    return out;
}

std::string rstripLines(const std::string& text) {
    std::string out;
    size_t pos = 0;
    while (pos < text.size()) {
        size_t end = text.find('\n', pos);
        if (end == std::string::npos) end = text.size();
        std::string line = text.substr(pos, end - pos);
        const size_t last = line.find_last_not_of(' ');
        line = (last == std::string::npos) ? std::string() : line.substr(0, last + 1);
        out += line + "\n";
        pos = end + 1;
    }
    return out;
}

std::string obsHeader() {
    std::string out;
    out += hdr("     3.04           OBSERVATION DATA    M", "RINEX VERSION / TYPE");
    out += hdr("unit test", "PGM / RUN BY / DATE");
    out += hdr("TEST", "MARKER NAME");
    out += hdr("  -3957000.0000  3310000.0000  3737000.0000", "APPROX POSITION XYZ");
    out += hdr("G    4 C1C L1C C2W L2W", "SYS / # / OBS TYPES");
    out += hdr("  2024     1     1     0     0    0.0000000     GPS", "TIME OF FIRST OBS");
    out += hdr("", "END OF HEADER");
    return out;
}

// One observation field: F14.3 + LLI + SSI.
std::string field(double value, char lli = ' ', char ssi = ' ') {
    char buf[24];
    std::snprintf(buf, sizeof(buf), "%14.3f", value);
    return std::string(buf) + lli + ssi;
}

std::string epochHeader(int sec, int nsat) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "> 2024 01 01 00 00%11.7f  0%3d\n",
                  static_cast<double>(sec), nsat);
    return buf;
}

std::filesystem::path writeTemp(const std::string& text, const std::string& name) {
    const auto path = std::filesystem::temp_directory_path() / ("libgnss_crlf_" + name);
    std::ofstream file(path, std::ios::binary);
    file << text;
    return path;
}

std::vector<ObservationData> readObs(const std::string& text, const std::string& name) {
    const auto path = writeTemp(text, name + ".obs");
    std::vector<ObservationData> epochs;
    io::RINEXReader reader;
    reader.setPreserveAdditionalFrequencyBands(true);
    io::RINEXReader::RINEXHeader header;
    if (reader.open(path.string()) && reader.readHeader(header)) {
        ObservationData epoch;
        while (reader.readObservationEpoch(epoch)) epochs.push_back(epoch);
    }
    std::error_code ec;
    std::filesystem::remove(path, ec);
    return epochs;
}

void expectSameObservations(const std::vector<ObservationData>& a,
                            const std::vector<ObservationData>& b) {
    ASSERT_EQ(a.size(), b.size());
    for (size_t e = 0; e < a.size(); ++e) {
        EXPECT_EQ(a[e].time.week, b[e].time.week);
        EXPECT_DOUBLE_EQ(a[e].time.tow, b[e].time.tow);
        ASSERT_EQ(a[e].observations.size(), b[e].observations.size());
        for (size_t i = 0; i < a[e].observations.size(); ++i) {
            const auto& x = a[e].observations[i];
            const auto& y = b[e].observations[i];
            EXPECT_EQ(x.satellite, y.satellite);
            EXPECT_EQ(x.signal, y.signal);
            EXPECT_EQ(x.has_pseudorange, y.has_pseudorange);
            EXPECT_DOUBLE_EQ(x.pseudorange, y.pseudorange);
            EXPECT_EQ(x.has_carrier_phase, y.has_carrier_phase);
            EXPECT_DOUBLE_EQ(x.carrier_phase, y.carrier_phase);
            EXPECT_EQ(x.lli, y.lli);
            EXPECT_EQ(x.loss_of_lock, y.loss_of_lock);
            EXPECT_EQ(x.signal_strength, y.signal_strength);
            EXPECT_DOUBLE_EQ(x.snr, y.snr);
        }
    }
}

std::string rinex3ObsBody(char lli, char ssi) {
    std::string text = obsHeader();
    for (int sec = 0; sec < 3; ++sec) {
        text += epochHeader(sec * 30, 2);
        text += "G05" + field(20000000.0 + sec) + field(100000000.0 + sec, lli, ssi) +
                field(20000003.0 + sec) + field(100000003.0 + sec, lli, ssi) + "\n";
        text += "G07" + field(21000000.0 + sec) + field(110000000.0 + sec, lli, ssi) +
                field(21000003.0 + sec) + field(110000003.0 + sec, lli, ssi) + "\n";
    }
    return text;
}

}  // namespace

TEST(RinexCrlfTest, Rinex3CrlfTrimmedObservationRowsHaveNoSpuriousLli) {
    const std::string lf = rstripLines(rinex3ObsBody(' ', ' '));
    const auto lf_epochs = readObs(lf, "r3_lf_trim");
    const auto crlf_epochs = readObs(toCrlf(lf), "r3_crlf_trim");
    ASSERT_EQ(lf_epochs.size(), 3U);
    ASSERT_EQ(crlf_epochs.size(), 3U);
    for (const auto& epoch : crlf_epochs) {
        ASSERT_FALSE(epoch.observations.empty());
        for (const auto& obs : epoch.observations) {
            EXPECT_EQ(obs.lli, 0);
            EXPECT_FALSE(obs.loss_of_lock);
        }
    }
    expectSameObservations(lf_epochs, crlf_epochs);

    // Padded CRLF rows (the '\r' follows the blank LLI/SSI of the last field).
    const std::string padded = rinex3ObsBody(' ', ' ');
    expectSameObservations(readObs(padded, "r3_lf_pad"),
                           readObs(toCrlf(padded), "r3_crlf_pad"));
}

TEST(RinexCrlfTest, Rinex3CrlfPreservesRealLliAndSsi) {
    // Trimmed rows whose last field carries real flags must keep them.
    const std::string lf = rstripLines(rinex3ObsBody('1', '7'));
    const auto lf_epochs = readObs(lf, "r3_flags_lf");
    const auto crlf_epochs = readObs(toCrlf(lf), "r3_flags_crlf");
    ASSERT_EQ(crlf_epochs.size(), 3U);
    expectSameObservations(lf_epochs, crlf_epochs);
    bool saw_lli = false;
    for (const auto& obs : crlf_epochs[0].observations) {
        if (obs.has_carrier_phase) {
            EXPECT_EQ(obs.lli, 1);
            EXPECT_TRUE(obs.loss_of_lock);
            EXPECT_EQ(obs.signal_strength, 7);
            saw_lli = true;
        }
    }
    EXPECT_TRUE(saw_lli);
}

TEST(RinexCrlfTest, Rinex3IgnoresInvalidLliAndSsiCharacters) {
    const auto blank = readObs(rinex3ObsBody(' ', ' '), "inv_blank");
    ASSERT_EQ(blank.size(), 3U);
    // LLI: letters, '8', '9', control characters; SSI: '0' and letters.
    const std::vector<std::pair<char, char>> bad = {
        {'x', ' '}, {'8', ' '}, {'9', ' '}, {'\r', ' '}, {'-', ' '}, {' ', '0'}, {' ', 'x'},
        {'x', 'x'}};
    for (const auto& pair : bad) {
        const auto epochs = readObs(rinex3ObsBody(pair.first, pair.second),
                                    "inv_" + std::to_string(static_cast<int>(pair.first)) + "_" +
                                        std::to_string(static_cast<int>(pair.second)));
        expectSameObservations(blank, epochs);
        for (const auto& epoch : epochs) {
            for (const auto& obs : epoch.observations) EXPECT_EQ(obs.lli, 0);
        }
    }

    // Valid boundaries are still honoured.
    const auto lli7 = readObs(rinex3ObsBody('7', '9'), "inv_valid");
    ASSERT_EQ(lli7.size(), 3U);
    for (const auto& obs : lli7[0].observations) {
        if (obs.has_carrier_phase) {
            EXPECT_EQ(obs.lli, 7);
            EXPECT_EQ(obs.signal_strength, 9);
        }
    }
}

TEST(RinexCrlfTest, Rinex2IgnoresInvalidLliAndSsiCharacters) {
    auto build = [](char lli, char ssi) {
        std::string text;
        text += hdr("     2.11           OBSERVATION DATA    G (GPS)", "RINEX VERSION / TYPE");
        text += hdr("unit test", "PGM / RUN BY / DATE");
        text += hdr("TEST", "MARKER NAME");
        text += hdr("  -3957000.0000  3310000.0000  3737000.0000", "APPROX POSITION XYZ");
        text += hdr("     2    C1    L1", "# / TYPES OF OBSERV");
        text += hdr("", "END OF HEADER");
        text += " 24  1  1  0  0  0.0000000  0  1G05\n";
        text += field(20000000.0) + field(100000000.0, lli, ssi) + "\n";
        return text;
    };
    const auto blank = readObs(build(' ', ' '), "r2_blank");
    ASSERT_EQ(blank.size(), 1U);
    for (const auto& pair : std::vector<std::pair<char, char>>{{'x', 'x'}, {'8', '0'}, {'9', ' '}}) {
        const auto epochs = readObs(build(pair.first, pair.second), "r2_bad");
        expectSameObservations(blank, epochs);
    }
    const auto good = readObs(build('3', '4'), "r2_good");
    ASSERT_EQ(good.size(), 1U);
    ASSERT_FALSE(good[0].observations.empty());
    EXPECT_EQ(good[0].observations[0].lli, 3);
    EXPECT_EQ(good[0].observations[0].signal_strength, 4);
}

namespace {

std::string dfield(double v) {
    char buf[40];
    std::snprintf(buf, sizeof(buf), "%19.12E", v);
    std::string s = buf;
    const size_t e = s.find('E');
    if (e != std::string::npos) s[e] = 'D';
    return s;
}

std::string navRecord(const std::string& sat, int hour, double toe) {
    char head[64];
    std::snprintf(head, sizeof(head), "%3s %04d %02d %02d %02d %02d %02d", sat.c_str(), 2024, 1, 1,
                  hour, 0, 0);
    std::string out = std::string(head) + dfield(1e-4) + dfield(1e-12) + dfield(0.0) + "\n";
    const std::vector<std::vector<double>> rows = {
        {10, 10, 1e-9, 0.5},       {1e-6, 0.001, 1e-6, 5153.7}, {toe, 0, 0, 0},
        {0.9, 0, 0, 0},            {0, 1, 2295, 0},             {2.0, 0, -5.6e-9, 10},
        {toe - 200.0, 4.0, 0, 0}};
    for (const auto& row : rows) {
        out += "    ";
        for (double v : row) out += dfield(v);
        out += "\n";
    }
    return out;
}

NavigationData readNav(const std::string& text, const std::string& name, bool* ok) {
    const auto path = writeTemp(text, name + ".nav");
    NavigationData nav;
    io::RINEXReader reader;
    *ok = reader.open(path.string()) && reader.readNavigationData(nav);
    std::error_code ec;
    std::filesystem::remove(path, ec);
    return nav;
}

}  // namespace

TEST(RinexCrlfTest, CrlfNavigationFileParsesLikeLf) {
    std::string lf;
    lf += hdr("     3.04           N: GNSS NAV DATA    M: MIXED", "RINEX VERSION / TYPE");
    lf += hdr("unit test", "PGM / RUN BY / DATE");
    lf += hdr("", "END OF HEADER");
    lf += navRecord("G05", 2, 7200.0);
    lf += navRecord("G07", 2, 7200.0);
    lf += navRecord("G05", 4, 14400.0);

    bool ok_lf = false;
    bool ok_crlf = false;
    const NavigationData a = readNav(lf, "lf", &ok_lf);
    const NavigationData b = readNav(toCrlf(lf), "crlf", &ok_crlf);
    ASSERT_TRUE(ok_lf);
    ASSERT_TRUE(ok_crlf);

    ASSERT_EQ(a.ephemeris_data.size(), 2U);
    ASSERT_EQ(a.ephemeris_data.size(), b.ephemeris_data.size());
    for (const auto& [sat, list_a] : a.ephemeris_data) {
        const auto it = b.ephemeris_data.find(sat);
        ASSERT_NE(it, b.ephemeris_data.end());
        const auto& list_b = it->second;
        ASSERT_EQ(list_a.size(), list_b.size());
        for (size_t i = 0; i < list_a.size(); ++i) {
            EXPECT_EQ(list_a[i].week, list_b[i].week);
            EXPECT_DOUBLE_EQ(list_a[i].toe.tow, list_b[i].toe.tow);
            EXPECT_DOUBLE_EQ(list_a[i].toc.tow, list_b[i].toc.tow);
            EXPECT_DOUBLE_EQ(list_a[i].sqrt_a, list_b[i].sqrt_a);
            EXPECT_DOUBLE_EQ(list_a[i].af0, list_b[i].af0);
            EXPECT_DOUBLE_EQ(list_a[i].tgd, list_b[i].tgd);
            EXPECT_EQ(list_a[i].health, list_b[i].health);
        }
    }
}
