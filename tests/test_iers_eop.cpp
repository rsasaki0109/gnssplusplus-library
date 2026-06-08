#include "libgnss++/iers/eop_table.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>

namespace {

using libgnss::iers::EopRecord;
using libgnss::iers::EopTable;

std::string writeTempFile(const std::string& contents) {
    char path_template[] = "/tmp/libgnss_eop_XXXXXX";
    int fd = mkstemp(path_template);
    if (fd < 0) {
        ADD_FAILURE() << "mkstemp failed";
        return {};
    }
    std::string path(path_template);
    close(fd);
    std::ofstream out(path);
    out << contents;
    return path;
}

}  // namespace

TEST(IersEopTable, ConstructFromRecordsAndExactLookup) {
    std::vector<EopRecord> recs;
    recs.push_back(EopRecord{
        /*mjd_utc=*/60800.0,
        /*xp_arcsec=*/0.10,
        /*yp_arcsec=*/0.30,
        /*ut1_minus_utc_seconds=*/-0.20,
        /*lod_seconds=*/0.0,
        /*dx_arcsec=*/0.0,
        /*dy_arcsec=*/0.0});
    recs.push_back(EopRecord{60801.0, 0.12, 0.31, -0.19, 0.0, 0.0, 0.0});
    EopTable table(std::move(recs));

    EXPECT_EQ(table.size(), 2u);
    EXPECT_DOUBLE_EQ(table.firstMjd(), 60800.0);
    EXPECT_DOUBLE_EQ(table.lastMjd(), 60801.0);

    auto eop = table.interpolateAt(60800.0);
    EXPECT_DOUBLE_EQ(eop.xp_arcsec, 0.10);
    EXPECT_DOUBLE_EQ(eop.yp_arcsec, 0.30);
    EXPECT_DOUBLE_EQ(eop.ut1_minus_utc_seconds, -0.20);
}

TEST(IersEopTable, LinearInterpolationAtMidpoint) {
    std::vector<EopRecord> recs = {
        EopRecord{60800.0, 0.10, 0.30, -0.20, 0.0, 0.0, 0.0},
        EopRecord{60801.0, 0.12, 0.32, -0.18, 0.0, 0.0, 0.0},
    };
    EopTable table(std::move(recs));

    auto eop = table.interpolateAt(60800.5);
    EXPECT_NEAR(eop.xp_arcsec, 0.11, 1e-12);
    EXPECT_NEAR(eop.yp_arcsec, 0.31, 1e-12);
    EXPECT_NEAR(eop.ut1_minus_utc_seconds, -0.19, 1e-12);
}

TEST(IersEopTable, OutOfRangeThrows) {
    std::vector<EopRecord> recs = {
        EopRecord{60800.0, 0.10, 0.30, -0.20, 0.0, 0.0, 0.0},
        EopRecord{60801.0, 0.12, 0.32, -0.18, 0.0, 0.0, 0.0},
    };
    EopTable table(std::move(recs));
    EXPECT_THROW(table.interpolateAt(60799.5), std::out_of_range);
    EXPECT_THROW(table.interpolateAt(60802.0), std::out_of_range);
}

TEST(IersEopTable, LeapSecondSnapForUt1Minus) {
    // Pre-leap dut1 = +0.4 s, post-leap dut1 = -0.6 s (1-s step).
    // xp/yp are continuous and should still interpolate.
    std::vector<EopRecord> recs = {
        EopRecord{60800.0, 0.10, 0.30, +0.4, 0.0, 0.0, 0.0},
        EopRecord{60801.0, 0.20, 0.40, -0.6, 0.0, 0.0, 0.0},
    };
    EopTable table(std::move(recs));

    // Slightly before midpoint → snap to lower (pre-leap).
    auto a = table.interpolateAt(60800.49);
    EXPECT_DOUBLE_EQ(a.ut1_minus_utc_seconds, +0.4);
    EXPECT_NEAR(a.xp_arcsec, 0.10 + 0.49 * 0.10, 1e-12);

    // Past midpoint → snap to upper (post-leap).
    auto b = table.interpolateAt(60800.6);
    EXPECT_DOUBLE_EQ(b.ut1_minus_utc_seconds, -0.6);
    EXPECT_NEAR(b.xp_arcsec, 0.10 + 0.60 * 0.10, 1e-12);
}

TEST(IersEopTable, ParseC04FixedWidthFixture) {
    // Real IERS 20 C04 lines (header comments + two daily samples).
    const std::string fixture =
        "# EARTH ORIENTATION PARAMETER (EOP) PRODUCT CENTER ...\n"
        "# EOP (IERS) 20 C04 TIME SERIES  consistent with ITRF 2020\n"
        "# YR  MM  DD  HH       MJD        x(\")        y(\")  UT1-UTC(s) "
        "      dX(\")       dY(\")  xrt(\"/day)  yrt(\"/day)      LOD(s)\n"
        "1962   1   1   0  37665.00   -0.012700    0.213000   0.0326338"
        "    0.000000    0.000000    0.000000    0.000000   0.0017230"
        "    0.030000    0.030000   0.0020000    0.004774    0.002000\n"
        "1962   1   2   0  37666.00   -0.015900    0.214100   0.0320547"
        "    0.000000    0.000000    0.000000    0.000000   0.0016690"
        "    0.030000    0.030000   0.0020000    0.004774    0.002000\n";
    const std::string path = writeTempFile(fixture);
    ASSERT_FALSE(path.empty());

    auto table = EopTable::fromC04File(path);
    std::remove(path.c_str());

    ASSERT_EQ(table.size(), 2u);
    EXPECT_DOUBLE_EQ(table.firstMjd(), 37665.0);
    EXPECT_DOUBLE_EQ(table.lastMjd(), 37666.0);

    auto eop = table.interpolateAt(37665.0);
    EXPECT_NEAR(eop.xp_arcsec, -0.012700, 1e-9);
    EXPECT_NEAR(eop.yp_arcsec, 0.213000, 1e-9);
    EXPECT_NEAR(eop.ut1_minus_utc_seconds, 0.0326338, 1e-9);
}

TEST(IersEopTable, FromC04FileMissingFileThrows) {
    EXPECT_THROW(
        EopTable::fromC04File("/this/path/does/not/exist/eopc04.txt"),
        std::runtime_error);
}

// --- Bulletin A finals2000A.daily parser ----------------------------

TEST(IersEopTable, ParseBulletinAFixedWidthFixture) {
    // Two real `finals2000A.daily` rows (USNO format, 187 chars each;
    // the trailing portion past the UT1-UTC error is truncated for
    // brevity — the parser only consumes the first ~78 cols).
    const std::string fixture =
        "26 2 8 61079.00 I  0.093941 0.000018  0.369169 0.000019  I 0.0671873 0.0000113\n"
        "26 4 8 61138.00 I  0.137078 0.000086  0.413609 0.000080  I 0.0477569 0.0000158\n";
    const std::string path = writeTempFile(fixture);
    ASSERT_FALSE(path.empty());

    auto table = EopTable::fromBulletinAFile(path);
    std::remove(path.c_str());

    ASSERT_EQ(table.size(), 2u);
    EXPECT_DOUBLE_EQ(table.firstMjd(), 61079.0);
    EXPECT_DOUBLE_EQ(table.lastMjd(), 61138.0);

    auto eop_first = table.interpolateAt(61079.0);
    EXPECT_NEAR(eop_first.xp_arcsec, 0.093941, 1e-9);
    EXPECT_NEAR(eop_first.yp_arcsec, 0.369169, 1e-9);
    EXPECT_NEAR(eop_first.ut1_minus_utc_seconds, 0.0671873, 1e-9);

    auto eop_last = table.interpolateAt(61138.0);
    EXPECT_NEAR(eop_last.xp_arcsec, 0.137078, 1e-9);
    EXPECT_NEAR(eop_last.yp_arcsec, 0.413609, 1e-9);
    EXPECT_NEAR(eop_last.ut1_minus_utc_seconds, 0.0477569, 1e-9);
}

TEST(IersEopTable, BulletinAKeepsObservedAndPredictedRows) {
    // An `I`-flagged observed row immediately followed by a
    // `P`-flagged predicted row. Both should make it into the table.
    const std::string fixture =
        "26 4 8 61138.00 I  0.137078 0.000086  0.413609 0.000080  I 0.0477569 0.0000158\n"
        "26 4 9 61139.00 P  0.138900 0.000100  0.413700 0.000100  P 0.0476800 0.0000200\n";
    const std::string path = writeTempFile(fixture);
    ASSERT_FALSE(path.empty());

    auto table = EopTable::fromBulletinAFile(path);
    std::remove(path.c_str());

    ASSERT_EQ(table.size(), 2u);
    auto eop_pred = table.interpolateAt(61139.0);
    EXPECT_NEAR(eop_pred.xp_arcsec, 0.138900, 1e-9);
}

TEST(IersEopTable, BulletinASkipsBlankFlagRows) {
    // Some Bulletin A trailing lines have a blank flag (no value
    // available yet). They must not be silently inserted as zeros.
    const std::string fixture =
        "26 4 8 61138.00 I  0.137078 0.000086  0.413609 0.000080  I 0.0477569 0.0000158\n"
        "26 9 1 61285.00                                                                \n"
        "26 9 2 61286.00 P  0.211000 0.008000  0.350000 0.011000  P 0.0700000 0.0090000\n";
    const std::string path = writeTempFile(fixture);
    ASSERT_FALSE(path.empty());

    auto table = EopTable::fromBulletinAFile(path);
    std::remove(path.c_str());
    EXPECT_EQ(table.size(), 2u);  // blank-flag row dropped
}

TEST(IersEopTable, FromFileAutoDetectsBulletinA) {
    const std::string fixture =
        "26 4 8 61138.00 I  0.137078 0.000086  0.413609 0.000080  I 0.0477569 0.0000158\n"
        "26 4 9 61139.00 P  0.138900 0.000100  0.413700 0.000100  P 0.0476800 0.0000200\n";
    const std::string path = writeTempFile(fixture);
    ASSERT_FALSE(path.empty());

    auto table = EopTable::fromFile(path);
    std::remove(path.c_str());
    ASSERT_EQ(table.size(), 2u);
    EXPECT_DOUBLE_EQ(table.firstMjd(), 61138.0);
}

TEST(IersEopTable, FromFileAutoDetectsC04) {
    const std::string fixture =
        "# EOP (IERS) 20 C04 TIME SERIES\n"
        "1962   1   1   0  37665.00   -0.012700    0.213000   0.0326338"
        "    0.000000    0.000000    0.000000    0.000000   0.0017230"
        "    0.030000    0.030000   0.0020000    0.004774    0.002000\n";
    const std::string path = writeTempFile(fixture);
    ASSERT_FALSE(path.empty());

    auto table = EopTable::fromFile(path);
    std::remove(path.c_str());
    ASSERT_EQ(table.size(), 1u);
    EXPECT_DOUBLE_EQ(table.firstMjd(), 37665.0);
}

TEST(IersEopTable, FromBulletinAFileMissingFileThrows) {
    EXPECT_THROW(
        EopTable::fromBulletinAFile("/nonexistent/finals2000A.daily"),
        std::runtime_error);
}
