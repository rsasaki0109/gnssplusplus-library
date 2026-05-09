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
