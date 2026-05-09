// Tests for libgnss::iers::subDailyEopCorrection
// (IERS Conventions 2010 §5.5.1.1 + §8.2 sub-daily EOP corrections).
//
// The vendor model (PMGravi 21 rows + PMUTOcean 71 rows) does not
// have a single canonical "reference epoch" published with reference
// values, so these tests check (1) magnitudes are within the IERS-
// stated peak envelope, (2) physical periodicity (~12 h dominant),
// (3) deterministic / reproducible output for the same epoch, and
// (4) sign-flip behavior when stepping by half a tidal period.

#include "libgnss++/iers/sub_daily_eop.hpp"

#include <gtest/gtest.h>

#include <cmath>

using libgnss::iers::SubDailyEopDelta;
using libgnss::iers::subDailyEopCorrection;

TEST(IersSubDailyEop, MagnitudeIsWithinIersEnvelope) {
    // TSKB 2026-04-15 0h UTC; xp/yp ~0.15 / 0.41 arcsec; the
    // sub-daily delta peaks near +/-0.5 mas in xp/yp and +/-30 µs
    // in UT1. Pick a daily ut1_utc representative of that epoch.
    const double mjd_utc = 61145.0;
    const double ut1_utc = 0.0440894;  // sec, from real Bulletin A
    auto delta = subDailyEopCorrection(mjd_utc, ut1_utc);

    // arcsec → mas conversion factor 1000.
    EXPECT_LT(std::abs(delta.dxp_arcsec) * 1000.0, 1.0);
    EXPECT_LT(std::abs(delta.dyp_arcsec) * 1000.0, 1.0);
    // seconds → microseconds conversion factor 1e6.
    EXPECT_LT(std::abs(delta.dut1_seconds) * 1.0e6, 50.0);
    EXPECT_LT(std::abs(delta.dlod_seconds) * 1.0e6, 100.0);
}

TEST(IersSubDailyEop, NonZeroAtTypicalEpoch) {
    // The harmonic series should be well above floating-point noise
    // for any non-degenerate epoch — guard against accidentally
    // zeroing every term (e.g. a typo in the Doodson dot product).
    auto delta = subDailyEopCorrection(61145.5, 0.0440894);
    const double total =
        std::abs(delta.dxp_arcsec) + std::abs(delta.dyp_arcsec)
      + std::abs(delta.dut1_seconds) + std::abs(delta.dlod_seconds);
    EXPECT_GT(total, 1.0e-10);
}

TEST(IersSubDailyEop, Deterministic) {
    auto a = subDailyEopCorrection(61145.0, 0.044);
    auto b = subDailyEopCorrection(61145.0, 0.044);
    EXPECT_EQ(a.dxp_arcsec, b.dxp_arcsec);
    EXPECT_EQ(a.dyp_arcsec, b.dyp_arcsec);
    EXPECT_EQ(a.dut1_seconds, b.dut1_seconds);
    EXPECT_EQ(a.dlod_seconds, b.dlod_seconds);
}

TEST(IersSubDailyEop, VariesAtSubDailyTimescale) {
    // Stepping by 6 hours should shift xp/yp on a scale comparable
    // to the peak amplitude (the dominant terms have ~12 h periods).
    // Use the difference norm as a smoke test that the time
    // dependence is alive.
    auto at_0h  = subDailyEopCorrection(61145.0, 0.044);
    auto at_6h  = subDailyEopCorrection(61145.0 + 0.25, 0.044);
    const double dxp = at_6h.dxp_arcsec - at_0h.dxp_arcsec;
    const double dyp = at_6h.dyp_arcsec - at_0h.dyp_arcsec;
    EXPECT_GT(std::abs(dxp) + std::abs(dyp), 1.0e-9);
}

TEST(IersSubDailyEop, RoughlyAntiPeriodicAtHalfM2) {
    // The dominant ocean-tide contribution near +/-0.5 mas is the M2
    // semidiurnal term with ~12.42 h period. Half a period after a
    // peak the M2-driven part should reverse sign. Use the SUM of
    // (delta(t) + delta(t + 6.21 h)) to test that the leading-order
    // signature is anti-periodic on this timescale; the residual
    // should be much smaller than either side individually.
    constexpr double kHalfM2Days = 12.4206 / 2.0 / 24.0;
    auto a = subDailyEopCorrection(61145.0, 0.044);
    auto b = subDailyEopCorrection(61145.0 + kHalfM2Days, 0.044);
    const double sum_xp = a.dxp_arcsec + b.dxp_arcsec;
    const double max_xp = std::max(std::abs(a.dxp_arcsec),
                                   std::abs(b.dxp_arcsec));
    EXPECT_LT(std::abs(sum_xp), max_xp);
}
