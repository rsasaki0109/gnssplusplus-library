// Tests for libgnss::iers::solidEarthTideDisplacement
// (include/libgnss++/iers/tides.hpp).
//
// The vendor-level smoke test (tests/test_ginan_iers2010_vendor.cpp)
// already validates the underlying IERS Conventions 2010 reference
// case at the iers2010::dehanttideinel_impl boundary. Here we
// re-verify the SAME reference case through the libgnss wrapper to
// catch any regression in the wrapper glue (time-scale conversions,
// vector marshaling).

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "libgnss++/iers/tides.hpp"

using libgnss::iers::solidEarthTideDisplacement;

TEST(IersTides, SolidEarthTideMatchesIersReference) {
    // Canonical IERS Conventions 2010 reference case (DEHANTTIDEINEL):
    //   Epoch 2009-04-13 0h UTC == MJD UTC 54934.0 exactly.
    //   Expected DXYZ = (0.07700420357108126,
    //                    0.06304056321824968,
    //                    0.05516568152597247) m.
    const double mjd_utc = 54934.0;

    const Eigen::Vector3d xsta(4075578.385, 931852.890, 4801570.154);
    const Eigen::Vector3d xsun(137859926952.015,
                                54228127881.4350,
                                23509422341.6960);
    const Eigen::Vector3d xmon(-179996231.920342,
                               -312468450.131567,
                               -169288918.592160);

    const Eigen::Vector3d dxyz =
        solidEarthTideDisplacement(mjd_utc, xsta, xsun, xmon);

    const Eigen::Vector3d expected(0.07700420357108126,
                                   0.06304056321824968,
                                   0.05516568152597247);

    // 1 µm tolerance — same as the vendor smoke test. The wrapper
    // adds only deterministic time-scale conversion glue.
    EXPECT_NEAR(dxyz.x(), expected.x(), 1e-6);
    EXPECT_NEAR(dxyz.y(), expected.y(), 1e-6);
    EXPECT_NEAR(dxyz.z(), expected.z(), 1e-6);
}

TEST(IersTides, SolidEarthTideAmplitudeIsBounded) {
    // Sanity check: the tidal displacement at any single epoch should
    // fall well under 50 cm in any component (peak diurnal amplitude
    // is ~30 cm radial). This guards against any catastrophic glue
    // failure (wrong unit, swapped components, etc.) that does not
    // hit the specific reference case.
    const Eigen::Vector3d xsta(4075578.385, 931852.890, 4801570.154);
    const Eigen::Vector3d xsun(137859926952.015,
                                54228127881.4350,
                                23509422341.6960);
    const Eigen::Vector3d xmon(-179996231.920342,
                               -312468450.131567,
                               -169288918.592160);

    const Eigen::Vector3d dxyz =
        solidEarthTideDisplacement(54934.0, xsta, xsun, xmon);

    EXPECT_LT(dxyz.cwiseAbs().maxCoeff(), 0.5);
}

// --- Pole tide (IERS Conventions 2010 §7.1.4) -----------------------

using libgnss::iers::poleTideDisplacement;
using libgnss::iers::EarthOrientationParams;

namespace {

// TSKB approx (Tsukuba, Japan): 36.106 N, 140.087 E, h ≈ 67 m → ECEF
// roughly (-3957209, 3310326, 3737690) m. Used as a representative
// mid-latitude site for amplitude/sanity tests.
const Eigen::Vector3d kTskbEcef(-3957209.0, 3310326.0, 3737690.0);

}  // namespace

TEST(IersPoleTide, ZeroAtMeanPole) {
    // 2026-04-15 UTC ≈ MJD 61145; mean pole at decimal year 2026.29
    // is x_mean = 55.0 + 1.677*26.29 = 99.09 mas, y_mean = 320.5 +
    // 3.460*26.29 = 411.46 mas. Setting xp/yp to those values exactly
    // makes m1 = m2 = 0, so the displacement must be exactly zero
    // at every station regardless of latitude/longitude.
    const double mjd_utc = 61145.0;
    const double dt = 2000.0 + (mjd_utc - 51544.5) / 365.25 - 2000.0;
    const double xp_mean_arcsec = (55.0 + 1.677 * dt) / 1000.0;
    const double yp_mean_arcsec = (320.5 + 3.460 * dt) / 1000.0;

    EarthOrientationParams eop{};
    eop.xp_arcsec = xp_mean_arcsec;
    eop.yp_arcsec = yp_mean_arcsec;
    eop.ut1_minus_utc_seconds = 0.0;

    const Eigen::Vector3d disp = poleTideDisplacement(mjd_utc, kTskbEcef, eop);
    EXPECT_NEAR(disp.x(), 0.0, 1e-12);
    EXPECT_NEAR(disp.y(), 0.0, 1e-12);
    EXPECT_NEAR(disp.z(), 0.0, 1e-12);
}

TEST(IersPoleTide, MagnitudeIsCmLevelForRealisticPolarMotion) {
    // Realistic 2026-04-15 polar motion (TSKB-area, Phase D-1 bench
    // case): xp ≈ 0.137 arcsec, yp ≈ 0.413 arcsec. m1, m2 are both
    // tens of mas → displacement must be sub-cm in any component.
    const double mjd_utc = 61145.0;
    EarthOrientationParams eop{};
    eop.xp_arcsec = 0.137;
    eop.yp_arcsec = 0.413;
    eop.ut1_minus_utc_seconds = 0.0;

    const Eigen::Vector3d disp = poleTideDisplacement(mjd_utc, kTskbEcef, eop);
    EXPECT_LT(disp.norm(), 0.05);   // < 5 cm
    EXPECT_GT(disp.norm(), 1e-6);   // > 1 µm — non-trivial signal
}

TEST(IersPoleTide, EquatorRadialIsZero) {
    // At φ = 0 (geocentric latitude = 0), sin(2θ) = sin(180°) = 0,
    // so the pole-tide RADIAL component vanishes. cos(θ) = cos(90°)
    // = 0 also kills the EAST component. Only NORTH remains.
    const Eigen::Vector3d xsta_eq(6378137.0, 0.0, 0.0);  // equator, λ=0
    EarthOrientationParams eop{};
    eop.xp_arcsec = 0.5;   // arbitrary but non-zero polar motion
    eop.yp_arcsec = 0.5;
    eop.ut1_minus_utc_seconds = 0.0;

    const Eigen::Vector3d disp = poleTideDisplacement(61145.0, xsta_eq, eop);
    // Radial @ equator with λ=0 maps to the +X direction, but with
    // sin(2θ)=0 and cos(θ)=0, only NORTH (+Z direction) survives.
    EXPECT_NEAR(disp.x(), 0.0, 1e-9);   // east + radial both zero at λ=0
    EXPECT_NEAR(disp.y(), 0.0, 1e-9);   // east axis at λ=0 maps to +Y; cos(θ)=0
    EXPECT_GT(std::abs(disp.z()), 1e-6); // north component → +Z at the equator
}

TEST(IersPoleTide, NorthPoleEastAndRadialZero) {
    // At φ = 90° (geographic North Pole), sin(2θ) = sin(0) = 0 and
    // cos(2θ) = 1, cos(θ) = 1. Radial vanishes; east and north both
    // exist as horizontal motions, mapping into ECEF X/Y depending
    // on the (singular) longitude reference.
    const Eigen::Vector3d xsta_np(0.0, 0.0, 6356752.0);
    EarthOrientationParams eop{};
    eop.xp_arcsec = 0.5;
    eop.yp_arcsec = 0.5;
    eop.ut1_minus_utc_seconds = 0.0;

    const Eigen::Vector3d disp = poleTideDisplacement(61145.0, xsta_np, eop);
    // Radial component vanishes at the pole (sin(2θ)=0).
    EXPECT_NEAR(disp.z(), 0.0, 1e-9);
    // Horizontal (X/Y) is non-zero — both north and east contribute.
    EXPECT_GT(disp.head<2>().norm(), 1e-6);
}

TEST(IersPoleTide, SignReversesWhenPolarMotionFlips) {
    // Linearity: flipping (xp, yp) about the mean pole flips m1, m2,
    // and the displacement must flip sign at any station.
    const double mjd_utc = 61145.0;
    const double dt = 2000.0 + (mjd_utc - 51544.5) / 365.25 - 2000.0;
    const double xp_mean = (55.0 + 1.677 * dt) / 1000.0;
    const double yp_mean = (320.5 + 3.460 * dt) / 1000.0;

    EarthOrientationParams ep{};
    ep.xp_arcsec = xp_mean + 0.05;
    ep.yp_arcsec = yp_mean + 0.05;

    EarthOrientationParams en{};
    en.xp_arcsec = xp_mean - 0.05;
    en.yp_arcsec = yp_mean - 0.05;

    const Eigen::Vector3d disp_p = poleTideDisplacement(mjd_utc, kTskbEcef, ep);
    const Eigen::Vector3d disp_n = poleTideDisplacement(mjd_utc, kTskbEcef, en);

    EXPECT_NEAR((disp_p + disp_n).norm(), 0.0, 1e-12);
}
