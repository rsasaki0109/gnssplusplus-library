// Tests for libgnss::iers::* sun/moon ephemeris helpers
// (include/libgnss++/iers/ephemeris.hpp).
//
// We use a mix of:
//   - Magnitude bounds (Earth-Sun ~1 AU ± 2%, Earth-Moon ~384,400 km
//     ± 14%) — frame-independent, only assumes physical orbit ranges.
//   - Direction sweep over a quarter / half year so the Sun's
//     ICRS angular position advances as expected.
//   - Integration smoke: feeding our SOFA-derived sun/moon vectors
//     into solidEarthTideDisplacement at the IERS reference epoch
//     should produce a displacement close to the published reference
//     (~7 cm magnitude), validating the wrapper as a self-contained
//     unit.

#include <gtest/gtest.h>

#include <cmath>

#include <Eigen/Dense>

#include "libgnss++/iers/ephemeris.hpp"
#include "libgnss++/iers/tides.hpp"

using libgnss::iers::moonPositionIcrs;
using libgnss::iers::solidEarthTideDisplacement;
using libgnss::iers::sunPositionIcrs;

namespace {
constexpr double kAuMeters = 149597870700.0;
}  // namespace

TEST(IersEphemeris, SunIcrsMagnitudeIsOneAU) {
    // 2020-01-01 12:00:00 UTC = MJD 58849.5
    const Eigen::Vector3d sun = sunPositionIcrs(58849.5);
    const double r = sun.norm();
    // Earth's orbit eccentricity is ~0.0167, so |Earth-Sun| varies
    // between ~0.983 AU and ~1.017 AU over the year. Allow 3%.
    EXPECT_GT(r, 0.97 * kAuMeters);
    EXPECT_LT(r, 1.03 * kAuMeters);
}

TEST(IersEphemeris, MoonIcrsMagnitudeNearLunarOrbit) {
    // Lunar orbit eccentricity is ~0.055; perigee ~363,000 km,
    // apogee ~406,000 km.
    const Eigen::Vector3d moon = moonPositionIcrs(58849.5);
    const double r = moon.norm();
    EXPECT_GT(r, 350.0e6);
    EXPECT_LT(r, 410.0e6);
}

TEST(IersEphemeris, SunDirectionSweepsOverHalfYear) {
    // The Sun's geocentric direction in ICRS sweeps 360° per year.
    // Sample at 0 / 90 / 180 days from MJD 58849.5 (2020-01-01 12 UT)
    // and check the angular separation from the start point. We
    // restrict the check to the [0, 180°] range so the angle-between
    // -vectors metric (always in that range) is unambiguous.
    auto angle_deg = [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        const double cosang = a.normalized().dot(b.normalized());
        return std::acos(std::min(1.0, std::max(-1.0, cosang))) * 180.0 / M_PI;
    };

    const Eigen::Vector3d s0  = sunPositionIcrs(58849.5);
    const Eigen::Vector3d s90 = sunPositionIcrs(58849.5 + 90.0);
    const Eigen::Vector3d s180 = sunPositionIcrs(58849.5 + 180.0);

    // After 90 days the Sun's ICRS direction has advanced by
    // ~90 * 360/365.25 ≈ 88.7°.
    EXPECT_NEAR(angle_deg(s0, s90), 90.0 * 360.0 / 365.25, 5.0);
    // After 180 days, ~177.4°.
    EXPECT_NEAR(angle_deg(s0, s180), 180.0 * 360.0 / 365.25, 5.0);
}

TEST(IersEphemeris, SolidEarthTideUsingComputedEphemeris) {
    // Integration test: at the IERS Conventions 2010 reference epoch
    // (2009-04-13 0h UTC, MJD 54934.0) feed our SOFA-computed sun and
    // moon ICRS vectors into solidEarthTideDisplacement. Compare to
    // the published reference value.
    //
    // The IERS reference test in test_iers_tides.cpp uses hand-supplied
    // sun/moon vectors (also in ICRS, despite the IERS routine docs
    // calling it "ECEF"). Here we replace those hand-supplied vectors
    // with SOFA-computed ones; small differences arise because:
    //   - iauEpv00 vs. JPL DE405 differ by ~15 km at the Sun.
    //   - iauMoon98 vs. JPL DE405 differ by ~1 km at the Moon.
    //   - Inverse-cube weighting in the tide formula amplifies any
    //     position error to ~3× relative error in displacement.
    // Net expected drift: well under 1 mm in the final displacement,
    // far below the model's intrinsic accuracy. Tolerance set to
    // 1 mm.
    const double mjd_utc = 54934.0;

    const Eigen::Vector3d xsta(4075578.385, 931852.890, 4801570.154);
    const Eigen::Vector3d xsun = sunPositionIcrs(mjd_utc);
    const Eigen::Vector3d xmon = moonPositionIcrs(mjd_utc);

    const Eigen::Vector3d dxyz =
        solidEarthTideDisplacement(mjd_utc, xsta, xsun, xmon);

    const Eigen::Vector3d expected(0.07700420357108126,
                                   0.06304056321824968,
                                   0.05516568152597247);

    EXPECT_NEAR(dxyz.x(), expected.x(), 1e-3);
    EXPECT_NEAR(dxyz.y(), expected.y(), 1e-3);
    EXPECT_NEAR(dxyz.z(), expected.z(), 1e-3);
}
