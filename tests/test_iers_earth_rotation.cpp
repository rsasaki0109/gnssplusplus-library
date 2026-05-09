// Tests for libgnss::iers::* earth-rotation primitives
// (include/libgnss++/iers/earth_rotation.hpp).
//
// These tests validate the wrapper layer over the vendored IAU SOFA
// library; SOFA itself is exercised by tests/test_sofa_vendor.cpp.

#include <gtest/gtest.h>

#include <cmath>

#include <Eigen/Dense>

#include "libgnss++/iers/earth_rotation.hpp"

using libgnss::GNSSTime;
using libgnss::iers::EarthOrientationParams;
using libgnss::iers::earthRotationAngle;
using libgnss::iers::gnssTimeToMjdUtc;
using libgnss::iers::icrsToItrs;
using libgnss::iers::itrsToIcrs;

TEST(IersEarthRotation, GnssTimeToMjdUtcAtGpsEpoch) {
    // GPS epoch = 1980-01-06 00:00:00 UTC = MJD 44244.0 exactly.
    // At GPS epoch ΔAT was 19, so the leap-second correction is 0.
    const GNSSTime epoch{0, 0.0};
    EXPECT_NEAR(gnssTimeToMjdUtc(epoch), 44244.0, 1e-9);
}

TEST(IersEarthRotation, GnssTimeToMjdUtcRecentEpoch) {
    // 2020-01-01 00:00:00 UTC = MJD 58849.0 (ΔAT = 37 since 2017).
    // GPS time at this UTC instant:
    //   GPS = TAI - 19 = UTC + (37 - 19) = UTC + 18 s.
    // Days from 1980-01-06 to 2020-01-01 = 14605
    //   week = 14605 / 7 = 2086.428..., week count = 2086, day = 3
    //   (Wed). tow at 00:00:00 GPS on 2020-01-01:
    //   day_of_week_at_2020_01_01 = (4 + 14605) mod 7 = 3 (Wed).
    //   GPS week = 14605 / 7 = 2086 (rounding toward zero).
    //   Actually: GPS week 2086 began 2020-01-05 (Sun); we want
    //   2020-01-01 which is in GPS week 2085. Let's compute directly:
    //   total GPS seconds since epoch =
    //       14605 * 86400 + 18 = 1,261,872,018 s
    //   week = floor(1261872018 / 604800) = 2086 (just barely; 2086 *
    //   604800 = 1,261,612,800; remainder 259,218 s -> tow). Hmm,
    //   that puts 2020-01-01 into GPS week 2086, day-of-week 3, which
    //   matches Wed. Use this.
    constexpr double kSecondsPerWeek = 604800.0;
    const double gps_seconds = 14605.0 * 86400.0 + 18.0;
    const int week = static_cast<int>(std::floor(gps_seconds / kSecondsPerWeek));
    const double tow = gps_seconds - week * kSecondsPerWeek;
    const GNSSTime t{week, tow};

    EXPECT_NEAR(gnssTimeToMjdUtc(t), 58849.0, 1e-6);
}

TEST(IersEarthRotation, EarthRotationAngleIsInRange) {
    // ERA must lie in [0, 2π). Sample a non-special epoch.
    // 2020-01-01 12:00:00 UTC = MJD 58849.5
    EarthOrientationParams eop;
    const double era = earthRotationAngle(58849.5, eop);
    EXPECT_GE(era, 0.0);
    EXPECT_LT(era, 2.0 * M_PI);
}

TEST(IersEarthRotation, EarthRotationAngleAdvances) {
    // Earth rotates ~360°/sidereal-day. After 12 sidereal hours
    // (≈ 11h 58m of UT1), ERA should advance by ~π radians.
    // Using exact UT1 days (12 sidereal hours = 12/24 sidereal days
    // ≈ 0.4972695 UT1 days) takes us close to π.
    EarthOrientationParams eop;
    const double mjd_a = 58849.0;
    const double mjd_b = 58849.5;
    const double era_a = earthRotationAngle(mjd_a, eop);
    const double era_b = earthRotationAngle(mjd_b, eop);
    double diff = era_b - era_a;
    if (diff < 0) diff += 2.0 * M_PI;
    // Earth makes one rotation per sidereal day (~23h 56m), so 12 h
    // of UT1 advances ERA by slightly more than π.
    EXPECT_NEAR(diff, M_PI, 0.01);
}

TEST(IersEarthRotation, IcrsToItrsIsOrthogonal) {
    // The full ICRS->ITRS rotation is an orthogonal matrix
    // (proper rotation, det = +1).
    EarthOrientationParams eop;
    eop.xp_arcsec = 0.123;
    eop.yp_arcsec = 0.456;
    eop.ut1_minus_utc_seconds = -0.2;
    const Eigen::Matrix3d R = icrsToItrs(58849.5, eop);

    // R^T * R should be the identity.
    const Eigen::Matrix3d ortho_residual =
        R.transpose() * R - Eigen::Matrix3d::Identity();
    EXPECT_LT(ortho_residual.cwiseAbs().maxCoeff(), 1e-12);

    // det(R) = +1 (proper rotation).
    EXPECT_NEAR(R.determinant(), 1.0, 1e-12);
}

TEST(IersEarthRotation, ItrsToIcrsIsInverse) {
    EarthOrientationParams eop;
    const Eigen::Matrix3d R_c2t = icrsToItrs(58849.5, eop);
    const Eigen::Matrix3d R_t2c = itrsToIcrs(58849.5, eop);

    const Eigen::Matrix3d roundtrip =
        R_t2c * R_c2t - Eigen::Matrix3d::Identity();
    EXPECT_LT(roundtrip.cwiseAbs().maxCoeff(), 1e-12);
}
