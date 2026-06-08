// Smoke tests for the vendored ginan-iers2010 library
// (third_party/ginan-iers2010/).
//
// Reference values for dehanttideinel come from the IERS Conventions
// 2010 software distribution test case for routine DEHANTTIDEINEL
// (https://iers-conventions.obspm.fr/content/chapter7/software/dehanttideinel)
// — the same test case ginan itself uses upstream.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <Eigen/Dense>

#include "iers2010.hpp"

TEST(GinanIers2010Vendor, FculaZenithIsUnity) {
    // At elevation = 90° the FCULa total mapping function is 1/sin(90°) = 1.
    // Use a typical mid-latitude station, modest height, ambient temperature.
    const double dlat = 35.6586;     // degrees N (Tokyo Tower latitude — arbitrary)
    const double dhgt = 333.0;       // m
    const double t_kelvin = 288.15;  // K (15 °C)
    const double elev_deg = 90.0;

    const double mf = iers2010::fcul_a(dlat, dhgt, t_kelvin, elev_deg);
    EXPECT_NEAR(mf, 1.0, 1e-6);
}

TEST(GinanIers2010Vendor, FculaIncreasesAsElevationDrops) {
    // The mapping function should grow monotonically as elevation drops
    // (the slant path through the atmosphere lengthens). This is a basic
    // sanity check that does not depend on a specific reference value.
    const double dlat = 35.6586;
    const double dhgt = 333.0;
    const double t_kelvin = 288.15;

    const double mf90 = iers2010::fcul_a(dlat, dhgt, t_kelvin, 90.0);
    const double mf30 = iers2010::fcul_a(dlat, dhgt, t_kelvin, 30.0);
    const double mf10 = iers2010::fcul_a(dlat, dhgt, t_kelvin, 10.0);

    EXPECT_GT(mf30, mf90);
    EXPECT_GT(mf10, mf30);
    // 1/sin(30°) ≈ 2 is the dominant geometric factor at this elevation.
    EXPECT_NEAR(mf30, 2.0, 0.05);
}

TEST(GinanIers2010Vendor, FculZdHpaSucceeds) {
    // Just verify that the zenith-delay routine returns success status
    // and produces non-pathological hydrostatic / wet zenith delays.
    const double dlat = 35.6586;     // degrees N
    const double dhgt = 333.0;       // m
    const double pressure = 1013.25; // hPa
    const double e_water = 12.0;     // hPa partial pressure of water vapor
    const double t_kelvin = 288.15;  // K

    double fcul_ztd = 0.0;
    double fcul_zhd = 0.0;
    double fcul_zwd = 0.0;
    const int rc = iers2010::fcul_zd_hpa(
        dlat, dhgt, pressure, e_water, t_kelvin,
        fcul_ztd, fcul_zhd, fcul_zwd);

    EXPECT_EQ(rc, 0);
    // Standard sea-level hydrostatic zenith delay is ~2.3 m;
    // wet delay is typically a few cm to tens of cm.
    EXPECT_GT(fcul_zhd, 2.0);
    EXPECT_LT(fcul_zhd, 2.6);
    EXPECT_GE(fcul_zwd, 0.0);
    EXPECT_LT(fcul_zwd, 0.5);
    EXPECT_NEAR(fcul_ztd, fcul_zhd + fcul_zwd, 1e-9);
}

TEST(GinanIers2010Vendor, DehanttideinelIersReferenceCase) {
    // Canonical IERS Conventions 2010 software reference test case for
    // DEHANTTIDEINEL (epoch 2009-04-13 0h UT). Inputs and expected
    // outputs are documented in the original FORTRAN routine header.
    //
    //   XSTA = (4075578.385, 931852.890, 4801570.154)  [m]
    //   XSUN = (137859926952.015,  54228127881.4350,  23509422341.6960) [m]
    //   XMON = (-179996231.920342, -312468450.131567, -169288918.592160) [m]
    //   YR=2009 MONTH=4 DAY=13 FHR=0.00
    //
    // Expected DXYZ = (0.07700420357108126,
    //                  0.06304056321824968,
    //                  0.05516568152597247) [m]
    //
    // Time argument computation:
    //   MJD(UTC) = 54934.0
    //   TT - UTC = 66.184 s (UTC+34 leap + 32.184 TAI->TT)
    //   MJD(TT) = 54934 + 66.184/86400
    //   J2000 epoch (TT) = MJD 51544.5
    //   julian_centuries_tt = (MJD(TT) - 51544.5) / 36525
    const double mjd_utc        = 54934.0;
    const double tt_minus_utc_s = 66.184;
    const double mjd_tt         = mjd_utc + tt_minus_utc_s / 86400.0;
    const double t              = (mjd_tt - 51544.5) / 36525.0;
    const double fhr_ut         = 0.0;

    Eigen::Vector3d xsun(137859926952.015,
                          54228127881.4350,
                          23509422341.6960);
    Eigen::Vector3d xmon(-179996231.920342,
                         -312468450.131567,
                         -169288918.592160);

    std::vector<Eigen::Vector3d> xsta_vec = {
        Eigen::Vector3d(4075578.385, 931852.890, 4801570.154)
    };
    std::vector<Eigen::Vector3d> xcor_vec;

    const int rc = iers2010::dehanttideinel_impl(
        t, fhr_ut, xsun, xmon, xsta_vec, xcor_vec);

    ASSERT_EQ(rc, 0);
    ASSERT_EQ(xcor_vec.size(), 1u);

    // Loose tolerance of 1e-6 m (1 µm) absorbs the small bias from our
    // approximate ΔT (66.184 s is a constant; the IERS reference uses
    // tabulated leap seconds + TAI-TT). The displacement itself is ~7 cm
    // so this is a tight functional gate without being overly fragile.
    const Eigen::Vector3d expected(0.07700420357108126,
                                   0.06304056321824968,
                                   0.05516568152597247);
    EXPECT_NEAR(xcor_vec[0].x(), expected.x(), 1e-6);
    EXPECT_NEAR(xcor_vec[0].y(), expected.y(), 1e-6);
    EXPECT_NEAR(xcor_vec[0].z(), expected.z(), 1e-6);
}
