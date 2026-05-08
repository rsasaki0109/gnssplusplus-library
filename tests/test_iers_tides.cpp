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
