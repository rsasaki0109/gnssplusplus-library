#include "libgnss++/iers/tides.hpp"

#include <cmath>
#include <vector>

#include "iers2010.hpp"

extern "C" {
#include "sofa.h"
}

namespace libgnss::iers {

namespace {
constexpr double kMjdJ2000Epoch  = 2400000.5;     // MJD <-> JD offset
constexpr double kJdJ2000        = 2451545.0;     // J2000.0 in JD (TT)
constexpr double kJulianCentury  = 36525.0;       // days per Julian century
}  // namespace

Eigen::Vector3d solidEarthTideDisplacement(
    double mjd_utc,
    const Eigen::Vector3d& xsta_ecef,
    const Eigen::Vector3d& xsun_ecef,
    const Eigen::Vector3d& xmon_ecef) {
    // dehanttideinel_impl wants:
    //   julian_centuries_tt — TT in Julian centuries from J2000
    //   fhr_ut              — fractional hour UT of the day
    //
    // For TT we go through SOFA's leap-second-aware UTC -> TAI -> TT
    // chain so the result is correct regardless of what leap-second
    // table the caller has in mind.
    double tai1 = 0.0;
    double tai2 = 0.0;
    double tt1  = 0.0;
    double tt2  = 0.0;
    iauUtctai(kMjdJ2000Epoch, mjd_utc, &tai1, &tai2);
    iauTaitt(tai1, tai2, &tt1, &tt2);

    const double centuries_tt =
        ((tt1 - kJdJ2000) + tt2) / kJulianCentury;

    // The Dehant model uses fhr_ut at the ~10 ms level only (it
    // affects the diurnal Step-2 corrections), so taking UT ≈ UTC
    // here introduces a sub-millimeter displacement error — well
    // below the model's intrinsic accuracy.
    const double fhr_ut = (mjd_utc - std::floor(mjd_utc)) * 24.0;

    std::vector<Eigen::Vector3d> xsta_vec = { xsta_ecef };
    std::vector<Eigen::Vector3d> xcor_vec;
    xcor_vec.reserve(1);

    iers2010::dehanttideinel_impl(
        centuries_tt, fhr_ut, xsun_ecef, xmon_ecef, xsta_vec, xcor_vec);

    if (xcor_vec.empty()) {
        return Eigen::Vector3d::Zero();
    }
    return xcor_vec.front();
}

}  // namespace libgnss::iers
