#pragma once

// libgnss++/iers/tides.hpp
//
// Thin C++ wrappers over the vendored ginan-iers2010 library
// (third_party/ginan-iers2010/), surfacing the IERS Conventions 2010
// solid-earth-tide displacement model in libgnss-native types.
//
// At the centimeter level, the solid-earth tide is the largest single
// non-trivial PPP correction (peak diurnal/semidiurnal amplitude is
// ~30 cm radial, ~10 cm horizontal). Ignoring it sets a hard floor
// on PPP positioning accuracy.

#include <Eigen/Dense>

#include "libgnss++/core/types.hpp"

namespace libgnss::iers {

/// @brief Solid-earth-tide station displacement.
///
/// Wraps `iers2010::dehanttideinel_impl`, which implements IERS
/// Conventions 2010 §7.1.1 (Dehant et al.) — the standard model for
/// solid-earth-tide displacement of a ground station due to lunisolar
/// gravitational forcing. The result is a 3-vector (m) to be ADDED
/// to the station's nominal ITRS coordinates to obtain the tidally
/// displaced instantaneous position.
///
/// FRAME NOTE: although the IERS Conventions 2010 routine
/// documentation describes all inputs as "ECEF", the published
/// reference test case actually uses ICRS positions for Sun and Moon
/// while keeping the station in ITRS. The libgnss::iers wrapper
/// follows the *test-case convention* (which defines the routine's
/// behavior in practice), so:
///
///   - `xsta_itrs` should be the station's ITRS / ECEF position.
///   - `xsun_icrs` and `xmon_icrs` should be Sun / Moon positions in
///     the ICRS / GCRS — *not* rotated to ITRS.
///
/// `libgnss::iers::sunPositionIcrs` and `moonPositionIcrs` (in
/// libgnss++/iers/ephemeris.hpp) return values in the correct frame
/// for direct use here.
///
/// @param mjd_utc        UTC modified Julian date of the epoch.
/// @param xsta_itrs      Station nominal ITRS / ECEF coordinates [m].
/// @param xsun_icrs      Sun ICRS / GCRS coordinates [m] at this epoch.
/// @param xmon_icrs      Moon ICRS / GCRS coordinates [m] at this epoch.
/// @return Displacement vector [m] in the station's ITRS frame; add
///         to `xsta_itrs` to obtain the instantaneously displaced
///         station position.
Eigen::Vector3d solidEarthTideDisplacement(
    double mjd_utc,
    const Eigen::Vector3d& xsta_itrs,
    const Eigen::Vector3d& xsun_icrs,
    const Eigen::Vector3d& xmon_icrs);

}  // namespace libgnss::iers
