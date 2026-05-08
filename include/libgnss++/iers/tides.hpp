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
/// to the station's nominal ECEF coordinates to obtain the tidally
/// displaced instantaneous position.
///
/// @param mjd_utc        UTC modified Julian date of the epoch.
/// @param xsta_ecef      Station nominal ECEF coordinates [m].
/// @param xsun_ecef      Sun ECEF coordinates [m] at this epoch.
///                       Must be supplied by the caller; libgnss++
///                       does not yet provide a built-in solar
///                       ephemeris (planned for a follow-up PR).
/// @param xmon_ecef      Moon ECEF coordinates [m] at this epoch.
///                       Same caveat as xsun_ecef.
/// @return Displacement vector [m]; add to xsta_ecef to obtain the
///         instantaneously displaced station position.
Eigen::Vector3d solidEarthTideDisplacement(
    double mjd_utc,
    const Eigen::Vector3d& xsta_ecef,
    const Eigen::Vector3d& xsun_ecef,
    const Eigen::Vector3d& xmon_ecef);

}  // namespace libgnss::iers
