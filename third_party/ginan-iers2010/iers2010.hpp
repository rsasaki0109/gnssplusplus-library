// IERS2010 wrapper header.
//
// Adapted from GeoscienceAustralia/ginan@535ef0a (release v4.1.1):
//     src/cpp/3rdparty/iers2010/iers2010.hpp
//
// Modifications from upstream (per Apache License 2.0 attribution
// requirements; see ./README.md for the full list):
//   - Removed `#include "common/gTime.hpp"` (HARDISP / GTime-dependent
//     declarations not vendored in this PR).
//   - Removed the `namespace hisp { ... }` section (HARDISP ocean tide
//     loading routines not vendored; will land in a separate PR with
//     native libgnss++ time-type adaptation).
//   - Replaced `#include "common/eigenIncluder.hpp"` with a direct
//     `#include <Eigen/Dense>` so this public header does not depend on
//     the libgnss++ internal `shim/` (which is PRIVATE to the vendored
//     .cpp files only).
//
// Original ginan code is licensed under Apache License 2.0; see
// ./LICENSE in this directory.

#pragma once

#include <vector>

#include <Eigen/Dense>

namespace iers2010
{
/// @brief Equatorial radius of the Earth [m].
/// @see Table 1.1: IERS numerical standards, IERS 2010
constexpr const double Re = 6'378'136.6e0;

/// Compute the global total FCULa mapping function.
double fcul_a(double, double, double, double) noexcept;
/*
/// Computes the global total FCULb mapping function.
double fcul_b(double, double, double, double) noexcept;
*/
// Determine the total zenith delay following Mendes and Pavlis, 2004.
int fcul_zd_hpa(double, double, double, double, double, double &, double &, double &) noexcept;

/// @brief Compute tidal corrections of station displacements caused by lunar
/// and solar gravitational attraction. This is just the implementation, use the
/// generic template function instead.
int dehanttideinel_impl(
    double julian_centuries_tt, double fhr_ut,
    const Eigen::Matrix<double, 3, 1> &xsun,
    const Eigen::Matrix<double, 3, 1> &xmon,
    const std::vector<Eigen::Matrix<double, 3, 1>> &xsta_vec,
    std::vector<Eigen::Matrix<double, 3, 1>> &xcor_vec) noexcept;

};
