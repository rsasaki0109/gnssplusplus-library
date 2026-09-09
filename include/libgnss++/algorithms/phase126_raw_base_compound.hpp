#pragma once

/**
 * @file phase126_raw_base_compound.hpp
 * @brief Source-locked, truth-free primitives for the Phase126 raw-base port.
 *
 * This header deliberately contains only deterministic geometry/reference
 * helpers.  The compound selector is wired by the native application and is
 * never enabled by default.  In particular, this API has no file, truth,
 * coordinate-output, or optimizer dependency.
 */

#include <libgnss++/core/types.hpp>

#include <string>

namespace libgnss::phase126_raw_base {

/**
 * The RINEX APPROX POSITION XYZ convention admitted by Phase126.  The
 * approximate XYZ is the antenna reference position.  ANTENNA: DELTA H/E/N
 * is retained as provenance, but is not silently added a second time.  A
 * caller that has an independently proven marker-to-antenna convention may
 * opt into the explicit ENU conversion through antennaReferenceEcef().
 */
struct StationReference {
    Vector3d approximate_position_ecef = Vector3d::Zero();
    Vector3d antenna_delta_enu = Vector3d::Zero();
    bool has_approximate_position = false;
    bool has_antenna_delta = false;
    bool antenna_reference_convention_proven = false;
};

/** Validate the finite Earth-valid header reference without any repair. */
bool validateStationReference(const StationReference& reference,
                              std::string& failure);

/** Convert an ENU antenna delta at an ECEF reference to an ECEF delta. */
Vector3d enuDeltaToEcef(const Vector3d& reference_ecef,
                        const Vector3d& delta_enu);

/**
 * Return the antenna reference represented by a verified header convention.
 * If apply_delta is false, the RINEX APPROX POSITION XYZ is used verbatim.
 * Non-finite inputs produce a non-finite vector; callers must fail closed.
 */
Vector3d antennaReferenceEcef(const StationReference& reference,
                             bool apply_delta);

struct Geometry {
    double range_m = 0.0;
    double elevation_rad = 0.0;
    double azimuth_rad = 0.0;
    Vector3d line_of_sight = Vector3d::Zero();
};

/**
 * RTKLIB geodist-compatible ECEF geometry.  The LOS is formed from the
 * unrotated satellite vector and the range receives exactly one Sagnac term:

 *   rho = ||rs-rr|| + OMEGA_E/CLIGHT * (rs_x rr_y - rs_y rr_x).
 *
 * This representation is intentionally separate from the legacy explicit
 * satellite rotation path so a Phase126 caller cannot accidentally apply
 * both corrections.
 */
Geometry geodistWithSagnac(const Vector3d& receiver_ecef,
                            const Vector3d& satellite_ecef);

/** Official Gobs.resPc code residual, all arguments and result in metres. */
double officialBaseCodeResidual(double pseudorange_m,
                                double satellite_clock_m,
                                double geometric_range_m,
                                double ionosphere_m,
                                double troposphere_m);

/** Phase126's symmetric official no-explicit-TGD/BGD policy. */
inline double officialExplicitCodeBiasMeters() noexcept { return 0.0; }

}  // namespace libgnss::phase126_raw_base
