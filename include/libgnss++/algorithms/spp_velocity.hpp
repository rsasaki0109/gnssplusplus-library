#pragma once

#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/navigation.hpp>

#include <Eigen/Dense>

#include <vector>

namespace libgnss {
namespace spp_velocity {

/**
 * @brief One range-rate observation ready for the Doppler velocity least
 * squares solve.
 *
 * The satellite position/velocity must already be corrected into the same
 * ECEF frame as receiver_position_ecef (i.e. Earth-rotation/Sagnac corrected
 * to the signal-reception epoch), mirroring how SPPProcessor's own position
 * least squares rotates the satellite position before use
 * (src/algorithms/spp.cpp's `corrected_sat_pos`). Callers building this
 * struct should reuse whatever satellite-state computation they already do
 * for the position solve rather than recomputing it independently.
 */
struct DopplerObservation {
    Eigen::Vector3d satellite_position_ecef = Eigen::Vector3d::Zero();
    Eigen::Vector3d satellite_velocity_ecef = Eigen::Vector3d::Zero();
    double satellite_clock_drift = 0.0;  ///< s/s
    double doppler_hz = 0.0;             ///< RINEX-convention Doppler (positive = approaching satellite)
    double frequency_hz = 0.0;           ///< Carrier frequency of the Doppler observation, Hz
    double elevation_rad = 0.0;          ///< Used only for elevation-dependent weighting
};

struct DopplerVelocityResult {
    bool ok = false;
    Eigen::Vector3d velocity_ecef = Eigen::Vector3d::Zero();
    Eigen::Matrix3d velocity_covariance = Eigen::Matrix3d::Zero();
    double receiver_clock_drift = 0.0;  ///< s/s
    double clock_drift_variance = 0.0;
    int num_satellites_used = 0;
    double residual_rms_mps = 0.0;
};

/**
 * @brief Estimate receiver ECEF velocity and clock drift from Doppler
 * range-rate observations via weighted least squares.
 *
 * Standard formulation (matches RTKLIB's estvel()/resdop()): unknowns are the
 * receiver velocity (3) and receiver clock drift expressed in m/s (1). Each
 * observation contributes a range-rate residual
 * `(-doppler*c/freq) - (dot(sat_vel - rx_vel, los) + rx_clock_drift_mps -
 * c*sat_clock_drift)`, which is linear in the unknowns -- unlike the position
 * solve, no iteration is needed since the receiver position (and therefore
 * the line-of-sight vectors) is already known/fixed here.
 *
 * Requires at least `min_satellites` usable observations (finite Doppler,
 * frequency, satellite state); otherwise returns `ok=false` and leaves the
 * outputs at their defaults so callers can cleanly gate `has_velocity` on
 * `ok` without special-casing a partially-populated result.
 */
DopplerVelocityResult solveVelocity(const std::vector<DopplerObservation>& observations,
                                    const Eigen::Vector3d& receiver_position_ecef,
                                    double doppler_sigma_mps = 0.5,
                                    int min_satellites = 4);

/**
 * @brief Convenience entry point for callers that only have a raw
 * ObservationData epoch (e.g. RTKProcessor's rover observations) rather than
 * a pre-built satellite-state list.
 *
 * Looks up each Doppler-bearing observation's broadcast-ephemeris satellite
 * position/velocity/clock-drift via `nav.calculateSatelliteState()` (the
 * same call SPPProcessor's own position solve uses), applies the same
 * Earth-rotation/Sagnac frame correction SPPProcessor applies to its
 * position solve, then delegates to `solveVelocity()`. This is the
 * "SPP-style Doppler LS on the rover obs" path RTKProcessor uses -- it does
 * not depend on any RTK-specific state (DD pairs, ambiguities, etc).
 */
DopplerVelocityResult solveVelocityFromObservations(const ObservationData& obs,
                                                    const NavigationData& nav,
                                                    const Eigen::Vector3d& receiver_position_ecef,
                                                    double doppler_sigma_mps = 0.5,
                                                    int min_satellites = 4);

}  // namespace spp_velocity
}  // namespace libgnss
