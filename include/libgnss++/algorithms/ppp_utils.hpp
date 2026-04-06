#pragma once

#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/types.hpp>

#include <cmath>
#include <vector>

namespace libgnss::ppp_utils {

/// Find the first valid carrier-phase observation matching one of the candidate signals.
inline const Observation* findCarrierObservation(
    const ObservationData& obs,
    const SatelliteId& sat,
    const std::vector<SignalType>& candidates) {
    for (const auto signal : candidates) {
        const Observation* candidate = obs.getObservation(sat, signal);
        if (candidate != nullptr && candidate->valid &&
            candidate->has_carrier_phase &&
            std::isfinite(candidate->carrier_phase)) {
            return candidate;
        }
    }
    return nullptr;
}

/// Convert SSR orbit correction from RAC (Radial/Along/Cross) to ECEF.
/// RTCM/CLASLIB convention: along=velocity, cross=pos×vel, radial=along×cross.
/// Returns the correction in ECEF (to be added to satellite position).
inline Vector3d ssrRacToEcef(
    const Vector3d& position_ecef,
    const Vector3d& velocity_ecef,
    const Vector3d& rac_correction) {
    const double velocity_norm = velocity_ecef.norm();
    const Vector3d cross_track = position_ecef.cross(velocity_ecef);
    const double cross_norm = cross_track.norm();
    if (velocity_norm <= 0.0 || cross_norm <= 0.0) {
        return Vector3d::Zero();
    }
    const Vector3d along = velocity_ecef / velocity_norm;
    const Vector3d cross = cross_track / cross_norm;
    const Vector3d radial = along.cross(cross);
    return -(radial * rac_correction.x() +
             along * rac_correction.y() +
             cross * rac_correction.z());
}

}  // namespace libgnss::ppp_utils
