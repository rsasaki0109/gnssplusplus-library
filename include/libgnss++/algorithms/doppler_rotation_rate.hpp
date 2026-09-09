#pragma once
#include <libgnss++/core/types.hpp>
#include <libgnss++/core/constants.hpp>
#include <optional>
#include <cmath>

namespace libgnss::doppler_rotation_rate {
// Rotation-rate-only model. Inputs already in the same receive-frame ECEF.
// Not the full transmit-time derivative; no clock or atmosphere correction.
struct Model {
    double satellite_mps;
    Vector3d receiver_velocity_jacobian;
    double denominator;
};
// Build from the retained unit receiver-to-satellite LOS, avoiding a new
// receiver seed after GNSS-first has updated the same-run trajectory.
inline std::optional<Model> fromLos(const Vector3d& u,
                                    const Vector3d& rotated_satellite,
                                    const Vector3d& rotated_velocity) {
    if (!u.allFinite() || std::abs(u.norm()-1.) > 1e-6 ||
        !rotated_satellite.allFinite() || !rotated_velocity.allFinite()) return std::nullopt;
    const double denominator = 1. - constants::OMEGA_E/constants::SPEED_OF_LIGHT *
        u.dot(Vector3d(rotated_satellite.y(),-rotated_satellite.x(),0.));
    if (!std::isfinite(denominator) || denominator <= 0.) return std::nullopt;
    Model result{u.dot(rotated_velocity)/denominator, -u/denominator, denominator};
    if (!std::isfinite(result.satellite_mps) || !result.receiver_velocity_jacobian.allFinite())
        return std::nullopt;
    return result;
}
inline std::optional<Model> make(const Vector3d& rotated_satellite,
                                const Vector3d& rotated_velocity,
                                const Vector3d& receiver) {
    if(!rotated_satellite.allFinite() || !rotated_velocity.allFinite() ||
       !receiver.allFinite()) return std::nullopt;
    const Vector3d delta=rotated_satellite-receiver;
    const double range=delta.norm();
    if(!std::isfinite(range) || range<=0) return std::nullopt;
    const Vector3d u=delta/range;
    const double k=constants::OMEGA_E/constants::SPEED_OF_LIGHT*
        u.dot(Vector3d(rotated_satellite.y(),-rotated_satellite.x(),0.));
    const double denominator=1.-k;
    if(!std::isfinite(denominator) || denominator<=0.) return std::nullopt;
    Model model{u.dot(rotated_velocity)/denominator,-u/denominator,denominator};
    if(!std::isfinite(model.satellite_mps) || !model.receiver_velocity_jacobian.allFinite())
        return std::nullopt;
    return model;
}
}
