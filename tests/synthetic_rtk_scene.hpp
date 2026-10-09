// Synthetic GPS scene shared by the RTK / online-processor unit tests: a
// 24-satellite constellation and L1 code+phase observations modelled with the
// same light-time / Earth-rotation geometry the processors use. Header-only.
#pragma once

#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>

#include <cmath>
#include <vector>

namespace synthetic_rtk_scene {

using namespace libgnss;

inline GNSSTime epochTime(double offset) { return GNSSTime(2300, 100000.0 + offset); }

inline NavigationData constellation() {
    NavigationData nav;
    for (uint8_t prn = 1; prn <= 24; ++prn) {
        Ephemeris eph;
        eph.satellite = SatelliteId(GNSSSystem::GPS, prn);
        eph.valid = true;
        eph.week = 2300;
        eph.toe = GNSSTime(2300, 100000.0);
        eph.toc = eph.toe;
        eph.tof = eph.toe;
        eph.toes = eph.toe.tow;
        eph.sqrt_a = std::sqrt(26560000.0);
        eph.e = 0.004 + 0.0002 * prn;
        eph.i0 = 0.94 + 0.01 * (prn % 3);
        eph.omega0 = 0.35 * prn;
        eph.omega = 0.17 * prn;
        eph.m0 = 0.61 * prn;
        eph.delta_n = 1e-9 * prn;
        eph.omega_dot = -8.0e-9;
        nav.addEphemeris(eph);
    }
    return nav;
}

struct SatGeometry {
    bool ok = false;
    double range_m = 0.0;
    double clock_s = 0.0;
    double elevation_rad = 0.0;
    // Doppler (Hz, RINEX sign) of a stationary receiver with zero clock drift.
    double doppler_hz = 0.0;
};

inline SatGeometry geometry(const NavigationData& nav, uint8_t prn, const GNSSTime& time,
                     const Vector3d& receiver) {
    SatGeometry out;
    const SatelliteId sat(GNSSSystem::GPS, prn);
    double travel = 0.075;
    Vector3d position, velocity;
    double clock = 0.0, drift = 0.0;
    Vector3d rotated = Vector3d::Zero();
    Vector3d rotated_velocity = Vector3d::Zero();
    for (int iteration = 0; iteration < 3; ++iteration) {
        const GNSSTime tx = time - travel;
        if (!nav.calculateSatelliteState(sat, tx, position, velocity, clock, drift)) return out;
        const double angle = constants::OMEGA_E * travel;
        Matrix3d rotation;
        rotation << std::cos(angle), std::sin(angle), 0.0,
                   -std::sin(angle), std::cos(angle), 0.0,
                    0.0, 0.0, 1.0;
        rotated = rotation * position;
        rotated_velocity = rotation * velocity;
        travel = (rotated - receiver).norm() / constants::SPEED_OF_LIGHT;
    }
    out.ok = true;
    out.range_m = (rotated - receiver).norm();
    out.clock_s = clock;
    out.elevation_rad = nav.calculateGeometry(receiver, rotated).elevation;
    {
        constexpr double kL1WavelengthM = 0.19029367;
        const Vector3d line_of_sight = (rotated - receiver).normalized();
        const double known_range_rate = rotated_velocity.dot(line_of_sight) -
                                        constants::SPEED_OF_LIGHT * drift;
        out.doppler_hz = -known_range_rate / kL1WavelengthM;
    }
    return out;
}

inline std::vector<uint8_t> visible(const NavigationData& nav, const GNSSTime& time,
                             const Vector3d& receiver, double min_elevation_deg) {
    std::vector<uint8_t> prns;
    for (uint8_t prn = 1; prn <= 24; ++prn) {
        const auto g = geometry(nav, prn, time, receiver);
        if (g.ok && g.elevation_rad >= min_elevation_deg * M_PI / 180.0) prns.push_back(prn);
    }
    return prns;
}

inline ObservationData observations(const NavigationData& nav, const GNSSTime& time,
                             const Vector3d& receiver, double clock_bias_m,
                             const std::vector<uint8_t>& prns, double ambiguity_offset,
                             bool with_doppler = false) {
    ObservationData data(time);
    constexpr double kWavelengthM = 0.19029367;
    for (const uint8_t prn : prns) {
        const auto g = geometry(nav, prn, time, receiver);
        if (!g.ok) continue;
        Observation obs(SatelliteId(GNSSSystem::GPS, prn), SignalType::GPS_L1CA);
        obs.pseudorange = g.range_m + clock_bias_m - constants::SPEED_OF_LIGHT * g.clock_s;
        obs.has_pseudorange = true;
        obs.carrier_phase = (g.range_m - constants::SPEED_OF_LIGHT * g.clock_s) / kWavelengthM +
                            ambiguity_offset + 1000.0 * prn;
        obs.has_carrier_phase = true;
        obs.snr = 45.0;
        if (with_doppler) {
            obs.doppler = g.doppler_hz;
            obs.has_doppler = true;
        }
        data.addObservation(obs);
    }
    return data;
}


}  // namespace synthetic_rtk_scene
