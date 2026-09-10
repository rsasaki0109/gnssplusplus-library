#pragma once
#include "source_transmission_clock.hpp"
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/constants.hpp>

namespace libgnss::source_transmission_clock {

// Ephemeris selection belongs to the caller. This does not silently select
// another message at the corrected time or alter the final measurement clock.
inline double initialClockSeconds(const Ephemeris& eph, const GNSSTime& time) {
    if (!eph.valid || !std::isfinite(time.tow))
        throw std::invalid_argument("invalid transmission clock ephemeris/time");
    switch (eph.satellite.system) {
        case GNSSSystem::GPS:
        case GNSSSystem::Galileo:
        case GNSSSystem::BeiDou:
        case GNSSSystem::QZSS:
        case GNSSSystem::NavIC:
            return polynomialClockSeconds(time-eph.toc, eph.af0, eph.af1, eph.af2);
        case GNSSSystem::GLONASS:
            return polynomialClockSeconds(time-eph.toe, -eph.glonass_taun,
                                          eph.glonass_gamn, 0.0);
        case GNSSSystem::SBAS: {
            // Literal pinned MALIB seph2clk recurrence, including its plus
            // af1*t term. Do not replace it with eph2clk's polynomial loop.
            const double elapsed = time-eph.toe;
            double t = elapsed;
            for (int i=0; i<2; ++i) t = elapsed-eph.af0+eph.af1*t;
            const double result = eph.af0+eph.af1*t;
            if (!std::isfinite(elapsed) || !std::isfinite(eph.af0) ||
                !std::isfinite(eph.af1) || !std::isfinite(result))
                throw std::invalid_argument("nonfinite SBAS transmission clock");
            return result;
        }
        default: throw std::invalid_argument("unsupported transmission clock system");
    }
}

inline GNSSTime transmissionTime(const GNSSTime& receive_time,
                                 double selected_pseudorange_m,
                                 const Ephemeris& selected_ephemeris) {
    if (!std::isfinite(receive_time.tow) || receive_time.tow < 0.0 ||
        receive_time.tow >= 604800.0 || !std::isfinite(selected_pseudorange_m) ||
        selected_pseudorange_m <= 0.0)
        throw std::invalid_argument("invalid transmission receive time/pseudorange");
    const auto satellite_time = receive_time-selected_pseudorange_m/constants::SPEED_OF_LIGHT;
    return satellite_time-initialClockSeconds(selected_ephemeris, satellite_time);
}

struct SelectedEphemerisState {
    GNSSTime transmit_time;
    Vector3d position_ecef;
    Vector3d velocity_ecef;
    double clock_seconds;
    double clock_drift;
};

// Use one caller-selected message for both propagation evaluations. Native
// orbital propagation is retained; this does not claim RTKLIB orbit parity.
inline SelectedEphemerisState stateFromSelectedEphemeris(
    const GNSSTime& receive_time, double pseudorange_m, const Ephemeris& eph) {
    SelectedEphemerisState result;
    result.transmit_time = transmissionTime(receive_time, pseudorange_m, eph);
    Vector3d unused_velocity, forward_position;
    double unused_drift, forward_clock;
    constexpr double step_s = 0.001;
    if (!eph.calculateSatelliteState(result.transmit_time, result.position_ecef,
                                    unused_velocity, result.clock_seconds, unused_drift) ||
        !eph.calculateSatelliteState(result.transmit_time+step_s, forward_position,
                                    unused_velocity, forward_clock, unused_drift)) {
        throw std::invalid_argument("selected ephemeris propagation failed");
    }
    result.velocity_ecef = (forward_position-result.position_ecef)/step_s;
    result.clock_drift = (forward_clock-result.clock_seconds)/step_s;
    if (!result.position_ecef.allFinite() || !result.velocity_ecef.allFinite() ||
        !std::isfinite(result.clock_seconds) || !std::isfinite(result.clock_drift)) {
        throw std::invalid_argument("selected ephemeris state is nonfinite");
    }
    return result;
}
} // namespace libgnss::source_transmission_clock
