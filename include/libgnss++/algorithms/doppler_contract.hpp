#pragma once

#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/types.hpp>

#include <cmath>
#include <limits>

namespace libgnss::doppler_contract {

/**
 * @brief Convert Android's uncorrected pseudorange rate to RINEX Doppler.
 *
 * Android exposes pseudorange rate in m/s and defines it as
 * ``pseudorange_rate = -k * doppler_shift``.  LibGNSS++ stores the RINEX D
 * observable in Hz, so the conversion is ``D = -rate / wavelength``.
 */
inline double androidRateToRinexDoppler(double pseudorange_rate_mps,
                                        double frequency_hz) {
    if (!std::isfinite(pseudorange_rate_mps) ||
        !(frequency_hz > 0.0) || !std::isfinite(frequency_hz)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double wavelength_m = constants::SPEED_OF_LIGHT / frequency_hz;
    return -pseudorange_rate_mps / wavelength_m;
}

/**
 * @brief Convert a RINEX Doppler observable (Hz) to geometric range rate.
 *
 * RINEX positive Doppler is approaching, hence the leading minus sign.
 */
inline double rinexDopplerToRangeRate(double doppler_hz,
                                      double frequency_hz) {
    if (!std::isfinite(doppler_hz) ||
        !(frequency_hz > 0.0) || !std::isfinite(frequency_hz)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return -doppler_hz * constants::SPEED_OF_LIGHT / frequency_hz;
}

/**
 * @brief Apply the same Earth-rotation correction to satellite position and
 * velocity.
 *
 * This is the no-base counterpart of the existing SPP velocity contract.
 * The rotation is evaluated at the signal travel time and is deliberately
 * kept separate from the historical FGO Doppler path, which remains the
 * default for byte/backward compatibility.
 */
inline bool earthRotationCorrectedSatelliteState(
    const Vector3d& satellite_position_ecef,
    const Vector3d& satellite_velocity_ecef,
    const Vector3d& receiver_position_ecef,
    Vector3d& corrected_position_ecef,
    Vector3d& corrected_velocity_ecef) {
    if (!satellite_position_ecef.allFinite() ||
        !satellite_velocity_ecef.allFinite() ||
        !receiver_position_ecef.allFinite()) {
        return false;
    }
    const Vector3d delta = satellite_position_ecef - receiver_position_ecef;
    const double range_m = delta.norm();
    if (!(range_m > 0.0) || !std::isfinite(range_m)) {
        return false;
    }
    const double angle = constants::OMEGA_E *
                         (range_m / constants::SPEED_OF_LIGHT);
    if (!std::isfinite(angle)) {
        return false;
    }
    Matrix3d earth_rotation;
    earth_rotation << std::cos(angle), std::sin(angle), 0.0,
                      -std::sin(angle), std::cos(angle), 0.0,
                      0.0, 0.0, 1.0;
    corrected_position_ecef = earth_rotation * satellite_position_ecef;
    corrected_velocity_ecef = earth_rotation * satellite_velocity_ecef;
    return corrected_position_ecef.allFinite() &&
           corrected_velocity_ecef.allFinite();
}

/**
 * @brief Receiver-to-satellite unit line of sight and known satellite term.
 */
inline bool receiverToSatelliteGeometry(const Vector3d& satellite_position_ecef,
                                         const Vector3d& receiver_position_ecef,
                                         Vector3d& los_receiver_to_satellite) {
    const Vector3d delta = satellite_position_ecef - receiver_position_ecef;
    const double range_m = delta.norm();
    if (!(range_m > 0.0) || !std::isfinite(range_m) || !delta.allFinite()) {
        return false;
    }
    los_receiver_to_satellite = delta / range_m;
    return los_receiver_to_satellite.allFinite();
}

/**
 * @brief Known satellite range-rate term, excluding receiver velocity/clock.
 *
 * ``satellite_clock_drift_sps`` is in seconds/second and is converted to
 * meters/second by the caller when forming the complete uncorrected Android
 * model.  The corrected branch has no explicit Sagnac addend because the
 * satellite position and velocity are rotated consistently at transmit time.
 */
inline bool knownSatelliteRangeRate(
    const Vector3d& satellite_position_ecef,
    const Vector3d& satellite_velocity_ecef,
    const Vector3d& receiver_position_ecef,
    bool rotate_earth,
    Vector3d& los_receiver_to_satellite,
    double& satellite_range_rate_mps) {
    Vector3d position = satellite_position_ecef;
    Vector3d velocity = satellite_velocity_ecef;
    if (rotate_earth &&
        !earthRotationCorrectedSatelliteState(
            satellite_position_ecef, satellite_velocity_ecef,
            receiver_position_ecef, position, velocity)) {
        return false;
    }
    if (!receiverToSatelliteGeometry(position, receiver_position_ecef,
                                     los_receiver_to_satellite)) {
        return false;
    }
    satellite_range_rate_mps = velocity.dot(los_receiver_to_satellite);
    if (!rotate_earth) {
        // The legacy FGO path models the Earth-rotation contribution as a
        // first-order Sagnac rate.  Keep this exact expression only for the
        // default compatibility path.
        satellite_range_rate_mps +=
            constants::OMEGA_E / constants::SPEED_OF_LIGHT *
            (velocity(1) * receiver_position_ecef(0) -
             velocity(0) * receiver_position_ecef(1));
    }
    return std::isfinite(satellite_range_rate_mps);
}

/**
 * @brief Remove known satellite terms from an uncorrected range rate.
 */
inline double receiverOnlyResidual(double measured_range_rate_mps,
                                   double satellite_range_rate_mps,
                                   double satellite_clock_drift_sps) {
    if (!std::isfinite(measured_range_rate_mps) ||
        !std::isfinite(satellite_range_rate_mps) ||
        !std::isfinite(satellite_clock_drift_sps)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return measured_range_rate_mps -
           (satellite_range_rate_mps -
            satellite_clock_drift_sps * constants::SPEED_OF_LIGHT);
}

/**
 * @brief Receiver-only contribution for a receiver-to-satellite LOS.
 */
inline double receiverPrediction(const Vector3d& los_receiver_to_satellite,
                                 const Vector3d& receiver_velocity_ecef,
                                 double receiver_clock_drift_mps) {
    if (!los_receiver_to_satellite.allFinite() ||
        !receiver_velocity_ecef.allFinite() ||
        !std::isfinite(receiver_clock_drift_mps)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return -los_receiver_to_satellite.dot(receiver_velocity_ecef) +
           receiver_clock_drift_mps;
}

}  // namespace libgnss::doppler_contract
