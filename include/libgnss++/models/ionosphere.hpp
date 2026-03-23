#pragma once

/**
 * @file ionosphere.hpp
 * @brief Ionospheric delay models for GNSS processing.
 *
 * Provides the Klobuchar broadcast ionosphere model and frequency-based
 * ionosphere scaling utilities.
 */

#include <libgnss++/core/constants.hpp>
#include <cmath>

namespace libgnss {
namespace models {

/**
 * @brief Klobuchar broadcast ionosphere model.
 *
 * Computes the L1 ionospheric delay using the 8-parameter Klobuchar model
 * broadcast in the GPS navigation message.  Accuracy is roughly 50% RMS
 * reduction of the total ionospheric delay.
 *
 * @param lat   Geodetic latitude of receiver (radians)
 * @param lon   Geodetic longitude of receiver (radians)
 * @param az    Azimuth to satellite (radians)
 * @param el    Elevation to satellite (radians)
 * @param tow   GPS time of week (seconds)
 * @param alpha Klobuchar alpha parameters (4 values); nullptr for defaults
 * @param beta  Klobuchar beta parameters (4 values); nullptr for defaults
 * @return Ionospheric delay on L1 in meters
 */
inline double ionoDelayKlobuchar(double lat, double lon, double az, double el,
                                  double tow,
                                  const double* alpha, const double* beta) {
    // Default Klobuchar parameters (average ionosphere)
    static constexpr double default_alpha[4] = {0.1118e-07, -0.7451e-08, -0.5961e-07, 0.1192e-06};
    static constexpr double default_beta[4]  = {0.1167e+06, -0.2294e+06, -0.1311e+06, 0.1049e+07};

    const double* a = alpha ? alpha : default_alpha;
    const double* b = beta  ? beta  : default_beta;

    // Semi-circles
    constexpr double SC = 1.0 / M_PI;  // radians to semi-circles

    double phi_u = lat * SC;    // user geodetic latitude (semi-circles)
    double lam_u = lon * SC;    // user geodetic longitude (semi-circles)

    // Earth-centred angle (semi-circles)
    double psi = 0.0137 / (el * SC / M_PI + 0.11) - 0.022;

    // Sub-ionospheric latitude (semi-circles)
    double phi_i = phi_u + psi * std::cos(az);
    if (phi_i >  0.416) phi_i =  0.416;
    if (phi_i < -0.416) phi_i = -0.416;

    // Sub-ionospheric longitude (semi-circles)
    double lam_i = lam_u + psi * std::sin(az) / std::cos(phi_i * M_PI);

    // Geomagnetic latitude (semi-circles)
    double phi_m = phi_i + 0.064 * std::cos((lam_i - 1.617) * M_PI);

    // Local time (seconds)
    double t = 43200.0 * lam_i + tow;
    t -= std::floor(t / 86400.0) * 86400.0;

    // Obliquity factor
    double F = 1.0 + 16.0 * std::pow(0.53 - el * SC / M_PI, 3.0);

    // Period and amplitude
    double PER = b[0] + b[1] * phi_m + b[2] * phi_m * phi_m + b[3] * phi_m * phi_m * phi_m;
    if (PER < 72000.0) PER = 72000.0;

    double AMP = a[0] + a[1] * phi_m + a[2] * phi_m * phi_m + a[3] * phi_m * phi_m * phi_m;
    if (AMP < 0.0) AMP = 0.0;

    // Phase
    double x = 2.0 * M_PI * (t - 50400.0) / PER;

    // Ionospheric delay (seconds)
    double iono_s;
    if (std::abs(x) < 1.57) {
        iono_s = F * (5e-9 + AMP * (1.0 - x * x / 2.0 + x * x * x * x / 24.0));
    } else {
        iono_s = F * 5e-9;
    }

    // Convert to meters
    return constants::SPEED_OF_LIGHT * iono_s;
}

/**
 * @brief Scale ionospheric delay from L1 to another frequency.
 *
 * Ionospheric delay is proportional to 1/f^2, so:
 *   delay_f = delay_L1 * (f_L1 / f)^2
 *
 * @param delay_l1      Ionospheric delay on L1 (meters)
 * @param frequency     Target signal frequency (Hz)
 * @return Ionospheric delay at the target frequency (meters)
 */
inline double ionoScaleFrequency(double delay_l1, double frequency) {
    double ratio = constants::GPS_L1_FREQ / frequency;
    return delay_l1 * ratio * ratio;
}

} // namespace models
} // namespace libgnss
