#pragma once

/**
 * @file troposphere.hpp
 * @brief Tropospheric delay models for GNSS processing.
 *
 * Extracted from RTKProcessor::tropModel() (rtk.cpp).
 * Provides the Saastamoinen model with simple and Niell mapping helpers.
 */

#include <libgnss++/core/coordinates.hpp>
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <algorithm>

namespace libgnss {
namespace models {

/**
 * @brief Saastamoinen troposphere model with 1/cos(z) mapping.
 *
 * Computes the total (hydrostatic + wet) tropospheric delay using the
 * Saastamoinen model with standard atmosphere assumptions.  The mapping
 * function is a simple 1/cos(z) which is adequate for short-baseline RTK
 * where double-differencing removes most of the tropospheric error.
 *
 * For short baselines the DD residual from NMF is < 1 mm and < 5 mm
 * at 3.3 km.
 *
 * @param pos_ecef  Receiver ECEF position (m)
 * @param elevation Satellite elevation angle (radians)
 * @return Tropospheric delay in meters (zero if elevation is too low)
 */
inline double tropDelaySaastamoinen(const Eigen::Vector3d& pos_ecef, double elevation) {
    if (elevation <= 0.05) return 0.0;

    double lat, lon, h;
    ecef2geodetic(pos_ecef, lat, lon, h);

    if (h < -100.0 || h > 10000.0) return 0.0;

    double hgt = std::max(h, 0.0);

    // Standard atmosphere
    double T = 15.0 - 6.5e-3 * hgt + 273.16;                          // Temperature (K)
    double P = 1013.25 * std::pow(1.0 - 2.2557e-5 * hgt, 5.2568);     // Pressure (mbar)
    double e = 6.108 * 0.7 * std::exp((17.15 * T - 4684.0) / (T - 38.45)); // Water vapour (mbar)

    // Zenith angle and 1/cos(z) mapping
    double z = M_PI / 2.0 - elevation;
    double cos_z = std::cos(z);
    if (cos_z < 0.067) cos_z = 0.067;

    // Hydrostatic (dry) component
    double trph = 0.0022768 * P /
        (1.0 - 0.00266 * std::cos(2.0 * lat) - 0.00028 * hgt * 1e-3) / cos_z;

    // Wet component
    double trpw = 0.002277 * (1255.0 / T + 0.05) * e / cos_z;

    return trph + trpw;
}

inline double niellMappingContinuedFraction(double sin_elevation,
                                            double a,
                                            double b,
                                            double c) {
    const double numerator = 1.0 + a / (1.0 + b / (1.0 + c));
    const double denominator = sin_elevation + a / (sin_elevation + b / (sin_elevation + c));
    return numerator / denominator;
}

inline double interpolateNiellCoefficient(double abs_lat_deg,
                                          const std::array<double, 5>& coefficients) {
    constexpr std::array<double, 5> latitude_grid_deg = {15.0, 30.0, 45.0, 60.0, 75.0};
    if (abs_lat_deg <= latitude_grid_deg.front()) {
        return coefficients.front();
    }
    if (abs_lat_deg >= latitude_grid_deg.back()) {
        return coefficients.back();
    }
    for (size_t i = 0; i + 1 < latitude_grid_deg.size(); ++i) {
        if (abs_lat_deg <= latitude_grid_deg[i + 1]) {
            const double span = latitude_grid_deg[i + 1] - latitude_grid_deg[i];
            const double weight = (abs_lat_deg - latitude_grid_deg[i]) / span;
            return coefficients[i] * (1.0 - weight) + coefficients[i + 1] * weight;
        }
    }
    return coefficients.back();
}

/**
 * @brief Niell hydrostatic mapping function.
 *
 * This follows the latitude/season dependent hydrostatic mapping function from
 * Niell (1996), including the standard height correction term.
 *
 * @param latitude_rad  Geodetic latitude in radians
 * @param height_m      Orthometric/ellipsoidal height in meters
 * @param elevation     Satellite elevation angle in radians
 * @param day_of_year   Day of year in [1, 366]
 * @return Mapping factor (>= 1.0)
 */
inline double niellHydrostaticMapping(double latitude_rad,
                                      double height_m,
                                      double elevation,
                                      int day_of_year) {
    constexpr std::array<double, 5> mean_a = {
        1.2769934e-3, 1.2683230e-3, 1.2465397e-3, 1.2196049e-3, 1.2045996e-3};
    constexpr std::array<double, 5> mean_b = {
        2.9153695e-3, 2.9152299e-3, 2.9288445e-3, 2.9022565e-3, 2.9024912e-3};
    constexpr std::array<double, 5> mean_c = {
        62.610505e-3, 62.837393e-3, 63.721774e-3, 63.824265e-3, 64.258455e-3};
    constexpr std::array<double, 5> amp_a = {
        0.0, 1.2709626e-5, 2.6523662e-5, 3.4000452e-5, 4.1202191e-5};
    constexpr std::array<double, 5> amp_b = {
        0.0, 2.1414979e-5, 3.0160779e-5, 7.2562722e-5, 11.723375e-5};
    constexpr std::array<double, 5> amp_c = {
        0.0, 9.0128400e-5, 4.3497037e-5, 84.795348e-5, 170.37206e-5};
    constexpr double height_a = 2.53e-5;
    constexpr double height_b = 5.49e-3;
    constexpr double height_c = 1.14e-3;

    const double sin_elevation = std::max(std::sin(elevation), 0.05);
    const double abs_lat_deg = std::clamp(std::abs(latitude_rad) * 180.0 / M_PI, 0.0, 90.0);
    const double seasonal_phase =
        std::cos(2.0 * M_PI * (static_cast<double>(day_of_year) - 28.0) / 365.25 +
                 (latitude_rad < 0.0 ? M_PI : 0.0));

    const double a = interpolateNiellCoefficient(abs_lat_deg, mean_a) -
        interpolateNiellCoefficient(abs_lat_deg, amp_a) * seasonal_phase;
    const double b = interpolateNiellCoefficient(abs_lat_deg, mean_b) -
        interpolateNiellCoefficient(abs_lat_deg, amp_b) * seasonal_phase;
    const double c = interpolateNiellCoefficient(abs_lat_deg, mean_c) -
        interpolateNiellCoefficient(abs_lat_deg, amp_c) * seasonal_phase;

    double mapping = niellMappingContinuedFraction(sin_elevation, a, b, c);
    const double height_km = std::max(height_m, 0.0) * 1e-3;
    if (height_km > 0.0) {
        mapping += (1.0 / sin_elevation -
                    niellMappingContinuedFraction(sin_elevation, height_a, height_b, height_c)) *
            height_km;
    }
    return mapping;
}

/**
 * @brief Niell wet mapping function.
 *
 * @param latitude_rad Geodetic latitude in radians
 * @param elevation    Satellite elevation angle in radians
 * @return Wet mapping factor (>= 1.0)
 */
inline double niellWetMapping(double latitude_rad, double elevation) {
    constexpr std::array<double, 5> wet_a = {
        5.8021897e-4, 5.6794847e-4, 5.8118019e-4, 5.9727542e-4, 6.1641693e-4};
    constexpr std::array<double, 5> wet_b = {
        1.4275268e-3, 1.5138625e-3, 1.4572752e-3, 1.5007428e-3, 1.7599082e-3};
    constexpr std::array<double, 5> wet_c = {
        4.3472961e-2, 4.6729510e-2, 4.3908931e-2, 4.4626982e-2, 5.4736038e-2};

    const double sin_elevation = std::max(std::sin(elevation), 0.05);
    const double abs_lat_deg = std::clamp(std::abs(latitude_rad) * 180.0 / M_PI, 0.0, 90.0);
    const double a = interpolateNiellCoefficient(abs_lat_deg, wet_a);
    const double b = interpolateNiellCoefficient(abs_lat_deg, wet_b);
    const double c = interpolateNiellCoefficient(abs_lat_deg, wet_c);
    return niellMappingContinuedFraction(sin_elevation, a, b, c);
}

struct ZenithTroposphereDelay {
    double pressure_hpa = 0.0;
    double temperature_k = 0.0;
    double water_vapor_pressure_hpa = 0.0;
    double hydrostatic_delay_m = 0.0;
    double wet_delay_m = 0.0;

    double totalDelayMeters() const {
        return hydrostatic_delay_m + wet_delay_m;
    }
};

inline double interpolateSeasonalClimatology(double abs_lat_deg,
                                             const std::array<double, 5>& mean_values,
                                             const std::array<double, 5>& amplitude_values,
                                             double seasonal_phase) {
    return interpolateNiellCoefficient(abs_lat_deg, mean_values) -
           interpolateNiellCoefficient(abs_lat_deg, amplitude_values) * seasonal_phase;
}

/**
 * @brief Lightweight GPT-style zenith troposphere climatology.
 *
 * Uses latitude/season/height dependent surface pressure, temperature, water
 * vapour pressure, temperature lapse rate, and water-vapour scale factor to
 * derive zenith hydrostatic and wet delay estimates. The coefficient set is
 * adapted from the UNB3m climatology and used here as a compact in-tree
 * fallback for PPP zenith-delay seeding.
 *
 * @param latitude_rad Geodetic latitude in radians
 * @param height_m     Receiver height in meters
 * @param day_of_year  Day of year in [1, 366]
 * @return Zenith hydrostatic/wet delay estimate
 */
inline ZenithTroposphereDelay estimateZenithTroposphereClimatology(double latitude_rad,
                                                                   double height_m,
                                                                   int day_of_year) {
    constexpr std::array<double, 5> mean_pressure_hpa = {
        1013.25, 1017.25, 1015.75, 1011.75, 1013.00};
    constexpr std::array<double, 5> amp_pressure_hpa = {
        0.0, -3.75, -2.25, -1.75, -0.50};
    constexpr std::array<double, 5> mean_temperature_k = {
        299.65, 294.15, 283.15, 272.15, 263.65};
    constexpr std::array<double, 5> amp_temperature_k = {
        0.0, 7.00, 11.00, 15.00, 14.50};
    constexpr std::array<double, 5> mean_water_vapor_hpa = {
        26.31, 21.79, 11.66, 6.78, 4.11};
    constexpr std::array<double, 5> amp_water_vapor_hpa = {
        0.0, 8.85, 7.24, 5.36, 3.39};
    constexpr std::array<double, 5> mean_beta = {
        6.30e-3, 6.05e-3, 5.58e-3, 5.39e-3, 4.53e-3};
    constexpr std::array<double, 5> amp_beta = {
        0.0, 0.25e-3, 0.32e-3, 0.81e-3, 0.62e-3};
    constexpr std::array<double, 5> mean_lambda = {
        2.77, 3.15, 2.57, 1.81, 1.55};
    constexpr std::array<double, 5> amp_lambda = {
        0.0, 0.33, 0.46, 0.74, 0.30};
    constexpr double gravity = 9.80665;
    constexpr double gas_constant_dry = 287.054;

    ZenithTroposphereDelay delay;
    const double abs_lat_deg = std::clamp(std::abs(latitude_rad) * 180.0 / M_PI, 0.0, 90.0);
    const double seasonal_phase =
        std::cos(2.0 * M_PI * (static_cast<double>(day_of_year) - 28.0) / 365.25 +
                 (latitude_rad < 0.0 ? M_PI : 0.0));

    const double pressure0_hpa = interpolateSeasonalClimatology(
        abs_lat_deg, mean_pressure_hpa, amp_pressure_hpa, seasonal_phase);
    const double temperature0_k = interpolateSeasonalClimatology(
        abs_lat_deg, mean_temperature_k, amp_temperature_k, seasonal_phase);
    const double water_vapor0_hpa = interpolateSeasonalClimatology(
        abs_lat_deg, mean_water_vapor_hpa, amp_water_vapor_hpa, seasonal_phase);
    const double beta = interpolateSeasonalClimatology(
        abs_lat_deg, mean_beta, amp_beta, seasonal_phase);
    const double lambda = interpolateSeasonalClimatology(
        abs_lat_deg, mean_lambda, amp_lambda, seasonal_phase);

    const double clamped_height_m = std::max(height_m, 0.0);
    const double temperature_scale = std::max(1.0 - beta * clamped_height_m / temperature0_k, 1e-3);
    const double exponent = gravity / (gas_constant_dry * beta);

    delay.pressure_hpa = pressure0_hpa * std::pow(temperature_scale, exponent);
    delay.temperature_k = temperature0_k - beta * clamped_height_m;
    delay.water_vapor_pressure_hpa =
        water_vapor0_hpa *
        std::pow(temperature_scale, exponent * (lambda + 1.0) - 1.0);

    delay.hydrostatic_delay_m =
        0.0022768 * delay.pressure_hpa /
        (1.0 - 0.00266 * std::cos(2.0 * latitude_rad) - 0.00028 * clamped_height_m * 1e-3);
    delay.wet_delay_m =
        0.002277 * (1255.0 / delay.temperature_k + 0.05) * delay.water_vapor_pressure_hpa;
    return delay;
}

/**
 * @brief GPT-style modeled slant troposphere delay.
 *
 * Uses the latitude/season/height dependent zenith hydrostatic and wet delay
 * climatology and maps each component with the corresponding Niell mapping
 * function.
 *
 * @param latitude_rad Geodetic latitude in radians
 * @param height_m     Receiver height in meters
 * @param elevation    Satellite elevation angle in radians
 * @param day_of_year  Day of year in [1, 366]
 * @return Slant hydrostatic + wet delay estimate in meters
 */
inline double modeledTroposphereDelayClimatology(double latitude_rad,
                                                 double height_m,
                                                 double elevation,
                                                 int day_of_year) {
    const auto zenith =
        estimateZenithTroposphereClimatology(latitude_rad, height_m, day_of_year);
    const double hydro_mapping =
        niellHydrostaticMapping(latitude_rad, height_m, elevation, day_of_year);
    const double wet_mapping = niellWetMapping(latitude_rad, elevation);
    return zenith.hydrostatic_delay_m * hydro_mapping + zenith.wet_delay_m * wet_mapping;
}

} // namespace models
} // namespace libgnss
