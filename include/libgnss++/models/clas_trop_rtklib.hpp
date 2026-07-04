#pragma once

/**
 * @file clas_trop_rtklib.hpp
 * @brief CLASLIB/RTKLIB troposphere helpers for grid trop parity.
 *
 * Ports get_stTv(), tropmapf() (NMF), and prectrop() composition from
 * CLASLIB rtkcmn.c / cssr2osr.c so gated OSR trop matches oracle exports.
 */

#include <cmath>

namespace libgnss {
namespace models {
namespace claslib {

constexpr double kCssrTropHsRefM = 2.3;
constexpr double kCssrTropWetRefM = 0.252;

inline double interpolateNiellLatitudeCoefficient(double abs_lat_deg,
                                                  const double coefficients[5]) {
    constexpr double latitude_grid_deg[5] = {15.0, 30.0, 45.0, 60.0, 75.0};
    if (abs_lat_deg <= latitude_grid_deg[0]) {
        return coefficients[0];
    }
    if (abs_lat_deg >= latitude_grid_deg[4]) {
        return coefficients[4];
    }
    for (int i = 0; i < 4; ++i) {
        if (abs_lat_deg <= latitude_grid_deg[i + 1]) {
            const double span = latitude_grid_deg[i + 1] - latitude_grid_deg[i];
            const double weight = (abs_lat_deg - latitude_grid_deg[i]) / span;
            return coefficients[i] * (1.0 - weight) + coefficients[i + 1] * weight;
        }
    }
    return coefficients[4];
}

inline double interpolateMopsLatitude(double lat_deg, const double table[5]) {
    constexpr double interval[5] = {15.0, 30.0, 45.0, 60.0, 75.0};
    if (lat_deg <= interval[0]) {
        return table[0];
    }
    if (lat_deg >= interval[4]) {
        return table[4];
    }
    for (int i = 0; i < 4; ++i) {
        if (lat_deg <= interval[i + 1]) {
            const double dx = (lat_deg - interval[i]) / (interval[i + 1] - interval[i]);
            return table[i] * (1.0 - dx) + table[i + 1] * dx;
        }
    }
    return table[4];
}

inline bool getMops(double lat_rad,
                    int day_of_year,
                    double& pressure_hpa,
                    double& temperature_k,
                    double& water_vapor_hpa,
                    double& beta,
                    double& lambda) {
    constexpr double kNorthMinDay = 28.0;
    const double lat_deg = lat_rad * 180.0 / M_PI;
    constexpr double p_mean[5] = {1013.25, 1017.25, 1015.75, 1011.75, 1013.00};
    constexpr double t_mean[5] = {299.65, 294.15, 283.15, 272.15, 263.65};
    constexpr double w_mean[5] = {26.31, 21.79, 11.66, 6.78, 4.11};
    constexpr double beta_mean[5] = {6.30e-3, 6.05e-3, 5.58e-3, 5.39e-3, 4.53e-3};
    constexpr double lambda_mean[5] = {2.77, 3.15, 2.57, 1.81, 1.55};
    constexpr double p_amp[5] = {0.00, -3.75, -2.25, -1.75, -0.50};
    constexpr double t_amp[5] = {0.00, 7.00, 11.00, 15.00, 14.50};
    constexpr double w_amp[5] = {0.00, 8.85, 7.24, 5.36, 3.39};
    constexpr double beta_amp[5] = {0.00e-3, 0.25e-3, 0.32e-3, 0.81e-3, 0.62e-3};
    constexpr double lambda_amp[5] = {0.00, 0.33, 0.46, 0.74, 0.30};

    double calc_p[5] = {};
    double calc_t[5] = {};
    double calc_w[5] = {};
    double calc_beta[5] = {};
    double calc_lambda[5] = {};
    const double seasonal = std::cos(2.0 * M_PI * (day_of_year - kNorthMinDay) / 365.25);
    for (int i = 0; i < 5; ++i) {
        calc_p[i] = p_mean[i] - p_amp[i] * seasonal;
        calc_t[i] = t_mean[i] - t_amp[i] * seasonal;
        calc_w[i] = w_mean[i] - w_amp[i] * seasonal;
        calc_beta[i] = beta_mean[i] - beta_amp[i] * seasonal;
        calc_lambda[i] = lambda_mean[i] - lambda_amp[i] * seasonal;
    }

    if (lat_deg < 15.0 || lat_deg > 75.0) {
        const int idx = lat_deg < 15.0 ? 0 : 4;
        pressure_hpa = calc_p[idx];
        temperature_k = calc_t[idx];
        water_vapor_hpa = calc_w[idx];
        beta = calc_beta[idx];
        lambda = calc_lambda[idx];
        return true;
    }

    pressure_hpa = interpolateMopsLatitude(lat_deg, calc_p);
    temperature_k = interpolateMopsLatitude(lat_deg, calc_t);
    water_vapor_hpa = interpolateMopsLatitude(lat_deg, calc_w);
    beta = interpolateMopsLatitude(lat_deg, calc_beta);
    lambda = interpolateMopsLatitude(lat_deg, calc_lambda);
    return true;
}

inline double dryZenithDelayMeters(double lat_rad, double height_m, double pressure_hpa) {
    return 0.0022768 * pressure_hpa /
           (1.0 - 0.00266 * std::cos(2.0 * lat_rad) - (2.8e-7) * height_m);
}

inline double wetZenithDelayMeters(double temperature_k, double water_vapor_hpa) {
    return 0.0022768 * (1255.0 / temperature_k + 0.05) * water_vapor_hpa;
}

inline bool getStTv(int day_of_year,
                    double lat_rad,
                    double ellipsoidal_height_m,
                    double geoid_height_m,
                    double& dry_zenith_m,
                    double& wet_zenith_m) {
    constexpr double kGravity = 9.80665;
    constexpr double kGasConstantDry = 287.0537625;

    double pressure0_hpa = 0.0;
    double temperature0_k = 0.0;
    double water_vapor0_hpa = 0.0;
    double beta = 0.0;
    double lambda = 0.0;
    if (!getMops(lat_rad, day_of_year, pressure0_hpa, temperature0_k,
                 water_vapor0_hpa, beta, lambda)) {
        return false;
    }

    const double orthometric_height_m = ellipsoidal_height_m - geoid_height_m;
    const double temperature_k = temperature0_k - beta * orthometric_height_m;
    const double pressure_hpa =
        pressure0_hpa *
        std::pow(1.0 - beta * orthometric_height_m / temperature0_k,
                 kGravity / (kGasConstantDry * beta));
    double water_vapor_hpa =
        water_vapor0_hpa *
        std::pow(1.0 - beta * orthometric_height_m / temperature0_k,
                 (lambda + 1.0) * kGravity / (kGasConstantDry * beta));

    constexpr double kT0 = 273.15;
    const double saturation_hpa =
        6.11 * std::pow(temperature_k / kT0, -5.3) *
        std::exp(25.2 * (temperature_k - kT0) / temperature_k);
    if (water_vapor_hpa > saturation_hpa) {
        water_vapor_hpa = saturation_hpa;
    }

    dry_zenith_m = dryZenithDelayMeters(lat_rad, ellipsoidal_height_m, pressure_hpa);
    wet_zenith_m = wetZenithDelayMeters(temperature_k, water_vapor_hpa);
    return true;
}

inline double nmfMappingContinuedFraction(double sin_elevation,
                                          double a,
                                          double b,
                                          double c) {
    const double numerator = 1.0 + a / (1.0 + b / (1.0 + c));
    const double denominator = sin_elevation + a / (sin_elevation + b / (sin_elevation + c));
    return numerator / denominator;
}

inline double tropMapDry(int day_of_year,
                         double lat_rad,
                         double height_m,
                         double elevation_rad,
                         double* wet_mapping_out) {
    constexpr double coef[9][5] = {
        {1.2769934E-3, 1.2683230E-3, 1.2465397E-3, 1.2196049E-3, 1.2045996E-3},
        {2.9153695E-3, 2.9152299E-3, 2.9288445E-3, 2.9022565E-3, 2.9024912E-3},
        {62.610505E-3, 62.837393E-3, 63.721774E-3, 63.824265E-3, 64.258455E-3},
        {0.0000000E-0, 1.2709626E-5, 2.6523662E-5, 3.4000452E-5, 4.1202191E-5},
        {0.0000000E-0, 2.1414979E-5, 3.0160779E-5, 7.2562722E-5, 11.723375E-5},
        {0.0000000E-0, 9.0128400E-5, 4.3497037E-5, 84.795348E-5, 170.37206E-5},
        {5.8021897E-4, 5.6794847E-4, 5.8118019E-4, 5.9727542E-4, 6.1641693E-4},
        {1.4275268E-3, 1.5138625E-3, 1.4572752E-3, 1.5007428E-3, 1.7599082E-3},
        {4.3472961E-2, 4.6729510E-2, 4.3908931E-2, 4.4626982E-2, 5.4736038E-2},
    };
    constexpr double height_coef[3] = {2.53E-5, 5.49E-3, 1.14E-3};

    if (elevation_rad <= 0.0) {
        if (wet_mapping_out) {
            *wet_mapping_out = 0.0;
        }
        return 0.0;
    }

    const double lat_deg = std::fabs(lat_rad * 180.0 / M_PI);
    const double seasonal_year =
        (static_cast<double>(day_of_year) - 28.0) / 365.25 + (lat_rad < 0.0 ? 0.5 : 0.0);
    const double seasonal = std::cos(2.0 * M_PI * seasonal_year);

    double hydro_a[3] = {};
    double wet_a[3] = {};
    for (int i = 0; i < 3; ++i) {
        hydro_a[i] = interpolateNiellLatitudeCoefficient(
                           lat_deg, coef[i]) -
                       interpolateNiellLatitudeCoefficient(lat_deg, coef[i + 3]) * seasonal;
        wet_a[i] = interpolateNiellLatitudeCoefficient(lat_deg, coef[i + 6]);
    }

    const double sin_elevation = std::sin(elevation_rad);
    const double height_correction =
        (1.0 / sin_elevation -
         nmfMappingContinuedFraction(
             sin_elevation, height_coef[0], height_coef[1], height_coef[2])) *
        height_m * 1e-3;
    if (wet_mapping_out) {
        *wet_mapping_out =
            nmfMappingContinuedFraction(sin_elevation, wet_a[0], wet_a[1], wet_a[2]);
    }
    return nmfMappingContinuedFraction(sin_elevation, hydro_a[0], hydro_a[1], hydro_a[2]) +
           height_correction;
}

inline double preciseTropMeters(int day_of_year,
                                double lat_rad,
                                double lon_rad,
                                double height_m,
                                double geoid_height_m,
                                double elevation_rad,
                                double interpolated_ztd,
                                double interpolated_zwd) {
    double dry_zenith_m = 0.0;
    double wet_zenith_m = 0.0;
    if (!getStTv(day_of_year, lat_rad, height_m, geoid_height_m,
                 dry_zenith_m, wet_zenith_m)) {
        return 0.0;
    }
    double wet_mapping = 0.0;
    const double dry_mapping =
        tropMapDry(day_of_year, lat_rad, height_m, elevation_rad, &wet_mapping);
    (void)lon_rad;
    return dry_mapping * dry_zenith_m * interpolated_ztd +
           wet_mapping * wet_zenith_m * interpolated_zwd;
}

inline double tropTotalFromResiduals(double hs_residual_m, double wet_residual_m) {
    return hs_residual_m + wet_residual_m + kCssrTropWetRefM + kCssrTropHsRefM;
}

inline double tropWetFromResidual(double wet_residual_m) {
    return wet_residual_m + kCssrTropWetRefM;
}

}  // namespace claslib
}  // namespace models
}  // namespace libgnss
