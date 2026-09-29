#pragma once

// Shared pieces of the observable-level pseudorange/Doppler/carrier batch
// solvers (gnss_pos_pd, gnss_pos_pdc, gnss_pos_vel_pd, gnss_pos_vel_pdc,
// gnss_vel_d). Only helpers that do not depend on the per-app state layout
// live here: anything indexing the state vector (kStateStride differs
// between the position-only and position/velocity graphs) stays in the app.

#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/models/ionosphere.hpp>
#include <libgnss++/models/troposphere.hpp>

#include <Eigen/Sparse>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <stdexcept>
#include <string>
#include <vector>

#include "observable_measurement_helpers.hpp"

namespace libgnss_apps {

namespace pd_detail {

inline constexpr double kPi = 3.141592653589793238462643383279502884;
inline constexpr double kDegreesToRadians = kPi / 180.0;

}  // namespace pd_detail

struct PseudorangeFactor {
    std::size_t epoch_index = 0;
    libgnss::GNSSTime time;
    libgnss::SatelliteId satellite;
    libgnss::SignalType signal = libgnss::SignalType::GPS_L1CA;
    std::size_t clock_group = 0;
    double snr_dbhz = 0.0;
    double elevation_rad = 0.0;
    double sigma_m = 1.0;
    double residual_m = 0.0;
    double corrected_pseudorange_m = 0.0;
    double modeled_range_m = 0.0;
    double ionosphere_delay_m = 0.0;
    double troposphere_delay_m = 0.0;
    double satellite_clock_m = 0.0;
    double group_delay_m = 0.0;
    libgnss::Vector3d los = libgnss::Vector3d::Zero();
};

struct CarrierResidual {
    std::size_t epoch_index = 0;
    libgnss::GNSSTime time;
    libgnss::SatelliteId satellite;
    libgnss::SignalType signal = libgnss::SignalType::GPS_L1CA;
    double snr_dbhz = 0.0;
    double elevation_rad = 0.0;
    double residual_m = 0.0;
    double corrected_carrier_m = 0.0;
    double modeled_range_m = 0.0;
    double raw_carrier_cycles = 0.0;
    double wavelength_m = 0.0;
    double ionosphere_delay_m = 0.0;
    double troposphere_delay_m = 0.0;
    double satellite_clock_m = 0.0;
    std::uint8_t lli = 0;
    bool loss_of_lock = false;
    libgnss::Vector3d los = libgnss::Vector3d::Zero();
};

struct TdcpFactor {
    std::size_t epoch_index = 0;
    std::size_t previous_epoch_index = 0;
    libgnss::GNSSTime time;
    libgnss::SatelliteId satellite;
    libgnss::SignalType signal = libgnss::SignalType::GPS_L1CA;
    double snr_dbhz = 0.0;
    double elevation_rad = 0.0;
    double previous_elevation_rad = 0.0;
    double sigma_m = 1.0;
    double residual_m = 0.0;
    double previous_carrier_residual_m = 0.0;
    double current_carrier_residual_m = 0.0;
    double previous_raw_carrier_cycles = 0.0;
    double current_raw_carrier_cycles = 0.0;
    double wavelength_m = 0.0;
    libgnss::Vector3d los = libgnss::Vector3d::Zero();
};

struct SolveResult {
    Eigen::VectorXd state;
    double initial_cost = 0.0;
    double final_cost = 0.0;
    double residual_rms_mps = 0.0;
    int iterations = 0;
    bool converged = false;
};

struct NormalEquation {
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd rhs;
};

template <typename UsageError>
int parseIntArg(const std::string& value,
                const std::string& name,
                const char* argv0,
                UsageError usage_error) {
    try {
        std::size_t consumed = 0;
        const int parsed = std::stoi(value, &consumed);
        if (consumed == value.size()) {
            return parsed;
        }
    } catch (const std::exception&) {
    }
    usage_error("invalid integer for " + name + ": " + value, argv0);
    throw std::logic_error("usage error handler returned");
}

template <typename UsageError>
double parseDoubleArg(const std::string& value,
                      const std::string& name,
                      const char* argv0,
                      UsageError usage_error) {
    try {
        std::size_t consumed = 0;
        const double parsed = std::stod(value, &consumed);
        if (consumed == value.size() && std::isfinite(parsed)) {
            return parsed;
        }
    } catch (const std::exception&) {
    }
    usage_error("invalid number for " + name + ": " + value, argv0);
    throw std::logic_error("usage error handler returned");
}

inline std::string jsonBool(bool value) {
    return value ? "true" : "false";
}

inline void addWeightedRow(std::vector<Eigen::Triplet<double>>& triplets,
                           Eigen::VectorXd& rhs,
                           const std::vector<int>& columns,
                           const std::vector<double>& coefficients,
                           double residual,
                           double sigma,
                           double robust_weight) {
    const double inv_variance = robust_weight / (sigma * sigma);
    for (std::size_t a = 0; a < columns.size(); ++a) {
        const double weighted_a = inv_variance * coefficients[a];
        rhs(columns[a]) += weighted_a * residual;
        for (std::size_t b = 0; b < columns.size(); ++b) {
            triplets.emplace_back(columns[a],
                                  columns[b],
                                  weighted_a * coefficients[b]);
        }
    }
}

template <typename Options>
bool calculateObservationModel(const libgnss::ObservationData& epoch,
                               const libgnss::Observation& observation,
                               const libgnss::NavigationData& nav,
                               const libgnss::Vector3d& receiver_position,
                               const Options& options,
                               libgnss::Vector3d& satellite_position,
                               libgnss::Vector3d& satellite_velocity,
                               double& satellite_clock_bias,
                               double& satellite_clock_drift,
                               const libgnss::Ephemeris*& eph,
                               libgnss::NavigationData::SatelliteGeometry& geometry,
                               libgnss::Vector3d& ex,
                               double& range_m) {
    if (!isPrimaryPdSignal(observation.signal) ||
        !observation.valid ||
        !observation.has_pseudorange ||
        observation.pseudorange <= 0.0 ||
        observation.snr < options.min_snr_dbhz) {
        return false;
    }

    libgnss::GNSSTime transmit_time =
        epoch.time - observation.pseudorange / libgnss::constants::SPEED_OF_LIGHT;
    if (!nav.calculateSatelliteState(observation.satellite,
                                     transmit_time,
                                     satellite_position,
                                     satellite_velocity,
                                     satellite_clock_bias,
                                     satellite_clock_drift)) {
        return false;
    }

    transmit_time = transmit_time - satellite_clock_bias;
    if (!nav.calculateSatelliteState(observation.satellite,
                                     transmit_time,
                                     satellite_position,
                                     satellite_velocity,
                                     satellite_clock_bias,
                                     satellite_clock_drift)) {
        return false;
    }

    eph = nav.getEphemeris(observation.satellite, transmit_time);
    if (!eph || !isHealthyForPositioning(observation, *eph)) {
        return false;
    }

    const libgnss::Vector3d delta = satellite_position - receiver_position;
    range_m = delta.norm();
    if (range_m <= 0.0) {
        return false;
    }
    ex = delta / range_m;
    geometry = nav.calculateGeometry(receiver_position, satellite_position);
    return geometry.elevation >=
           options.min_elevation_deg * pd_detail::kDegreesToRadians;
}

template <typename Options>
bool preparePseudorangeFactor(const libgnss::ObservationData& epoch,
                              std::size_t epoch_index,
                              const libgnss::Observation& observation,
                              const libgnss::NavigationData& nav,
                              const libgnss::Vector3d& receiver_position,
                              const Options& options,
                              PseudorangeFactor& factor) {
    libgnss::Vector3d satellite_position;
    libgnss::Vector3d satellite_velocity;
    double satellite_clock_bias = 0.0;
    double satellite_clock_drift = 0.0;
    const libgnss::Ephemeris* eph = nullptr;
    libgnss::NavigationData::SatelliteGeometry geometry;
    libgnss::Vector3d ex = libgnss::Vector3d::Zero();
    double geometric_range = 0.0;
    if (!libgnss_apps::calculateObservationModel(epoch,
                                                 observation,
                                                 nav,
                                                 receiver_position,
                                                 options,
                                                 satellite_position,
                                                 satellite_velocity,
                                                 satellite_clock_bias,
                                                 satellite_clock_drift,
                                                 eph,
                                                 geometry,
                                                 ex,
                                                 geometric_range)) {
        return false;
    }

    double receiver_lat = 0.0;
    double receiver_lon = 0.0;
    double receiver_height = 0.0;
    libgnss::ecef2geodetic(receiver_position,
                           receiver_lat,
                           receiver_lon,
                           receiver_height);
    double ionosphere_delay = 0.0;
    if (nav.ionosphere_model.valid) {
        ionosphere_delay = libgnss::models::ionoDelayKlobuchar(
            receiver_lat,
            receiver_lon,
            geometry.azimuth,
            geometry.elevation,
            epoch.time.tow,
            nav.ionosphere_model.alpha,
            nav.ionosphere_model.beta);
        const double frequency_hz =
            libgnss::signalFrequencyHz(observation.signal, eph);
        if (frequency_hz > 0.0) {
            const double scale = libgnss::constants::GPS_L1_FREQ / frequency_hz;
            ionosphere_delay *= scale * scale;
        }
    }
    const double troposphere_delay =
        libgnss::models::tropDelaySaastamoinen(receiver_position,
                                               geometry.elevation);
    const double satellite_clock_m =
        satellite_clock_bias * libgnss::constants::SPEED_OF_LIGHT;
    const double group_delay_m = groupDelayCorrectionMeters(observation, *eph);
    const double modeled_range =
        geometric_range + sagnacRangeCorrection(satellite_position, receiver_position);
    const double corrected_pseudorange =
        observation.pseudorange +
        satellite_clock_m -
        ionosphere_delay -
        troposphere_delay -
        group_delay_m;
    const double residual = corrected_pseudorange - modeled_range;
    const double sin_el = std::sin(geometry.elevation);
    if (sin_el <= 0.0) {
        return false;
    }

    factor.epoch_index = epoch_index;
    factor.time = epoch.time;
    factor.satellite = observation.satellite;
    factor.signal = observation.signal;
    factor.clock_group = clockGroup(observation.satellite.system);
    factor.snr_dbhz = observation.snr;
    factor.elevation_rad = geometry.elevation;
    factor.sigma_m = options.pseudorange_sigma_zenith_m / std::sqrt(sin_el);
    factor.residual_m = residual;
    factor.corrected_pseudorange_m = corrected_pseudorange;
    factor.modeled_range_m = modeled_range;
    factor.ionosphere_delay_m = ionosphere_delay;
    factor.troposphere_delay_m = troposphere_delay;
    factor.satellite_clock_m = satellite_clock_m;
    factor.group_delay_m = group_delay_m;
    factor.los = -ex;
    return std::isfinite(factor.residual_m) &&
           std::isfinite(factor.sigma_m) &&
           factor.sigma_m > 0.0;
}

template <typename Options, typename DopplerFactor>
bool prepareDopplerFactor(const libgnss::ObservationData& epoch,
                          std::size_t epoch_index,
                          std::size_t previous_epoch_index,
                          const libgnss::Observation& observation,
                          const libgnss::NavigationData& nav,
                          const libgnss::Vector3d& receiver_position,
                          const libgnss::Vector3d& receiver_velocity,
                          double sigma_elevation_rad,
                          double dt_s,
                          const Options& options,
                          DopplerFactor& factor) {
    if (!isPrimaryPdSignal(observation.signal) ||
        !observation.valid ||
        !observation.has_pseudorange ||
        !observation.has_doppler ||
        observation.pseudorange <= 0.0) {
        return false;
    }

    libgnss::Vector3d satellite_position;
    libgnss::Vector3d satellite_velocity;
    double satellite_clock_bias = 0.0;
    double satellite_clock_drift = 0.0;
    const libgnss::Ephemeris* eph = nullptr;
    libgnss::NavigationData::SatelliteGeometry geometry;
    libgnss::Vector3d ex = libgnss::Vector3d::Zero();
    double range = 0.0;
    libgnss::GNSSTime transmit_time =
        epoch.time - observation.pseudorange / libgnss::constants::SPEED_OF_LIGHT;
    if (!nav.calculateSatelliteState(observation.satellite,
                                     transmit_time,
                                     satellite_position,
                                     satellite_velocity,
                                     satellite_clock_bias,
                                     satellite_clock_drift)) {
        return false;
    }
    transmit_time = transmit_time - satellite_clock_bias;
    if (!nav.calculateSatelliteState(observation.satellite,
                                     transmit_time,
                                     satellite_position,
                                     satellite_velocity,
                                     satellite_clock_bias,
                                     satellite_clock_drift)) {
        return false;
    }
    eph = nav.getEphemeris(observation.satellite, transmit_time);
    if (!eph || !isHealthyForPositioning(observation, *eph)) {
        return false;
    }

    const libgnss::Vector3d delta = satellite_position - receiver_position;
    range = delta.norm();
    if (range <= 0.0 || dt_s <= 0.0) {
        return false;
    }
    ex = delta / range;
    geometry = nav.calculateGeometry(receiver_position, satellite_position);

    double wavelength = libgnss::signalWavelengthMeters(observation.signal, eph);
    if (wavelength <= 0.0) {
        wavelength = libgnss::signalWavelengthMeters(observation);
    }
    if (wavelength <= 0.0) {
        return false;
    }

    const double sagnac_rate =
        libgnss::constants::OMEGA_E / libgnss::constants::SPEED_OF_LIGHT *
        (satellite_velocity(1) * receiver_position(0) -
         satellite_velocity(0) * receiver_position(1));
    const double modeled_range_rate =
        (satellite_velocity - receiver_velocity).dot(ex) + sagnac_rate;
    const double satellite_clock_drift_mps =
        satellite_clock_drift * libgnss::constants::SPEED_OF_LIGHT;
    const double measured_range_rate = -observation.doppler * wavelength;
    const double residual =
        measured_range_rate - (modeled_range_rate - satellite_clock_drift_mps);
    const double sin_el = std::sin(sigma_elevation_rad);
    if (sin_el <= 0.0) {
        return false;
    }

    factor.epoch_index = epoch_index;
    factor.previous_epoch_index = previous_epoch_index;
    factor.time = epoch.time;
    factor.midpoint_time = epoch.time;
    factor.satellite = observation.satellite;
    factor.signal = observation.signal;
    factor.snr_dbhz = observation.snr;
    factor.elevation_rad = sigma_elevation_rad;
    factor.midpoint_elevation_rad = geometry.elevation;
    factor.dt_s = dt_s;
    factor.sigma_mps = options.doppler_sigma_zenith_mps / std::sqrt(sin_el);
    factor.residual_mps = residual;
    factor.measured_range_rate_mps = measured_range_rate;
    factor.modeled_range_rate_mps = modeled_range_rate;
    factor.satellite_clock_drift_mps = satellite_clock_drift_mps;
    factor.wavelength_m = wavelength;
    factor.los = -ex;
    return std::isfinite(factor.residual_mps) &&
           std::isfinite(factor.sigma_mps) &&
           factor.sigma_mps > 0.0;
}

inline libgnss::Observation interpolateObservation(const libgnss::Observation& lower,
                                                   const libgnss::Observation& upper,
                                                   double alpha) {
    libgnss::Observation interpolated = lower;
    interpolated.valid = lower.valid && upper.valid;
    interpolated.has_pseudorange =
        lower.has_pseudorange && upper.has_pseudorange;
    interpolated.has_carrier_phase =
        lower.has_carrier_phase && upper.has_carrier_phase;
    interpolated.has_doppler = lower.has_doppler && upper.has_doppler;
    if (interpolated.has_pseudorange) {
        interpolated.pseudorange =
            lower.pseudorange + alpha * (upper.pseudorange - lower.pseudorange);
    }
    if (interpolated.has_carrier_phase) {
        interpolated.carrier_phase =
            lower.carrier_phase +
            alpha * (upper.carrier_phase - lower.carrier_phase);
    }
    if (interpolated.has_doppler) {
        interpolated.doppler =
            lower.doppler + alpha * (upper.doppler - lower.doppler);
    }
    interpolated.snr = std::min(lower.snr, upper.snr);
    interpolated.signal_strength =
        std::min(lower.signal_strength, upper.signal_strength);
    interpolated.lli = lower.lli | upper.lli;
    interpolated.loss_of_lock =
        lower.loss_of_lock || upper.loss_of_lock ||
        ((interpolated.lli & 0x01U) != 0);
    interpolated.has_glonass_frequency_channel =
        lower.has_glonass_frequency_channel &&
        upper.has_glonass_frequency_channel &&
        lower.glonass_frequency_channel == upper.glonass_frequency_channel;
    if (interpolated.has_glonass_frequency_channel) {
        interpolated.glonass_frequency_channel = lower.glonass_frequency_channel;
    }
    return interpolated;
}

inline bool observationElevationRad(const libgnss::ObservationData& epoch,
                                    const libgnss::Observation& observation,
                                    const libgnss::NavigationData& nav,
                                    const libgnss::Vector3d& receiver_position,
                                    double& elevation_rad) {
    if (!isPrimaryPdSignal(observation.signal) ||
        !observation.valid ||
        !observation.has_pseudorange ||
        observation.pseudorange <= 0.0) {
        return false;
    }

    libgnss::Vector3d satellite_position;
    libgnss::Vector3d satellite_velocity;
    double satellite_clock_bias = 0.0;
    double satellite_clock_drift = 0.0;
    libgnss::GNSSTime transmit_time =
        epoch.time - observation.pseudorange / libgnss::constants::SPEED_OF_LIGHT;
    if (!nav.calculateSatelliteState(observation.satellite,
                                     transmit_time,
                                     satellite_position,
                                     satellite_velocity,
                                     satellite_clock_bias,
                                     satellite_clock_drift)) {
        return false;
    }
    transmit_time = transmit_time - satellite_clock_bias;
    if (!nav.calculateSatelliteState(observation.satellite,
                                     transmit_time,
                                     satellite_position,
                                     satellite_velocity,
                                     satellite_clock_bias,
                                     satellite_clock_drift)) {
        return false;
    }
    const libgnss::Ephemeris* eph =
        nav.getEphemeris(observation.satellite, transmit_time);
    if (!eph || !isHealthyForPositioning(observation, *eph)) {
        return false;
    }
    elevation_rad =
        nav.calculateGeometry(receiver_position, satellite_position).elevation;
    return std::isfinite(elevation_rad);
}

}  // namespace libgnss_apps
