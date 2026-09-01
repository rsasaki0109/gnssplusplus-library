#include <libgnss++/algorithms/base_pseudorange_compensation.hpp>

#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/algorithms/galileo_group_delay.hpp>
#include <libgnss++/models/ionosphere.hpp>
#include <libgnss++/models/troposphere.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace libgnss::base_pseudorange_compensation {
namespace {

bool finiteTime(const GNSSTime& time) {
    return std::isfinite(time.tow) && time.week >= -10000 && time.week <= 100000;
}

Vector3d earthRotationCorrected(const Vector3d& satellite_position,
                                const Vector3d& receiver_position) {
    const double travel_time =
        (satellite_position - receiver_position).norm() /
        constants::SPEED_OF_LIGHT;
    const double angle = constants::OMEGA_E * travel_time;
    Eigen::Matrix3d rotation;
    rotation << std::cos(angle), std::sin(angle), 0.0,
                -std::sin(angle), std::cos(angle), 0.0,
                0.0, 0.0, 1.0;
    return rotation * satellite_position;
}

bool isHealthyForPositioning(const Observation& observation,
                             const Ephemeris& ephemeris) {
    int health = static_cast<int>(ephemeris.health);
    if (observation.satellite.system == GNSSSystem::QZSS) {
        health &= 0xFE;
    }
    return health == 0;
}

double groupDelayCorrectionMeters(const Observation& observation,
                                  const Ephemeris& ephemeris,
                                  bool use_signal_specific_e1) {
    if (observation.satellite.system == GNSSSystem::Galileo) {
        return galileo_group_delay::correctionMeters(
            observation, ephemeris, use_signal_specific_e1);
    }
    switch (observation.satellite.system) {
        case GNSSSystem::GPS:
        case GNSSSystem::QZSS:
            return ephemeris.tgd * constants::SPEED_OF_LIGHT;
        case GNSSSystem::BeiDou:
            switch (observation.signal) {
                case SignalType::BDS_B1I:
                case SignalType::BDS_B1C:
                    return ephemeris.tgd * constants::SPEED_OF_LIGHT;
                case SignalType::BDS_B2I:
                case SignalType::BDS_B2A:
                    return ephemeris.tgd_secondary * constants::SPEED_OF_LIGHT;
                default:
                    return 0.0;
            }
        default:
            return 0.0;
    }
}

double median(std::vector<double> values) {
    values.erase(std::remove_if(values.begin(), values.end(),
                                [](double value) {
                                    return !std::isfinite(value);
                                }),
                  values.end());
    if (values.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2U;
    return values.size() % 2U == 0U
               ? 0.5 * (values[middle - 1U] + values[middle])
               : values[middle];
}

double percentile(std::vector<double> values, double p) {
    values.erase(std::remove_if(values.begin(), values.end(),
                                [](double value) {
                                    return !std::isfinite(value);
                                }),
                  values.end());
    if (values.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    std::sort(values.begin(), values.end());
    if (values.size() == 1U) return values.front();
    const double rank = 0.5 + p / 100.0 * static_cast<double>(values.size());
    if (rank <= 1.0) return values.front();
    if (rank >= static_cast<double>(values.size())) return values.back();
    const double lower_rank = std::floor(rank);
    const std::size_t lower = static_cast<std::size_t>(lower_rank - 1.0);
    const std::size_t upper = lower + 1U;
    return values[lower] + (rank - lower_rank) * (values[upper] - values[lower]);
}

}  // namespace

std::vector<double> centeredMovingMean(const std::vector<double>& values,
                                       std::size_t window_samples) {
    if (values.empty() || window_samples == 0U) return {};
    std::vector<double> result(values.size(),
                               std::numeric_limits<double>::quiet_NaN());
    const std::size_t left = window_samples / 2U;
    const std::size_t right = window_samples - left - 1U;
    for (std::size_t index = 0U; index < values.size(); ++index) {
        const std::size_t begin = index > left ? index - left : 0U;
        const std::size_t end = std::min(values.size() - 1U, index + right);
        double sum = 0.0;
        std::size_t count = 0U;
        for (std::size_t cursor = begin; cursor <= end; ++cursor) {
            if (std::isfinite(values[cursor])) {
                sum += values[cursor];
                ++count;
            }
        }
        if (count != 0U) result[index] = sum / static_cast<double>(count);
    }
    return result;
}

bool Model::build(const ObservationSeries& base_epochs,
                  const NavigationData& nav,
                  const Config& config) {
    streams_.clear();
    diagnostics_ = Diagnostics{};
    diagnostics_.enabled = true;
    diagnostics_.base_epochs = base_epochs.epochs.size();
    diagnostics_.moving_mean_samples = config.moving_mean_samples;
    if (!config.base_position_ecef.allFinite() ||
        config.base_position_ecef.norm() < 6.0e6 ||
        config.base_position_ecef.norm() > 7.0e6) {
        diagnostics_.failure = "base coordinate is not finite/Earth-valid";
        return false;
    }
    if (!(config.expected_interval_s == 1.0 || config.expected_interval_s == 15.0) ||
        config.moving_mean_samples == 0U) {
        diagnostics_.failure = "unsupported base interval or moving-mean window";
        return false;
    }
    double base_lat = 0.0;
    double base_lon = 0.0;
    double base_height = 0.0;
    ecef2geodetic(config.base_position_ecef, base_lat, base_lon, base_height);
    if (!std::isfinite(base_lat) || !std::isfinite(base_lon) ||
        !std::isfinite(base_height)) {
        diagnostics_.failure = "base coordinate has no finite geodetic form";
        return false;
    }
    GNSSTime previous_time;
    bool have_previous_time = false;
    std::vector<double> observed_intervals;
    for (const auto& epoch : base_epochs.epochs) {
        if (!finiteTime(epoch.time)) {
            diagnostics_.failure = "base epoch time is non-finite";
            return false;
        }
        if (have_previous_time) {
            const double dt = epoch.time - previous_time;
            if (!(dt > 0.0)) {
                diagnostics_.failure = "base epochs are not strictly monotonic";
                return false;
            }
            observed_intervals.push_back(dt);
        }
        previous_time = epoch.time;
        have_previous_time = true;
        for (const auto& observation : epoch.observations) {
            ++diagnostics_.base_observation_rows;
            if (!observation.valid || !observation.has_pseudorange ||
                !(observation.pseudorange > 0.0) ||
                !std::isfinite(observation.pseudorange)) {
                continue;
            }
            ++diagnostics_.matched_base_rows;
            GNSSTime transmit_time =
                epoch.time - observation.pseudorange / constants::SPEED_OF_LIGHT;
            Vector3d satellite_position;
            Vector3d satellite_velocity;
            double satellite_clock_bias = 0.0;
            double satellite_clock_drift = 0.0;
            if (!nav.calculateSatelliteState(observation.satellite,
                                             transmit_time,
                                             satellite_position,
                                             satellite_velocity,
                                             satellite_clock_bias,
                                             satellite_clock_drift)) {
                continue;
            }
            transmit_time = transmit_time - satellite_clock_bias;
            if (!nav.calculateSatelliteState(observation.satellite,
                                             transmit_time,
                                             satellite_position,
                                             satellite_velocity,
                                             satellite_clock_bias,
                                             satellite_clock_drift)) {
                continue;
            }
            const Ephemeris* ephemeris =
                nav.getEphemeris(observation.satellite, transmit_time);
            if (ephemeris == nullptr ||
                !isHealthyForPositioning(observation, *ephemeris)) {
                continue;
            }
            const Vector3d rotated_satellite = earthRotationCorrected(
                satellite_position, config.base_position_ecef);
            const auto geometry =
                nav.calculateGeometry(config.base_position_ecef, rotated_satellite);
            const double geometric_range = geometry.distance;
            if (!(geometric_range > 0.0) || !std::isfinite(geometric_range)) {
                continue;
            }
            double ionosphere_delay = 0.0;
            if (config.use_ionosphere_model && nav.ionosphere_model.valid) {
                ionosphere_delay = models::ionoDelayKlobuchar(
                    base_lat, base_lon, geometry.azimuth, geometry.elevation,
                    epoch.time.tow, nav.ionosphere_model.alpha,
                    nav.ionosphere_model.beta);
                const double frequency_hz =
                    signalFrequencyHz(observation.signal, ephemeris);
                if (frequency_hz > 0.0) {
                    const double scale = constants::GPS_L1_FREQ / frequency_hz;
                    ionosphere_delay *= scale * scale;
                }
            }
            const double troposphere_delay =
                config.use_troposphere_model
                    ? models::tropDelaySaastamoinen(config.base_position_ecef,
                                                    geometry.elevation)
                    : 0.0;
            const double satellite_clock_m =
                satellite_clock_bias * constants::SPEED_OF_LIGHT;
            const double group_delay_m = groupDelayCorrectionMeters(
                observation, *ephemeris,
                config.use_signal_specific_galileo_group_delay);
            // Native FGO's corrected pseudorange model is
            // P + satellite_clock_m - ionosphere - troposphere - group_delay.
            // Keeping the same terms here prevents base atmosphere/TGD from
            // leaking into pc before it is subtracted from rover P.
            const double residual = observation.pseudorange + satellite_clock_m -
                                    ionosphere_delay - troposphere_delay -
                                    group_delay_m - geometric_range;
            if (!std::isfinite(residual)) continue;
            ++diagnostics_.finite_base_residual_rows;
            streams_[{observation.satellite, observation.signal}].push_back(
                {epoch.time, residual});
        }
    }
    // The published MATLAB path selects obsb.dt, which is a route-level
    // sampling interval rather than a requirement that every adjacent epoch
    // be present.  Use the observed median and retain strictly-positive
    // monotonicity above; isolated gaps therefore do not silently change the
    // 1-Hz/15-s window contract.
    if (!observed_intervals.empty()) {
        diagnostics_.base_interval_s = median(observed_intervals);
    }
    if (diagnostics_.base_epochs > 1U &&
        (!std::isfinite(diagnostics_.base_interval_s) ||
         std::abs(diagnostics_.base_interval_s - config.expected_interval_s) > 1.0e-6)) {
        diagnostics_.failure = "observed base interval differs from frozen dt";
        return false;
    }
    diagnostics_.base_interval_s = config.expected_interval_s;
    std::vector<double> absolute_corrections;
    for (auto& [key, samples] : streams_) {
        std::sort(samples.begin(), samples.end(),
                  [](const Sample& lhs, const Sample& rhs) {
                      return lhs.time < rhs.time;
                  });
        if (samples.empty()) continue;
        ++diagnostics_.matching_streams;
        std::vector<double> values;
        values.reserve(samples.size());
        for (const auto& sample : samples) values.push_back(sample.residual_m);
        const std::vector<double> smoothed = centeredMovingMean(
            values, config.moving_mean_samples);
        for (std::size_t index = 0U; index < samples.size(); ++index) {
            if (!std::isfinite(smoothed[index])) continue;
            samples[index].residual_m = smoothed[index];
            ++diagnostics_.smoothed_rows;
            absolute_corrections.push_back(std::abs(smoothed[index]));
        }
    }
    diagnostics_.correction_abs_p50_m = median(absolute_corrections);
    diagnostics_.correction_abs_p95_m = percentile(absolute_corrections, 95.0);
    diagnostics_.correction_abs_max_m = absolute_corrections.empty()
                                            ? std::numeric_limits<double>::quiet_NaN()
                                            : *std::max_element(absolute_corrections.begin(), absolute_corrections.end());
    diagnostics_.built = diagnostics_.matching_streams != 0U &&
                         diagnostics_.smoothed_rows != 0U;
    if (!diagnostics_.built && diagnostics_.failure.empty()) {
        diagnostics_.failure = "no finite same-satellite/signal base residual stream";
    }
    return diagnostics_.built;
}

bool Model::correctionAt(const GNSSTime& time,
                         const SatelliteId& satellite,
                         SignalType signal,
                         double& correction_m) const {
    correction_m = std::numeric_limits<double>::quiet_NaN();
    const auto it = streams_.find({satellite, signal});
    if (it == streams_.end() || it->second.empty() || !finiteTime(time)) {
        return false;
    }
    const auto& samples = it->second;
    if (time < samples.front().time || time > samples.back().time) {
        return false;
    }
    const auto upper = std::lower_bound(
        samples.begin(), samples.end(), time,
        [](const Sample& sample, const GNSSTime& value) {
            return sample.time < value;
        });
    if (upper == samples.begin()) {
        correction_m = upper->residual_m;
        return std::isfinite(correction_m);
    }
    if (upper == samples.end()) {
        correction_m = samples.back().residual_m;
        return std::isfinite(correction_m);
    }
    const Sample& right = *upper;
    const Sample& left = *(upper - 1);
    const double dt = right.time - left.time;
    if (!(dt > 0.0) || !std::isfinite(dt)) return false;
    const double fraction = (time - left.time) / dt;
    correction_m = left.residual_m + fraction * (right.residual_m - left.residual_m);
    return std::isfinite(correction_m);
}

bool Model::hasStream(const SatelliteId& satellite, SignalType signal) const {
    const auto it = streams_.find({satellite, signal});
    return it != streams_.end() && !it->second.empty();
}

}  // namespace libgnss::base_pseudorange_compensation
