#pragma once

// Raw-only stationary-stop contract used by the opt-in smartphone batch FGO
// lane.  The thresholds and the nearest epoch mapping mirror the pinned
// taroz fgo_gnss_imu.m/imuprocessing.m specification; this helper deliberately
// does not inspect a trajectory, truth, or any device-provided coordinates.

#include <libgnss++/io/imu.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace libgnss::upstream_stop {

struct Config {
    std::size_t window_samples = 500;
    double acceleration_std_offset_mps2 = 0.08;
    double gyro_std_offset_radps = 0.005;
    double gyro_norm_max_radps = 0.05;
};

struct Detection {
    bool ok = false;
    std::string error;
    std::vector<bool> imu_stop;
    std::vector<bool> epoch_stop;
    std::size_t finite_samples = 0;
    std::size_t stop_samples = 0;
    std::size_t stop_epochs = 0;
    double acceleration_min_std_mps2 = std::numeric_limits<double>::quiet_NaN();
    double acceleration_std_threshold_mps2 = std::numeric_limits<double>::quiet_NaN();
    double gyro_min_std_radps = std::numeric_limits<double>::quiet_NaN();
    double gyro_std_threshold_radps = std::numeric_limits<double>::quiet_NaN();
};

namespace detail {

inline double timeKey(const GNSSTime& time) {
    return static_cast<double>(time.week) * 604800.0 + time.tow;
}

inline double sampleStd(const std::vector<double>& values, std::size_t begin,
                        std::size_t end) {
    const std::size_t count = end > begin ? end - begin : 0U;
    if (count < 2U) return std::numeric_limits<double>::quiet_NaN();
    double mean = 0.0;
    for (std::size_t i = begin; i < end; ++i) mean += values[i];
    mean /= static_cast<double>(count);
    double sum = 0.0;
    for (std::size_t i = begin; i < end; ++i) {
        const double delta = values[i] - mean;
        sum += delta * delta;
    }
    // MATLAB movstd's default normalization is N-1.  Keep that convention
    // explicit rather than silently substituting a population standard
    // deviation for short fixture windows.
    return std::sqrt(sum / static_cast<double>(count - 1U));
}

}  // namespace detail

inline Detection detect(const std::vector<ImuSample>& samples,
                        const std::vector<GNSSTime>& epoch_times,
                        const Config& config = {}) {
    Detection result;
    if (samples.size() < 2U) {
        result.error = "station-stop detection requires at least two IMU samples";
        return result;
    }
    if (epoch_times.empty()) {
        result.error = "station-stop detection requires at least one GNSS epoch";
        return result;
    }
    if (config.window_samples < 2U ||
        !std::isfinite(config.acceleration_std_offset_mps2) ||
        !std::isfinite(config.gyro_std_offset_radps) ||
        !std::isfinite(config.gyro_norm_max_radps) ||
        config.acceleration_std_offset_mps2 < 0.0 ||
        config.gyro_std_offset_radps < 0.0 ||
        config.gyro_norm_max_radps <= 0.0) {
        result.error = "invalid stationary-stop configuration";
        return result;
    }

    std::vector<double> sample_times;
    std::vector<double> acceleration_norms;
    std::vector<double> gyro_norms;
    sample_times.reserve(samples.size());
    acceleration_norms.reserve(samples.size());
    gyro_norms.reserve(samples.size());
    double previous_time = -std::numeric_limits<double>::infinity();
    for (const auto& sample : samples) {
        const double time = detail::timeKey(sample.time);
        const double acceleration_norm = sample.accel_raw.norm();
        const double gyro_norm = sample.gyro_raw_radps.norm();
        if (!std::isfinite(time) || !std::isfinite(acceleration_norm) ||
            !std::isfinite(gyro_norm)) {
            result.error = "non-finite stationary-stop sample";
            return result;
        }
        if (time <= previous_time) {
            result.error = "stationary-stop samples must have strictly increasing times";
            return result;
        }
        previous_time = time;
        sample_times.push_back(time);
        acceleration_norms.push_back(acceleration_norm);
        gyro_norms.push_back(gyro_norm);
    }
    result.finite_samples = samples.size();

    std::vector<double> acceleration_stds(samples.size(),
                                           std::numeric_limits<double>::quiet_NaN());
    std::vector<double> gyro_stds(samples.size(),
                                  std::numeric_limits<double>::quiet_NaN());
    const std::size_t left = (config.window_samples - 1U) / 2U;
    const std::size_t right = config.window_samples / 2U;
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const std::size_t begin = i > left ? i - left : 0U;
        const std::size_t end = std::min(samples.size(), i + right + 1U);
        acceleration_stds[i] = detail::sampleStd(acceleration_norms, begin, end);
        gyro_stds[i] = detail::sampleStd(gyro_norms, begin, end);
    }
    for (double value : acceleration_stds) {
        if (std::isfinite(value)) {
            result.acceleration_min_std_mps2 =
                std::isfinite(result.acceleration_min_std_mps2)
                    ? std::min(result.acceleration_min_std_mps2, value)
                    : value;
        }
    }
    for (double value : gyro_stds) {
        if (std::isfinite(value)) {
            result.gyro_min_std_radps =
                std::isfinite(result.gyro_min_std_radps)
                    ? std::min(result.gyro_min_std_radps, value)
                    : value;
        }
    }
    if (!std::isfinite(result.acceleration_min_std_mps2) ||
        !std::isfinite(result.gyro_min_std_radps)) {
        result.error = "stationary-stop window has no finite standard deviation";
        return result;
    }
    result.acceleration_std_threshold_mps2 =
        result.acceleration_min_std_mps2 + config.acceleration_std_offset_mps2;
    result.gyro_std_threshold_radps =
        result.gyro_min_std_radps + config.gyro_std_offset_radps;
    result.imu_stop.assign(samples.size(), false);
    for (std::size_t i = 0; i < samples.size(); ++i) {
        result.imu_stop[i] = std::isfinite(acceleration_stds[i]) &&
            std::isfinite(gyro_stds[i]) &&
            acceleration_stds[i] < result.acceleration_std_threshold_mps2 &&
            gyro_stds[i] < result.gyro_std_threshold_radps &&
            gyro_norms[i] < config.gyro_norm_max_radps;
        if (result.imu_stop[i]) ++result.stop_samples;
    }

    std::vector<double> epoch_keys;
    epoch_keys.reserve(epoch_times.size());
    double previous_epoch_time = -std::numeric_limits<double>::infinity();
    for (const auto& epoch_time : epoch_times) {
        const double time = detail::timeKey(epoch_time);
        if (!std::isfinite(time) || time <= previous_epoch_time) {
            result.error = "GNSS epoch times must be finite and strictly increasing";
            return result;
        }
        previous_epoch_time = time;
        epoch_keys.push_back(time);
    }
    result.epoch_stop.assign(epoch_keys.size(), false);
    for (std::size_t i = 0; i < epoch_keys.size(); ++i) {
        const auto it = std::lower_bound(sample_times.begin(), sample_times.end(),
                                         epoch_keys[i]);
        std::size_t selected = 0U;
        if (it == sample_times.begin()) {
            selected = 0U;
        } else if (it == sample_times.end()) {
            selected = sample_times.size() - 1U;
        } else {
            const std::size_t next = static_cast<std::size_t>(it - sample_times.begin());
            const std::size_t previous = next - 1U;
            // MATLAB's nearest interpolation is made deterministic at an
            // exact tie by retaining the preceding sample.
            selected = (epoch_keys[i] - sample_times[previous] <=
                        sample_times[next] - epoch_keys[i])
                           ? previous
                           : next;
        }
        result.epoch_stop[i] = result.imu_stop[selected];
        if (result.epoch_stop[i]) ++result.stop_epochs;
    }
    result.ok = true;
    return result;
}

}  // namespace libgnss::upstream_stop
