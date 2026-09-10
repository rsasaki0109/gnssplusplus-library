#pragma once

#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace libgnss::native_imu_bias_density {

// Experimental covariance-density conversion, not a sensor calibration.
// N counts mapped IMU samples in the exact inclusive GNSS interval. T is
// the actual duration integrated by the selected native preintegrator.
// Multiplying Q by N/T yields bias marginal N*Q after duration T. Combined
// preintegration still retains motion/bias correlations, unlike independent
// source ImuFactor + BetweenFactor. Never modify a shared params object.
inline double sourceCountScale(std::size_t sample_count,
                               double integrated_duration_s) {
    if (sample_count == 0 || !std::isfinite(integrated_duration_s) ||
        integrated_duration_s <= 0.0) {
        throw std::invalid_argument("IMU bias density requires samples and positive finite duration");
    }
    const double scale = static_cast<double>(sample_count) / integrated_duration_s;
    if (!std::isfinite(scale) || scale <= 0.0) {
        throw std::invalid_argument("IMU bias covariance density scale is invalid");
    }
    return scale;
}

}  // namespace libgnss::native_imu_bias_density
