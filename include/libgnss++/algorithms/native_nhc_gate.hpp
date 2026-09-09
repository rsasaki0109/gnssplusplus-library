#pragma once

#include <libgnss++/io/imu.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace libgnss {

struct NativeNhcGateResult {
    bool supported = false;
    bool admitted = false;
    std::size_t samples = 0;
    double peak_angular_speed_radps = 0.;
};

// Caller supplies body-FLU samples in [begin,end), all within [t0,t1].
// Full angular speed is deliberately conservative: includes roll and pitch,
// is rotation invariant, and cannot cancel when the direction reverses.
// Both endpoint gaps and every interior gap must meet the coverage limit.
// Thresholds are explicit caller policy, not fitted here from route truth.
inline NativeNhcGateResult nativeNhcGate(
    const std::vector<ImuSample>& samples, std::size_t begin, std::size_t end,
    const GNSSTime& t0, const GNSSTime& t1,
    const Eigen::Vector3d& gyro_bias, double horizontal_speed_mps,
    double min_speed_mps, double max_angular_speed_radps,
    double max_gap_s) {
    NativeNhcGateResult result;
    const double duration = t1 - t0;
    if (begin > end || end > samples.size() || end - begin < 2 ||
        !std::isfinite(t0.tow) || !std::isfinite(t1.tow) ||
        !std::isfinite(duration) || duration <= 0. ||
        !gyro_bias.allFinite() || !std::isfinite(horizontal_speed_mps) ||
        horizontal_speed_mps < 0. || !std::isfinite(min_speed_mps) ||
        min_speed_mps < 0. || !std::isfinite(max_angular_speed_radps) ||
        max_angular_speed_radps < 0. || !std::isfinite(max_gap_s) || max_gap_s <= 0.)
        return result;
    double previous = 0.;
    for (std::size_t k = begin; k < end; ++k) {
        const auto& sample = samples[k];
        const double offset = sample.time - t0;
        const double gap = offset - previous;
        if (!std::isfinite(offset) || offset < 0. || offset > duration ||
            gap > max_gap_s || (k > begin && gap <= 0.) ||
            !sample.accel_raw.allFinite() || !sample.gyro_raw_radps.allFinite())
            return result;
        const double angular_speed = (sample.gyro_raw_radps - gyro_bias).norm();
        if (!std::isfinite(angular_speed)) return result;
        result.peak_angular_speed_radps =
            std::max(result.peak_angular_speed_radps, angular_speed);
        ++result.samples;
        previous = offset;
    }
    if (duration - previous > max_gap_s) return result;
    result.supported = true;
    result.admitted = horizontal_speed_mps >= min_speed_mps &&
        result.peak_angular_speed_radps <= max_angular_speed_radps;
    return result;
}
}  // namespace libgnss
