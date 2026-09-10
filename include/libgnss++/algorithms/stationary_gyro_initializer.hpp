#pragma once

#include <libgnss++/algorithms/upstream_stop_constraints.hpp>
#include <stdexcept>

namespace libgnss::stationary_gyro {

struct Estimate {
    Eigen::Vector3d bias_radps = Eigen::Vector3d::Zero();
    std::size_t blocks = 0;
    std::size_t stop_samples = 0;
    double block_scatter_rms_radps = 0.0;
};

// Batch-only raw estimate. A stop classification is not proof of zero true
// rotation. No fallback, saved state, or truth-derived calibration is used.
// Fixed support rule matches the Phase359 diagnostic, not an H score fit.
inline Estimate estimate(const std::vector<ImuSample>& samples) {
    if (samples.empty()) throw std::invalid_argument("stationary gyro: empty input");
    const auto detection = upstream_stop::detect(samples, {samples.front().time});
    if (!detection.ok) throw std::invalid_argument(detection.error);
    constexpr std::size_t block_size = 250;
    std::vector<Eigen::Vector3d> means;
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    std::size_t count = 0;
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const bool gap = i > 0 &&
            upstream_stop::detail::timeKey(samples[i].time) -
            upstream_stop::detail::timeKey(samples[i-1].time) > 0.1;
        if (!detection.imu_stop[i] || gap) { count = 0; sum.setZero(); }
        if (!detection.imu_stop[i]) continue;
        sum += samples[i].gyro_raw_radps;
        if (++count == block_size) {
            means.push_back(sum / static_cast<double>(block_size));
            count = 0; sum.setZero();
        }
    }
    if (means.size() < 2)
        throw std::invalid_argument("stationary gyro: fewer than two complete blocks");
    Estimate result;
    result.blocks = means.size();
    result.stop_samples = detection.stop_samples;
    for (const auto& mean : means) result.bias_radps += mean;
    result.bias_radps /= static_cast<double>(means.size());
    for (const auto& mean : means)
        result.block_scatter_rms_radps += (mean - result.bias_radps).squaredNorm();
    result.block_scatter_rms_radps = std::sqrt(
        result.block_scatter_rms_radps / static_cast<double>(means.size()));
    return result;
}

}  // namespace libgnss::stationary_gyro
