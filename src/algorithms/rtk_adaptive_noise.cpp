#include <libgnss++/algorithms/rtk_adaptive_noise.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace libgnss {
namespace rtk_adaptive_noise {

namespace {

double clampToModelWindow(double variance,
                          double model_variance,
                          const AdaptiveNoiseConfig& config) {
    const double floor_variance = config.min_variance_scale * model_variance;
    const double ceil_variance = config.max_variance_scale * model_variance;
    return std::clamp(variance, floor_variance, ceil_variance);
}

}  // namespace

long long AdaptiveNoiseTracker::combinedKey(int key, rtk_measurement::MeasurementKind kind) {
    return static_cast<long long>(kind) * (1LL << 32) + static_cast<long long>(key);
}

double AdaptiveNoiseTracker::alphaForKind(rtk_measurement::MeasurementKind kind,
                                          const AdaptiveNoiseConfig& config) {
    switch (kind) {
        case rtk_measurement::MeasurementKind::PHASE: return config.alpha_phase;
        case rtk_measurement::MeasurementKind::CODE: return config.alpha_code;
        default: return config.alpha_doppler;
    }
}

double AdaptiveNoiseTracker::adaptedVariance(int key,
                                             rtk_measurement::MeasurementKind kind,
                                             double model_variance,
                                             const AdaptiveNoiseConfig& config) const {
    if (!(model_variance > 0.0) || !std::isfinite(model_variance)) {
        return model_variance;
    }
    const auto it = entries_.find(combinedKey(key, kind));
    if (it == entries_.end()) {
        return model_variance;
    }
    return clampToModelWindow(it->second.variance, model_variance, config);
}

void AdaptiveNoiseTracker::update(int key,
                                  rtk_measurement::MeasurementKind kind,
                                  double innovation,
                                  double hph_diagonal,
                                  double reference_variance,
                                  double model_variance,
                                  double tow,
                                  const AdaptiveNoiseConfig& config) {
    if (!(model_variance > 0.0) || !std::isfinite(model_variance) ||
        !std::isfinite(innovation) || !std::isfinite(hph_diagonal) ||
        !std::isfinite(reference_variance)) {
        return;
    }
    const double sample = innovation * innovation - hph_diagonal - reference_variance;
    const double alpha = std::clamp(alphaForKind(kind, config), 0.0, 1.0);

    const long long entry_key = combinedKey(key, kind);
    auto it = entries_.find(entry_key);
    const double previous = (it != entries_.end()) ? it->second.variance : model_variance;
    const double blended = alpha * previous + (1.0 - alpha) * sample;
    Entry entry;
    entry.variance = clampToModelWindow(blended, model_variance, config);
    entry.model_variance = model_variance;
    entry.last_tow = tow;
    entries_[entry_key] = entry;
}

void AdaptiveNoiseTracker::resetKey(int key, rtk_measurement::MeasurementKind kind) {
    entries_.erase(combinedKey(key, kind));
}

void AdaptiveNoiseTracker::pruneStale(double tow, double reset_gap_s) {
    for (auto it = entries_.begin(); it != entries_.end();) {
        if (std::abs(tow - it->second.last_tow) > reset_gap_s) {
            it = entries_.erase(it);
        } else {
            ++it;
        }
    }
}

void AdaptiveNoiseTracker::clear() {
    entries_.clear();
}

double AdaptiveNoiseTracker::meanVarianceScale(rtk_measurement::MeasurementKind kind) const {
    double sum = 0.0;
    int count = 0;
    for (const auto& [entry_key, entry] : entries_) {
        if (entry_key / (1LL << 32) != static_cast<long long>(kind)) continue;
        if (!(entry.model_variance > 0.0)) continue;
        sum += entry.variance / entry.model_variance;
        ++count;
    }
    return count > 0 ? sum / static_cast<double>(count) : 1.0;
}

}  // namespace rtk_adaptive_noise
}  // namespace libgnss
