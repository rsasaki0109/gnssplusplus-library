#pragma once

#include <libgnss++/core/types.hpp>
#include <Eigen/Core>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace libgnss::fusion_initialization {

// In-memory inputs only. Callers must supply same-run native states.
struct RelativeHeightSeed {
    GNSSTime time;
    Eigen::Vector3d position_ecef;
    Eigen::Vector3d velocity_nav;
    bool stopped = false;
};

// Explicit source convention: cumsum(speed), NOT time-integrated distance.
// Strict thresholds reproduce the source selector; this does not establish
// that nearby points are on the same road level. No factors are enabled here.
inline std::vector<std::pair<std::size_t, std::size_t>>
selectSourceSampleSumRelativeHeightPairs(
    const std::vector<RelativeHeightSeed>& seeds,
    double proximity_m = 15.0, double speed_sample_sum_threshold = 100.0) {
    if (!std::isfinite(proximity_m) || proximity_m <= 0.0 ||
        !std::isfinite(speed_sample_sum_threshold) || speed_sample_sum_threshold < 0.0)
        throw std::invalid_argument("invalid relative-height thresholds");
    std::vector<double> cumulative;
    cumulative.reserve(seeds.size());
    double sum = 0.0;
    for (std::size_t i = 0; i < seeds.size(); ++i) {
        const auto& seed = seeds[i];
        if (!std::isfinite(seed.time.tow) || seed.time.tow < 0.0 ||
            seed.time.tow >= 604800.0 || !seed.position_ecef.allFinite() ||
            !seed.velocity_nav.allFinite())
            throw std::invalid_argument("invalid relative-height seed");
        if (i && !(seed.time - seeds[i - 1].time > 0.0))
            throw std::invalid_argument("relative-height epochs must increase");
        sum += seed.velocity_nav.stableNorm();
        if (!std::isfinite(sum))
            throw std::invalid_argument("relative-height speed sum overflow");
        cumulative.push_back(sum);
    }
    std::vector<std::pair<std::size_t, std::size_t>> pairs;
    for (std::size_t i = 0; i < seeds.size(); ++i) {
        if (seeds[i].stopped) continue;
        for (std::size_t j = i + 1; j < seeds.size(); ++j) {
            if (!seeds[j].stopped &&
                cumulative[j] - cumulative[i] > speed_sample_sum_threshold &&
                (seeds[j].position_ecef - seeds[i].position_ecef).stableNorm() < proximity_m)
                pairs.emplace_back(i, j);
        }
    }
    return pairs;
}

}  // namespace libgnss::fusion_initialization
