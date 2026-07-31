#pragma once

#include "../core/types.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace libgnss::rtk_surplus_validation {

struct Config {
    bool enabled = false;
    bool monitor_only = true;
    bool allow_low_pair_rescue = false;
    int minimum_surplus_pairs = 2;
    int minimum_fixed_pairs_for_rescue = 4;
    double aperture_pdop_lt1_cycles = 0.10;
    double aperture_pdop_1to2_cycles = 0.20;
    double aperture_pdop_gt2_cycles = 0.30;
    bool require_all = true;
    double majority_fraction = 0.50;
};

struct Sample {
    GNSSSystem system = GNSSSystem::GPS;
    double distance_from_integer_cycles = 0.0;
};

struct Outcome {
    bool evaluated = false;
    bool passed = false;
    int fallback_level = -1;
    int surplus_available = 0;
    int surplus_used = 0;
    int passing_pairs = 0;
    double aperture_cycles = 0.0;
    double maximum_distance_cycles = 0.0;
};

inline bool promotionEvidencePassed(bool disjoint_passed,
                                    bool surplus_passed) noexcept {
    return disjoint_passed || surplus_passed;
}

inline bool systemAllowedAtLevel(GNSSSystem system, int level) {
    switch (system) {
        case GNSSSystem::GPS:
        case GNSSSystem::QZSS:
            return true;
        case GNSSSystem::Galileo:
            return level >= 0 && level <= 3;
        case GNSSSystem::BeiDou:
            return level == 0 || level == 1 || level == 4;
        case GNSSSystem::GLONASS:
            return level == 0 || level == 2;
        default:
            return level == 0;
    }
}

inline Outcome evaluate(const std::vector<Sample>& samples,
                        double fixed_set_pdop,
                        const Config& config) {
    Outcome outcome;
    outcome.surplus_available = static_cast<int>(samples.size());
    if (!config.enabled || !std::isfinite(fixed_set_pdop)) {
        return outcome;
    }

    outcome.aperture_cycles =
        fixed_set_pdop < 1.0
            ? config.aperture_pdop_lt1_cycles
            : (fixed_set_pdop <= 2.0
                   ? config.aperture_pdop_1to2_cycles
                   : config.aperture_pdop_gt2_cycles);
    if (!std::isfinite(outcome.aperture_cycles) ||
        outcome.aperture_cycles < 0.0) {
        return outcome;
    }

    const int minimum_pairs = std::max(1, config.minimum_surplus_pairs);
    for (int level = 0; level < 6; ++level) {
        std::vector<double> pool;
        for (const auto& sample : samples) {
            if (std::isfinite(sample.distance_from_integer_cycles) &&
                systemAllowedAtLevel(sample.system, level)) {
                pool.push_back(sample.distance_from_integer_cycles);
            }
        }
        if (static_cast<int>(pool.size()) < minimum_pairs) {
            continue;
        }

        Outcome level_outcome = outcome;
        level_outcome.evaluated = true;
        level_outcome.fallback_level = level;
        level_outcome.surplus_used = static_cast<int>(pool.size());
        level_outcome.maximum_distance_cycles =
            *std::max_element(pool.begin(), pool.end());
        level_outcome.passing_pairs = static_cast<int>(std::count_if(
            pool.begin(), pool.end(), [&](double distance) {
                return distance <= outcome.aperture_cycles;
            }));
        level_outcome.passed =
            config.require_all
                ? level_outcome.passing_pairs ==
                      level_outcome.surplus_used
                : static_cast<double>(level_outcome.passing_pairs) >=
                      std::ceil(std::clamp(config.majority_fraction, 0.0, 1.0) *
                                static_cast<double>(
                                    level_outcome.surplus_used));
        outcome = level_outcome;
        if (outcome.passed) {
            return outcome;
        }
    }
    return outcome;
}

}  // namespace libgnss::rtk_surplus_validation
