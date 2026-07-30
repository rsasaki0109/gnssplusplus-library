#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace libgnss::fix_failure_budget {

// Sources in one family may share observations, models, or ambiguity states.
// Multiple algorithms from the same family therefore consume one fault domain.
enum class SourceFamily : std::size_t {
    PRIMARY_CARRIER_AR = 0,
    MULTIFREQUENCY_CASCADE = 1,
    INERTIAL_SOLUTION_SEPARATION = 2,
    DISJOINT_SATELLITE_PARTITION_A = 3,
    DISJOINT_SATELLITE_PARTITION_B = 4,
    COUNT = 5,
};

struct Evidence {
    SourceFamily family = SourceFamily::PRIMARY_CARRIER_AR;
    bool accepted = false;
    double fixed_failure_probability =
        std::numeric_limits<double>::quiet_NaN();
};

struct Config {
    int minimum_independent_families = 2;
    double maximum_joint_failure_probability = 2e-6;
};

struct Decision {
    bool passed = false;
    int independent_families = 0;
    double joint_failure_probability = 1.0;
};

template <std::size_t N>
Decision evaluate(
    const Config& config,
    const std::array<Evidence, N>& evidence) {
    Decision decision;
    constexpr std::size_t family_count =
        static_cast<std::size_t>(SourceFamily::COUNT);
    std::array<double, family_count> best;
    best.fill(std::numeric_limits<double>::infinity());
    for (const auto& source : evidence) {
        const auto family = static_cast<std::size_t>(source.family);
        if (!source.accepted || family >= family_count ||
            !std::isfinite(source.fixed_failure_probability) ||
            source.fixed_failure_probability <= 0.0 ||
            source.fixed_failure_probability > 1.0) {
            continue;
        }
        best[family] =
            std::min(best[family], source.fixed_failure_probability);
    }
    for (const double probability : best) {
        if (!std::isfinite(probability)) {
            continue;
        }
        ++decision.independent_families;
        decision.joint_failure_probability *= probability;
    }
    decision.passed =
        config.minimum_independent_families >= 2 &&
        decision.independent_families >=
            config.minimum_independent_families &&
        std::isfinite(config.maximum_joint_failure_probability) &&
        config.maximum_joint_failure_probability > 0.0 &&
        decision.joint_failure_probability <=
            config.maximum_joint_failure_probability;
    return decision;
}

}  // namespace libgnss::fix_failure_budget
