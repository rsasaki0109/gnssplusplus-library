#include <libgnss++/algorithms/rtk_cp_pr_gate.hpp>

#include <cmath>

namespace libgnss::rtk_cp_pr_gate {

Result evaluate(const std::vector<Observation>& observations, const Config& config) {
    Result result;
    if (!std::isfinite(config.innovation_threshold_m) ||
        config.innovation_threshold_m <= 0.0 || config.min_pairs == 0) {
        return result;
    }

    double squared_sum = 0.0;
    double max_abs = 0.0;
    for (const auto& observation : observations) {
        if (!std::isfinite(observation.dd_pseudorange_m) ||
            !std::isfinite(observation.dd_carrier_m) ||
            !std::isfinite(observation.fixed_ambiguity_m)) {
            continue;
        }
        const double innovation = observation.dd_pseudorange_m -
            (observation.dd_carrier_m - observation.fixed_ambiguity_m);
        const double abs_innovation = std::abs(innovation);
        squared_sum += innovation * innovation;
        max_abs = std::max(max_abs, abs_innovation);
        ++result.checked_pairs;
        if (abs_innovation > config.innovation_threshold_m) {
            ++result.bad_pairs;
        }
    }

    if (result.checked_pairs < config.min_pairs) {
        return result;
    }
    result.valid = true;
    result.consistent = result.bad_pairs <= config.max_bad_pairs;
    result.rms_innovation_m = std::sqrt(squared_sum / result.checked_pairs);
    result.max_abs_innovation_m = max_abs;
    return result;
}

bool EscalationTracker::update(const Result& result) {
    if (!result.valid) {
        consecutive_rejections_ = 0;
        return false;
    }
    if (result.consistent) {
        consecutive_rejections_ = 0;
        return false;
    }
    ++consecutive_rejections_;
    return escalation_epochs_ > 0 && consecutive_rejections_ >= escalation_epochs_;
}

}  // namespace libgnss::rtk_cp_pr_gate
