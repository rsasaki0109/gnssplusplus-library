#pragma once

#include <cstddef>
#include <limits>
#include <vector>

namespace libgnss::rtk_cp_pr_gate {

struct Observation {
    double dd_pseudorange_m = 0.0;
    double dd_carrier_m = 0.0;
    double fixed_ambiguity_m = 0.0;
};

struct Config {
    double innovation_threshold_m = 10.0;
    std::size_t min_pairs = 4;
    std::size_t max_bad_pairs = 0;
    std::size_t escalation_epochs = 2;
};

struct Result {
    bool valid = false;
    bool consistent = false;
    std::size_t checked_pairs = 0;
    std::size_t bad_pairs = 0;
    double rms_innovation_m = std::numeric_limits<double>::quiet_NaN();
    double max_abs_innovation_m = std::numeric_limits<double>::quiet_NaN();
};

/**
 * Position-independent fixed-integer validation.
 *
 * For each DD pair, evaluates
 *   DD_PR - (DD_CP - fixed_DD_ambiguity_m).
 * Neither the FLOAT nor FIXED candidate position enters the test.
 */
Result evaluate(const std::vector<Observation>& observations, const Config& config);

class EscalationTracker {
public:
    explicit EscalationTracker(std::size_t escalation_epochs = 2)
        : escalation_epochs_(escalation_epochs) {}

    bool update(const Result& result);
    void reset() { consecutive_rejections_ = 0; }
    std::size_t consecutiveRejections() const { return consecutive_rejections_; }

private:
    std::size_t escalation_epochs_ = 2;
    std::size_t consecutive_rejections_ = 0;
};

}  // namespace libgnss::rtk_cp_pr_gate
