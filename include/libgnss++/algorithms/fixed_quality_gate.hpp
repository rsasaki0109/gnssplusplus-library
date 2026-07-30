#pragma once

#include <cmath>

namespace libgnss::fixed_quality_gate {

struct Config {
    bool enabled = false;
    // When enabled, ordinary quality evidence is necessary but no longer
    // sufficient: a separately computed two-family failure budget must also
    // pass.  Kept default-off so the established quality gate remains
    // byte-for-byte opt-in compatible.
    bool require_independent_failure_budget = false;
    // Permit an exported FLOAT to become FIXED only when the causal shadow
    // state has declared the current-epoch LAMBDA candidate and the
    // independent failure budget passes. This never commits held integers
    // to the RTK filter and remains default-off.
    bool allow_safe_shadow_promotion = false;
    // Also permit a current-epoch FFRT candidate with a passing independent
    // failure budget to be evaluated by the ordinary covariance/innovation
    // quality branches. A failed quality branch immediately demotes it.
    bool allow_failure_budget_candidate_promotion = false;
    double maximum_float_position_covariance_trace_m2 = 0.00025;
    double maximum_covariance_branch_nis_per_observation = 10.0;
    int minimum_strong_innovation_observations = 28;
    double maximum_strong_innovation_nis_per_observation = 1.0;
    double maximum_strong_innovation_suppressed_fraction = 0.5;
};

struct Evidence {
    bool safe_fix_shadow_declared_fixed = false;
    bool independent_failure_budget_passed = false;
    double float_position_covariance_trace_m2 = NAN;
    int update_observations = 0;
    int suppressed_outliers = 0;
    double update_nis_per_observation = NAN;
};

struct Decision {
    bool passed = false;
    bool independent_failure_budget_passed = false;
    bool safe_shadow_branch = false;
    bool covariance_branch = false;
    bool strong_innovation_branch = false;
};

inline Decision evaluate(const Config& config, const Evidence& evidence) {
    Decision decision;
    decision.independent_failure_budget_passed =
        evidence.independent_failure_budget_passed;
    decision.safe_shadow_branch =
        evidence.safe_fix_shadow_declared_fixed;
    decision.covariance_branch =
        std::isfinite(evidence.float_position_covariance_trace_m2) &&
        evidence.float_position_covariance_trace_m2 <=
            config.maximum_float_position_covariance_trace_m2 &&
        std::isfinite(evidence.update_nis_per_observation) &&
        evidence.update_nis_per_observation <=
            config.maximum_covariance_branch_nis_per_observation;
    decision.strong_innovation_branch =
        evidence.update_observations >=
            config.minimum_strong_innovation_observations &&
        evidence.suppressed_outliers >= 0 &&
        static_cast<double>(evidence.suppressed_outliers) /
                evidence.update_observations <=
            config.maximum_strong_innovation_suppressed_fraction &&
        std::isfinite(evidence.update_nis_per_observation) &&
        evidence.update_nis_per_observation <=
            config.maximum_strong_innovation_nis_per_observation;
    const bool quality_passed =
        decision.safe_shadow_branch ||
        decision.covariance_branch ||
        decision.strong_innovation_branch;
    decision.passed =
        quality_passed &&
        (!config.require_independent_failure_budget ||
         decision.independent_failure_budget_passed);
    return decision;
}

}  // namespace libgnss::fixed_quality_gate
