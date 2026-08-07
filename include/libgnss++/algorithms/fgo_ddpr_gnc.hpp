#pragma once

#include <cstddef>
#include <vector>

namespace libgnss::fgo_ddpr_gnc {

/// Configuration for the diagnostic graduated Geman--McClure schedule.
struct Config {
    double shape = 2.0;
    double graduation_divisor = 1.4;
    int max_stages = 32;
    double downweighted_threshold = 0.5;
};

/// One DD pseudorange residual and the sigma used by its graph factor.
struct Residual {
    double residual_m = 0.0;
    double sigma_m = 1.0;
};

/// Fixed-linearization GNC shadow result. The weights are never fed to FGO.
struct Result {
    bool evaluated = false;
    int stages = 0;
    double initial_mu = 0.0;
    double final_mu = 0.0;
    double min_weight = 1.0;
    double mean_weight = 1.0;
    double effective_factor_count = 0.0;
    std::size_t downweighted_factors = 0;
    double weighted_rms_m = 0.0;
    std::vector<double> weights;
};

/// Evaluates a coarse-to-fine Geman--McClure weight schedule without
/// re-optimizing or modifying estimator state.
Result evaluate(const std::vector<Residual>& residuals,
                const Config& config = Config{});

}  // namespace libgnss::fgo_ddpr_gnc
