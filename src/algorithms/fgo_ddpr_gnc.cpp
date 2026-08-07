#include <libgnss++/algorithms/fgo_ddpr_gnc.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace libgnss::fgo_ddpr_gnc {

Result evaluate(const std::vector<Residual>& residuals, const Config& config) {
    Result result;
    if (residuals.empty() || !std::isfinite(config.shape) || config.shape <= 0.0 ||
        !std::isfinite(config.graduation_divisor) || config.graduation_divisor <= 1.0 ||
        config.max_stages <= 0 || !std::isfinite(config.downweighted_threshold) ||
        config.downweighted_threshold < 0.0 || config.downweighted_threshold > 1.0) {
        return result;
    }

    std::vector<double> normalized_squared;
    normalized_squared.reserve(residuals.size());
    double max_normalized_squared = 0.0;
    for (const auto& residual : residuals) {
        if (!std::isfinite(residual.residual_m) || !std::isfinite(residual.sigma_m) ||
            residual.sigma_m <= 0.0) {
            return result;
        }
        const double normalized = residual.residual_m / residual.sigma_m;
        const double squared = normalized * normalized;
        normalized_squared.push_back(squared);
        max_normalized_squared = std::max(max_normalized_squared, squared);
    }

    const double shape_squared = config.shape * config.shape;
    double mu = std::max(1.0, max_normalized_squared / shape_squared);
    result.initial_mu = mu;
    result.weights.assign(residuals.size(), 1.0);
    double applied_mu = mu;

    // Wen et al.'s continuation schedule: solve at the current scale, update
    // weights, then reduce the scale. In this shadow milestone the residuals
    // remain fixed deliberately; a later counterfactual optimizer can reuse
    // the same deterministic schedule while alternating state solves.
    for (int stage = 0; stage < config.max_stages; ++stage) {
        applied_mu = mu;
        const double scale = mu * shape_squared;
        for (std::size_t i = 0; i < normalized_squared.size(); ++i) {
            result.weights[i] = scale / (scale + normalized_squared[i]);
        }
        ++result.stages;
        if (mu <= 1.0) {
            break;
        }
        mu = std::max(1.0, mu / config.graduation_divisor);
    }

    result.final_mu = applied_mu;
    double weight_sum = 0.0;
    double weighted_square_sum = 0.0;
    result.min_weight = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < residuals.size(); ++i) {
        const double weight = result.weights[i];
        weight_sum += weight;
        weighted_square_sum += weight * residuals[i].residual_m * residuals[i].residual_m;
        result.min_weight = std::min(result.min_weight, weight);
        if (weight < config.downweighted_threshold) {
            ++result.downweighted_factors;
        }
    }
    result.mean_weight = weight_sum / static_cast<double>(residuals.size());
    result.effective_factor_count = weight_sum;
    result.weighted_rms_m = weight_sum > 0.0
        ? std::sqrt(weighted_square_sum / weight_sum)
        : 0.0;
    result.evaluated = true;
    return result;
}

}  // namespace libgnss::fgo_ddpr_gnc
