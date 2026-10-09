#pragma once

#include <Eigen/Dense>

namespace libgnss::rtk_covariance_consistency {

/**
 * @brief Variance factor s >= 1 that makes a FLOAT solution consistent with
 * an independent code-only (SPP) solution of the same epoch.
 *
 * `difference` is float_position - spp_position. If both estimates were
 * consistent, `difference` has covariance `float_covariance +
 * spp_covariance`. The returned s is the smallest value >= 1 for which
 * difference' * (s * float_covariance + spp_covariance)^-1 * difference <= 3,
 * the expected value of a three degree-of-freedom statistic, so a FLOAT whose
 * Kalman covariance is incompatible with the SPP solution reports a variance
 * that explains the disagreement, while a compatible one is left unchanged
 * (s == 1). Returns 1 for non-finite or indefinite input (no evidence).
 * The state and positions are never touched; this is a reporting factor.
 */
double sppConsistencyScale(const Eigen::Vector3d& difference,
                           const Eigen::Matrix3d& float_covariance,
                           const Eigen::Matrix3d& spp_covariance);

}  // namespace libgnss::rtk_covariance_consistency
