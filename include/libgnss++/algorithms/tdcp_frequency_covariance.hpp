#pragma once

// Research primitive, not enabled in any solver. For a simultaneous
// same-satellite TDCP pair, r = a*dI + epsilon, dI ~ N(0,q),
// epsilon ~ N(0,diag(sigma^2)), marginal covariance is diag(sigma^2)+q*a*a'.
// a contains signed residual coefficients in m/m, not raw carrier wavelengths.
// Caller owns physical modelling, exact pair identity, atmospheric correction
// convention and temporal independence. This is NOT equivalent to marginalizing
// independently Huber-robustified rows. Never append this as independent evidence
// alongside the original two rows. No default prior or data-fitted parameter.

#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

namespace libgnss::tdcp_frequency {

struct MarginalNoise {
    Eigen::Matrix2d covariance_m2;
    // W = L^-1, where LL' = covariance. W*r has unit Gaussian covariance.
    Eigen::Matrix2d sqrt_information_per_m;
};

inline MarginalNoise marginalNoise(const Eigen::Vector2d& sigma_m,
                                   const Eigen::Vector2d& coefficients,
                                   double slant_change_variance_m2) {
    if (!sigma_m.allFinite() || (sigma_m.array() <= 0.0).any() ||
        !coefficients.allFinite() ||
        !std::isfinite(slant_change_variance_m2) || slant_change_variance_m2 < 0.0) {
        throw std::invalid_argument("Invalid TDCP marginal covariance inputs");
    }
    MarginalNoise result;
    result.covariance_m2 = sigma_m.array().square().matrix().asDiagonal();
    result.covariance_m2 += slant_change_variance_m2 *
                            coefficients * coefficients.transpose();
    if (!result.covariance_m2.allFinite()) {
        throw std::invalid_argument("Nonfinite TDCP marginal covariance");
    }
    const Eigen::LLT<Eigen::Matrix2d> llt(result.covariance_m2);
    if (llt.info() != Eigen::Success) {
        throw std::invalid_argument("TDCP marginal covariance is not positive definite");
    }
    result.sqrt_information_per_m = llt.matrixL().solve(Eigen::Matrix2d::Identity());
    if (!result.sqrt_information_per_m.allFinite()) {
        throw std::invalid_argument("Nonfinite TDCP marginal whitening");
    }
    return result;
}

}  // namespace libgnss::tdcp_frequency
