#pragma once

#include <Eigen/Dense>

namespace libgnss {

using Eigen::VectorXd;
using Eigen::MatrixXd;

/**
 * LAMBDA method for integer least-squares ambiguity resolution.
 *
 * Implements the LAMBDA decorrelation (ref [1]) with modified LAMBDA
 * (mlambda) search (ref [2]), ported from RTKLIB's lambda.c.
 *
 * [1] P.J.G.Teunissen, The least-square ambiguity decorrelation adjustment,
 *     J.Geodesy, Vol.70, 65-82, 1995
 * [2] X.-W.Chang, X.Yang, T.Zhou, MLAMBDA: A modified LAMBDA method for
 *     integer least-squares estimation, J.Geodesy, Vol.79, 552-565, 2005
 *
 * @param float_amb   Float ambiguity estimates (n x 1)
 * @param Q_amb       Ambiguity covariance matrix (n x n, positive definite)
 * @param fixed_amb   Output: best integer solution (n x 1)
 * @param ratio       Output: ratio of 2nd-best to best squared residual
 * @return true on success
 */
bool lambdaSearch(const VectorXd& float_amb, const MatrixXd& Q_amb,
                  VectorXd& fixed_amb, double& ratio);

/**
 * LAMBDA search returning both integer candidates used by the ratio test.
 *
 * This is useful for partial ambiguity resolution, where ambiguities that
 * differ between the best and second-best candidates are removed before a
 * retry.  lambdaSearch() remains the compatibility wrapper for callers that
 * only need the best candidate.
 */
bool lambdaSearchCandidates(const VectorXd& float_amb, const MatrixXd& Q_amb,
                            VectorXd& best_amb, VectorXd& second_amb,
                            double& ratio);

/**
 * LAMBDA search diagnostics for FFRT/PAR and multi-hypothesis shadowing.
 *
 * Candidate integer vectors are returned as columns, ordered by increasing
 * squared ILS residual. conditional_variances contains the diagonal D after
 * LAMBDA reduction. bootstrapped_success_rate is
 * product_i(2 Phi(1/(2 sqrt(D_i))) - 1), evaluated without position truth.
 */
struct LambdaCandidateDiagnostics {
    MatrixXd candidates;
    VectorXd squared_residuals;
    VectorXd conditional_variances;
    // z = decorrelation_transform.transpose() * float ambiguities.
    MatrixXd decorrelation_transform;
    VectorXd decorrelated_float;
    MatrixXd decorrelated_covariance;
    double bootstrapped_success_rate = 0.0;
};

/**
 * Fixed failure-rate ratio-test threshold from Hou et al. (2016).
 *
 * The paper defines mu = best_squared_residual / second_squared_residual,
 * whereas lambdaSearch() reports the reciprocal ratio. The converted
 * threshold below is therefore 1 / mu. A zero mu means reject all candidates.
 *
 * The initial implementation intentionally supports only the published
 * Pf_tol=0.001 coefficient table and ambiguity dimensions 1..66. Unsupported
 * requests fail closed rather than interpolating or extrapolating.
 */
struct FixedFailureRateRatioThreshold {
    // Conservative covariance-derived proxy: 1 - bootstrapped success rate.
    double ils_failure_rate_proxy = 1.0;
    double mu = 0.0;
    double minimum_second_to_best_ratio = 0.0;
    bool accepts_any_candidate = false;
};

bool fixedFailureRateRatioThreshold(
    int ambiguity_count, double bootstrapped_success_rate,
    double tolerable_failure_rate,
    FixedFailureRateRatioThreshold& threshold);

/**
 * Re-evaluate the bootstrapped success-rate lower bound after multiplying the
 * ambiguity covariance by a positive scalar. Returns NaN on invalid input.
 */
double bootstrappedSuccessRate(const VectorXd& conditional_variances,
                               double covariance_scale = 1.0);

/**
 * Number of trailing decorrelated ambiguities meeting an SRC success-rate
 * threshold. LAMBDA reduction orders the trailing entries as the preferred
 * conditional-precision subset. Returns zero on invalid input or when even the
 * most precise ambiguity misses the threshold.
 */
int successRateCriterionSubsetSize(
    const VectorXd& conditional_variances, double covariance_scale,
    double minimum_success_rate);

/**
 * Return the best candidate_count integer vectors and covariance-only quality
 * diagnostics. This API does not apply a ratio threshold or declare FIX.
 */
bool lambdaSearchTopK(const VectorXd& float_amb, const MatrixXd& Q_amb,
                      int candidate_count,
                      LambdaCandidateDiagnostics& diagnostics);

} // namespace libgnss
