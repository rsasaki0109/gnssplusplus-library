#pragma once

#include <Eigen/Dense>

namespace libgnss {

using Eigen::VectorXd;
using Eigen::MatrixXd;

/**
 * Extended LAMBDA solution exposing top-2 candidates and Bootstrap Success Rate.
 *
 * `bootstrap_sr` is the theoretical success probability for integer rounding
 * of the decorrelated ambiguities, computed from the LD factorisation
 * conditional variances D[i] as
 *     BSR = prod_i ( 2 * Phi(0.5 / sqrt(D[i])) - 1 ).
 * It is independent of the float estimate and reflects ambiguity geometry
 * quality alone (Teunissen 1998, "Success probability of integer GPS ambiguity
 * rounding and bootstrapping").
 */
struct LambdaSolution {
    int n = 0;                   // ambiguity dimension
    VectorXd best_amb;           // top-1 integer solution (n x 1, original space)
    VectorXd second_amb;         // top-2 integer solution (n x 1, original space)
    double best_norm = 0.0;      // squared norm of top-1 residual in z-space
    double second_norm = 0.0;    // squared norm of top-2 residual in z-space
    double ratio = 0.0;          // second_norm / best_norm (0 if best_norm == 0)
    double bootstrap_sr = 0.0;   // bootstrap success rate, in [0, 1]
    double min_cond_var = 0.0;   // min(D[i]) — most informative decorrelated ambiguity
    double max_cond_var = 0.0;   // max(D[i]) — least informative decorrelated ambiguity
};

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
 * Extended LAMBDA search returning top-2 candidates, ratio, and bootstrap
 * success rate. Caller can use BSR for FFRT-style gating, principled subset
 * selection (drop ambiguities with largest D[i]), or telemetry. Existing
 * lambdaSearch delegates to this and exposes only best_amb + ratio.
 *
 * @param float_amb  Float ambiguity estimates (n x 1)
 * @param Q_amb      Ambiguity covariance matrix (n x n, positive definite)
 * @param solution   Output: full LAMBDA solution including top-2 and BSR
 * @return true on success
 */
bool lambdaSearchExtended(const VectorXd& float_amb, const MatrixXd& Q_amb,
                          LambdaSolution& solution);

} // namespace libgnss
