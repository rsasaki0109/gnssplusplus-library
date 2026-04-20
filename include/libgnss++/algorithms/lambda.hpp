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

} // namespace libgnss
