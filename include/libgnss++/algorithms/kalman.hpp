#pragma once

#include <Eigen/Dense>

namespace libgnss {

using Eigen::VectorXd;
using Eigen::MatrixXd;

/**
 * Kalman filter measurement update, compatible with RTKLIB's filter() behavior.
 *
 * Performs the standard KF update on active states only (states where
 * x[i] != 0.0 and P[i,i] > 0.0), matching RTKLIB's sparse-state convention.
 *
 * Algorithm:
 *   ix = active state indices (x[i] != 0 && P[i,i] > 0)
 *   x_ = x[ix],  P_ = P[ix,ix],  H_ = H[:,ix]
 *   F  = P_ * H_'
 *   Q  = H_ * P_ * H_' + R
 *   K  = F * Q^{-1}          (solved via LU factorization)
 *   x_ = x_ + K * v
 *   P_ = (I - K * H_) * P_
 *   Write back: x[ix] = x_, P[ix,ix] = P_
 *   Symmetrize P.
 *
 * @param x  State vector (n x 1), modified in-place
 * @param P  Covariance matrix (n x n), modified in-place
 * @param H  Design/observation matrix (m x n), H maps states to measurements
 * @param v  Innovation vector (m x 1), measurement minus predicted
 * @param R  Measurement noise covariance (m x m)
 * @return 0 on success, non-zero on failure (singular Q)
 *
 * Note: RTKLIB's filter() takes H transposed (n x m, column-major).
 * This function takes H in standard form (m x n): z = H * x.
 */
inline int kalmanFilter(VectorXd& x, MatrixXd& P,
                        const MatrixXd& H, const VectorXd& v,
                        const MatrixXd& R) {
    const int n = static_cast<int>(x.size());
    const int m = static_cast<int>(v.size());

    // Collect active state indices: x[i] != 0.0 && P[i,i] > 0.0
    std::vector<int> ix;
    ix.reserve(n);
    for (int i = 0; i < n; ++i) {
        if (x(i) != 0.0 && P(i, i) > 0.0) {
            ix.push_back(i);
        }
    }
    const int k = static_cast<int>(ix.size());
    if (k == 0) return -1;

    // Extract active sub-vectors/matrices
    VectorXd x_(k);
    MatrixXd P_(k, k);
    MatrixXd H_(m, k);

    for (int i = 0; i < k; ++i) {
        x_(i) = x(ix[i]);
        for (int j = 0; j < k; ++j) {
            P_(i, j) = P(ix[i], ix[j]);
        }
        for (int j = 0; j < m; ++j) {
            H_(j, i) = H(j, ix[i]);
        }
    }

    // F = P_ * H_'  (k x m)
    MatrixXd F = P_ * H_.transpose();

    // Q = H_ * P_ * H_' + R = H_ * F + R  (m x m)
    MatrixXd Q = H_ * F + R;

    // K = F * Q^{-1}  =>  Q' * K' = F'  =>  K' = Q^{-T} * F'
    // Since Q is symmetric: K' = Q^{-1} * F', so K = (Q^{-1} * F')'
    // Equivalently: solve Q * K' = F' for K', then transpose
    Eigen::PartialPivLU<MatrixXd> lu(Q);
    MatrixXd Kt = lu.solve(F.transpose());  // (m x k)
    if (!Kt.allFinite()) return -1;
    MatrixXd K = Kt.transpose();  // (k x m)

    // State update: xp = x_ + K * v
    VectorXd xp = x_ + K * v;

    // Covariance update: Pp = (I - K * H_) * P_
    MatrixXd IKH = MatrixXd::Identity(k, k) - K * H_;
    MatrixXd Pp = IKH * P_;

    // Write back to full state
    for (int i = 0; i < k; ++i) {
        x(ix[i]) = xp(i);
        for (int j = 0; j < k; ++j) {
            P(ix[i], ix[j]) = Pp(i, j);
        }
    }

    // Symmetrize P
    P = (P + P.transpose()) * 0.5;

    return 0;
}

} // namespace libgnss
