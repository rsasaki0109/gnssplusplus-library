/**
 * LAMBDA method for integer least-squares ambiguity resolution.
 *
 * Ported line-by-line from RTKLIB's lambda.c (BSD-2-Clause, T.Takasu).
 * Uses Eigen instead of raw C arrays; column-major storage matches RTKLIB.
 */
#include <libgnss++/algorithms/lambda.hpp>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <limits>

namespace libgnss {

static constexpr int LOOPMAX = 10000;

static inline double SGN(double x) { return x <= 0.0 ? -1.0 : 1.0; }
static inline double ROUND(double x) { return std::floor(x + 0.5); }

// LD factorization: Q = L' * diag(D) * L
// L is stored column-major in a flat n*n array (RTKLIB convention).
// L[i + j*n] corresponds to element (row=i, col=j).
// On output L has unit diagonal and lower entries below the diagonal,
// matching RTKLIB's LD() output exactly.
static int ldFactorization(int n, const double* Q, double* L, double* D) {
    std::vector<double> A(n * n);
    std::memcpy(A.data(), Q, sizeof(double) * n * n);

    for (int i = n - 1; i >= 0; --i) {
        D[i] = A[i + i * n];
        if (D[i] <= 0.0) return -1;
        double a = std::sqrt(D[i]);
        for (int j = 0; j <= i; ++j) L[i + j * n] = A[i + j * n] / a;
        for (int j = 0; j <= i - 1; ++j)
            for (int k = 0; k <= j; ++k)
                A[j + k * n] -= L[i + k * n] * L[i + j * n];
        for (int j = 0; j <= i; ++j) L[i + j * n] /= L[i + i * n];
    }
    return 0;
}

// Integer Gauss transformation
static void gauss(int n, double* L, double* Z, int i, int j) {
    int mu = (int)ROUND(L[i + j * n]);
    if (mu != 0) {
        for (int k = i; k < n; ++k) L[k + n * j] -= (double)mu * L[k + i * n];
        for (int k = 0; k < n; ++k) Z[k + n * j] -= (double)mu * Z[k + i * n];
    }
}

// Permutation
static void perm(int n, double* L, double* D, int j, double del, double* Z) {
    double eta = D[j] / del;
    double lam = D[j + 1] * L[j + 1 + j * n] / del;
    D[j] = eta * D[j + 1];
    D[j + 1] = del;
    for (int k = 0; k <= j - 1; ++k) {
        double a0 = L[j + k * n];
        double a1 = L[j + 1 + k * n];
        L[j + k * n] = -L[j + 1 + j * n] * a0 + a1;
        L[j + 1 + k * n] = eta * a0 + lam * a1;
    }
    L[j + 1 + j * n] = lam;
    for (int k = j + 2; k < n; ++k) std::swap(L[k + j * n], L[k + (j + 1) * n]);
    for (int k = 0; k < n; ++k) std::swap(Z[k + j * n], Z[k + (j + 1) * n]);
}

// LAMBDA reduction (z=Z'*a, Qz=Z'*Q*Z=L'*diag(D)*L)
static void reduction(int n, double* L, double* D, double* Z) {
    int j = n - 2, k = n - 2;
    while (j >= 0) {
        if (j <= k)
            for (int i = j + 1; i < n; ++i) gauss(n, L, Z, i, j);
        double del = D[j] + L[j + 1 + j * n] * L[j + 1 + j * n] * D[j + 1];
        if (del + 1e-6 < D[j + 1]) {
            perm(n, L, D, j, del, Z);
            k = j;
            j = n - 2;
        } else {
            --j;
        }
    }
}

// Modified LAMBDA (mlambda) search
static int search(int n, int m, const double* L, const double* D,
                  const double* zs, double* zn, double* s) {
    int nn = 0, imax = 0;
    double maxdist = 1e99, y;

    std::vector<double> S(n * n, 0.0);
    std::vector<double> dist(n), zb(n), z(n), step(n);

    int k = n - 1;
    dist[k] = 0.0;
    zb[k] = zs[k];
    z[k] = ROUND(zb[k]);
    y = zb[k] - z[k];
    step[k] = SGN(y);

    int c;
    for (c = 0; c < LOOPMAX; ++c) {
        double newdist = dist[k] + y * y / D[k];
        if (newdist < maxdist) {
            if (k != 0) {
                dist[--k] = newdist;
                for (int i = 0; i <= k; ++i)
                    S[k + i * n] = S[k + 1 + i * n] + (z[k + 1] - zb[k + 1]) * L[k + 1 + i * n];
                zb[k] = zs[k] + S[k + k * n];
                z[k] = ROUND(zb[k]);
                y = zb[k] - z[k];
                step[k] = SGN(y);
            } else {
                if (nn < m) {
                    if (nn == 0 || newdist > s[imax]) imax = nn;
                    for (int i = 0; i < n; ++i) zn[i + nn * n] = z[i];
                    s[nn++] = newdist;
                } else {
                    if (newdist < s[imax]) {
                        for (int i = 0; i < n; ++i) zn[i + imax * n] = z[i];
                        s[imax] = newdist;
                        for (int i = imax = 0; i < m; ++i)
                            if (s[imax] < s[i]) imax = i;
                    }
                    maxdist = s[imax];
                }
                z[0] += step[0];
                y = zb[0] - z[0];
                step[0] = -step[0] - SGN(step[0]);
            }
        } else {
            if (k == n - 1) break;
            ++k;
            z[k] += step[k];
            y = zb[k] - z[k];
            step[k] = -step[k] - SGN(step[k]);
        }
    }

    // Sort by s
    for (int i = 0; i < m - 1; ++i) {
        for (int j = i + 1; j < m; ++j) {
            if (s[i] < s[j]) continue;
            std::swap(s[i], s[j]);
            for (int kk = 0; kk < n; ++kk) std::swap(zn[kk + i * n], zn[kk + j * n]);
        }
    }

    if (c >= LOOPMAX) return -1;
    return 0;
}

// Back-transform one integer candidate from z-space by solving Z' * f = e.
static int solveZt(int n, const double* Z, const double* e, double* f) {
    // Build Eigen matrices (column-major, matching RTKLIB storage)
    Eigen::Map<const Eigen::MatrixXd> Zm(Z, n, n);
    Eigen::MatrixXd Zt = Zm.transpose();

    // Z is unimodular after Gauss/permutation transforms, so a cheaper LU is enough.
    Eigen::PartialPivLU<Eigen::MatrixXd> lu(Zt);
    if (lu.matrixLU().diagonal().cwiseAbs().minCoeff() <= 0.0) return -1;

    const Eigen::Map<const Eigen::VectorXd> rhs(e, n);
    const Eigen::VectorXd solution = lu.solve(rhs);
    if (!solution.array().isFinite().all()) return -1;
    for (int i = 0; i < n; ++i) f[i] = solution(i);
    return 0;
}

bool lambdaSearch(const VectorXd& float_amb, const MatrixXd& Q_amb,
                  VectorXd& fixed_amb, double& ratio) {
    VectorXd second_amb;
    return lambdaSearchCandidates(
        float_amb, Q_amb, fixed_amb, second_amb, ratio);
}

bool lambdaSearchCandidates(const VectorXd& float_amb, const MatrixXd& Q_amb,
                            VectorXd& best_amb, VectorXd& second_amb,
                            double& ratio) {
    LambdaCandidateDiagnostics diagnostics;
    if (!lambdaSearchTopK(float_amb, Q_amb, 2, diagnostics)) {
        return false;
    }
    best_amb = diagnostics.candidates.col(0);
    second_amb = diagnostics.candidates.col(1);
    const double best_residual = diagnostics.squared_residuals(0);
    const double second_residual = diagnostics.squared_residuals(1);
    ratio = (best_residual > 0.0) ? second_residual / best_residual : 0.0;
    return true;
}

bool fixedFailureRateRatioThreshold(
    int ambiguity_count, double bootstrapped_success_rate,
    double tolerable_failure_rate,
    FixedFailureRateRatioThreshold& threshold) {
    // CoefficientMu.csv, Pf_tol=0.001, from the supplementary material for:
    // Hou, Verhagen, Wu, Sensors 2016, 16, 945, doi:10.3390/s16070945.
    // mu(Pf_ILS) = p1 * Pf_ILS^p2 + p3 for Pf_tol < Pf_ILS < 0.2.
    static constexpr double coefficients[66][3] = {
        {0.0549, -0.4626, -0.1968},
        {0.0507, -0.4739, -0.1450},
        {0.0838, -0.3960, -0.1556},
        {0.1343, -0.3225, -0.1755},
        {0.1946, -0.2672, -0.1980},
        {0.1876, -0.2651, -0.1429},
        {0.1645, -0.2750, -0.0755},
        {0.1751, -0.2605, -0.0404},
        {0.1229, -0.3011, 0.0634},
        {0.1133, -0.3065, 0.1151},
        {0.0938, -0.3238, 0.1795},
        {0.0636, -0.3737, 0.2505},
        {0.0630, -0.3670, 0.2833},
        {0.0522, -0.3879, 0.3263},
        {0.0512, -0.3843, 0.3543},
        {0.0498, -0.3824, 0.3789},
        {0.0483, -0.3801, 0.4054},
        {0.0489, -0.3726, 0.4257},
        {0.0492, -0.3659, 0.4450},
        {0.0454, -0.3699, 0.4690},
        {0.0443, -0.3689, 0.4880},
        {0.0419, -0.3721, 0.5072},
        {0.0347, -0.3933, 0.5322},
        {0.0321, -0.3999, 0.5500},
        {0.0318, -0.3958, 0.5613},
        {0.0273, -0.4144, 0.5805},
        {0.0261, -0.4147, 0.5928},
        {0.0242, -0.4219, 0.6072},
        {0.0226, -0.4288, 0.6193},
        {0.0208, -0.4348, 0.6309},
        {0.0172, -0.4602, 0.6431},
        {0.0189, -0.4421, 0.6524},
        {0.0212, -0.4206, 0.6574},
        {0.0197, -0.4278, 0.6673},
        {0.0206, -0.4178, 0.6716},
        {0.0174, -0.4399, 0.6852},
        {0.0182, -0.4294, 0.6901},
        {0.0161, -0.4431, 0.7004},
        {0.0132, -0.4681, 0.7071},
        {0.0137, -0.4613, 0.7155},
        {0.0117, -0.4808, 0.7232},
        {0.0118, -0.4736, 0.7286},
        {0.0103, -0.4912, 0.7351},
        {0.0111, -0.4773, 0.7402},
        {0.0095, -0.4982, 0.7474},
        {0.0095, -0.4969, 0.7525},
        {0.0085, -0.5058, 0.7578},
        {0.0098, -0.4837, 0.7602},
        {0.0105, -0.4706, 0.7633},
        {0.0108, -0.4651, 0.7673},
        {0.0072, -0.5210, 0.7757},
        {0.0079, -0.5051, 0.7767},
        {0.0082, -0.4956, 0.7819},
        {0.0094, -0.4744, 0.7840},
        {0.0077, -0.5017, 0.7885},
        {0.0056, -0.5433, 0.7956},
        {0.0057, -0.5400, 0.7998},
        {0.0086, -0.4742, 0.7975},
        {0.0070, -0.4977, 0.7998},
        {0.0085, -0.4741, 0.8039},
        {0.0107, -0.4327, 0.8016},
        {0.0058, -0.5173, 0.8121},
        {0.0050, -0.5369, 0.8181},
        {0.0081, -0.4521, 0.8137},
        {0.0015, -0.7293, 0.8205},
        {0.0016, -0.7571, 0.8317},
    };

    threshold = {};
    if (ambiguity_count < 1 || ambiguity_count > 66 ||
        !std::isfinite(bootstrapped_success_rate) ||
        bootstrapped_success_rate < 0.0 ||
        bootstrapped_success_rate > 1.0 ||
        !std::isfinite(tolerable_failure_rate) ||
        std::abs(tolerable_failure_rate - 0.001) > 1e-12) {
        return false;
    }

    threshold.ils_failure_rate_proxy = 1.0 - bootstrapped_success_rate;
    if (threshold.ils_failure_rate_proxy >= 0.2 - 1e-12) {
        threshold.minimum_second_to_best_ratio =
            std::numeric_limits<double>::infinity();
        return true;
    }

    if (threshold.ils_failure_rate_proxy <= tolerable_failure_rate) {
        threshold.mu = 1.0;
    } else {
        const double* c = coefficients[ambiguity_count - 1];
        threshold.mu =
            std::clamp(c[0] * std::pow(
                           threshold.ils_failure_rate_proxy, c[1]) + c[2],
                       0.0, 1.0);
    }
    if (!(threshold.mu > 0.0) || !std::isfinite(threshold.mu)) {
        threshold.minimum_second_to_best_ratio =
            std::numeric_limits<double>::infinity();
        return true;
    }
    threshold.minimum_second_to_best_ratio = 1.0 / threshold.mu;
    threshold.accepts_any_candidate = true;
    return true;
}

double bootstrappedSuccessRate(const VectorXd& conditional_variances,
                               double covariance_scale) {
    if (conditional_variances.size() <= 0 ||
        !conditional_variances.array().isFinite().all() ||
        (conditional_variances.array() <= 0.0).any() ||
        !std::isfinite(covariance_scale) || covariance_scale <= 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    double success_rate = 1.0;
    for (int i = 0; i < conditional_variances.size(); ++i) {
        const double scaled_variance =
            covariance_scale * conditional_variances(i);
        const double argument = 0.5 / std::sqrt(2.0 * scaled_variance);
        success_rate *=
            std::clamp(std::erf(argument), 0.0, 1.0);
    }
    return std::clamp(success_rate, 0.0, 1.0);
}

int successRateCriterionSubsetSize(
    const VectorXd& conditional_variances, double covariance_scale,
    double minimum_success_rate) {
    if (conditional_variances.size() <= 0 ||
        !conditional_variances.array().isFinite().all() ||
        (conditional_variances.array() <= 0.0).any() ||
        !std::isfinite(covariance_scale) || covariance_scale <= 0.0 ||
        !std::isfinite(minimum_success_rate) ||
        minimum_success_rate <= 0.0 || minimum_success_rate > 1.0) {
        return 0;
    }
    double success_rate = 1.0;
    int selected = 0;
    for (int i = conditional_variances.size() - 1; i >= 0; --i) {
        const double scaled_variance =
            covariance_scale * conditional_variances(i);
        const double conditional_success =
            std::clamp(
                std::erf(0.5 / std::sqrt(2.0 * scaled_variance)),
                0.0, 1.0);
        const double extended_success_rate =
            success_rate * conditional_success;
        if (extended_success_rate < minimum_success_rate) {
            break;
        }
        success_rate = extended_success_rate;
        ++selected;
    }
    return selected;
}

bool lambdaSearchTopK(const VectorXd& float_amb, const MatrixXd& Q_amb,
                      int candidate_count,
                      LambdaCandidateDiagnostics& diagnostics) {
    int n = float_amb.size();
    const int m = candidate_count;
    if (n <= 0 || m <= 0 ||
        Q_amb.rows() != n || Q_amb.cols() != n ||
        !float_amb.array().isFinite().all() ||
        !Q_amb.array().isFinite().all()) {
        return false;
    }

    // Convert to column-major flat arrays (RTKLIB convention)
    std::vector<double> Q(n * n), L(n * n, 0.0), D(n), Z(n * n, 0.0);
    std::vector<double> a(n), z(n), E(n * m), F(n * m), s(m);

    // Q: column-major
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            Q[i + j * n] = Q_amb(i, j);

    for (int i = 0; i < n; ++i) a[i] = float_amb(i);

    // Z = identity
    for (int i = 0; i < n; ++i) Z[i + i * n] = 1.0;

    // LD factorization
    int info = ldFactorization(n, Q.data(), L.data(), D.data());
    if (info != 0) return false;

    // Reduction
    reduction(n, L.data(), D.data(), Z.data());

    // z = Z' * a
    for (int i = 0; i < n; ++i) {
        z[i] = 0.0;
        for (int j = 0; j < n; ++j) z[i] += Z[j + i * n] * a[j];
    }

    // mlambda search
    info = search(n, m, L.data(), D.data(), z.data(), E.data(), s.data());
    if (info != 0) return false;

    // Back-transform all candidates. Their component-wise disagreement is
    // available to partial-AR and temporal multi-hypothesis shadow policies.
    for (int candidate = 0; candidate < m; ++candidate) {
        info = solveZt(n, Z.data(), E.data() + candidate * n,
                       F.data() + candidate * n);
        if (info != 0) return false;
    }

    diagnostics.candidates.resize(n, m);
    diagnostics.squared_residuals.resize(m);
    diagnostics.conditional_variances.resize(n);
    const Eigen::Map<const Eigen::MatrixXd> decorrelation_transform(
        Z.data(), n, n);
    diagnostics.decorrelation_transform = decorrelation_transform;
    diagnostics.decorrelated_float =
        Eigen::Map<const Eigen::VectorXd>(z.data(), n);
    diagnostics.decorrelated_covariance =
        decorrelation_transform.transpose() * Q_amb *
        decorrelation_transform;
    for (int candidate = 0; candidate < m; ++candidate) {
        diagnostics.squared_residuals(candidate) = s[candidate];
        for (int i = 0; i < n; ++i) {
            diagnostics.candidates(i, candidate) = F[i + candidate * n];
        }
    }

    for (int i = 0; i < n; ++i) {
        diagnostics.conditional_variances(i) = D[i];
    }
    diagnostics.bootstrapped_success_rate =
        bootstrappedSuccessRate(diagnostics.conditional_variances);
    return true;
}

} // namespace libgnss
