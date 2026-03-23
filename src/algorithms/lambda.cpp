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

// Solve Z' * F = E for F, where Z is n x n and E is n x m.
// This is equivalent to RTKLIB's solve("T", Z, E, n, m, F).
// Z' * F = E  =>  F = inv(Z') * E = inv(Z)' * E
// Since Z is an integer matrix from Gauss transforms, we use LU.
static int solveZt(int n, int m, const double* Z, const double* E, double* F) {
    // Build Eigen matrices (column-major, matching RTKLIB storage)
    Eigen::Map<const Eigen::MatrixXd> Zm(Z, n, n);
    Eigen::MatrixXd Zt = Zm.transpose();

    // Solve Z' * F = E  for each column of E
    Eigen::FullPivLU<Eigen::MatrixXd> lu(Zt);
    if (!lu.isInvertible()) return -1;

    for (int j = 0; j < m; ++j) {
        Eigen::Map<const Eigen::VectorXd> ej(E + j * n, n);
        Eigen::VectorXd fj = lu.solve(ej);
        for (int i = 0; i < n; ++i) F[i + j * n] = fj(i);
    }
    return 0;
}

bool lambdaSearch(const VectorXd& float_amb, const MatrixXd& Q_amb,
                  VectorXd& fixed_amb, double& ratio) {
    int n = float_amb.size();
    int m = 2;  // find 2 best solutions for ratio test
    if (n <= 0) return false;

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

    // F = Z' \ E  (solve Z' * F = E)
    info = solveZt(n, m, Z.data(), E.data(), F.data());
    if (info != 0) return false;

    // Extract best solution
    fixed_amb.resize(n);
    for (int i = 0; i < n; ++i) fixed_amb(i) = F[i];

    // Ratio test
    ratio = (s[0] > 0.0) ? s[1] / s[0] : 0.0;

    return true;
}

} // namespace libgnss
