#pragma once

#include "../core/observation.hpp"

#include <Eigen/Dense>
#include <vector>

namespace libgnss {
namespace rtk_ar_selection {

struct PairDescriptor {
    GNSSSystem system = GNSSSystem::GPS;
    double variance = 0.0;
};

std::vector<int> filterPairsByRelativeVariance(const std::vector<PairDescriptor>& pairs,
                                               double multiplier = 10.0,
                                               double minimum_threshold = 1e-4,
                                               int minimum_pairs = 4);

std::vector<std::vector<int>> buildPreferredSubsets(const std::vector<PairDescriptor>& pairs);

std::vector<std::vector<int>> buildProgressiveVarianceDropSubsets(
    const std::vector<PairDescriptor>& pairs,
    int minimum_pairs = 4,
    int max_drop_steps = 6);

// BSR-guided partial AR decimation (LAMBDA-geometry-aware variant of
// progressive variance drop). Drops original DD pairs that have the largest
// loadings on the *least-informative* directions of the ambiguity covariance
// Qb — the directions where integer rounding has the lowest bootstrap
// success rate (Teunissen 1998).
//
// Implementation: a self-contained `Eigen::SelfAdjointEigenSolver` on Qb
// gives eigenvalues (axis-aligned variances) and orthonormal eigenvectors.
// The K largest-eigenvalue directions are the worst for integer
// resolution. Per pair i, loading_i = sum_{k in worst K} |eigvec_k(i)| *
// eigval_k. Pairs are dropped greedily by descending loading.
//
// Eigendecomposition is used as a simple covariance-geometry heuristic for
// ranking the original pairs by their contribution to high-variance axes.
// It avoids coupling this helper to the LAMBDA reduction internals.
//
// Returns an empty vector when Qb is the wrong size or non-PSD.
std::vector<std::vector<int>> buildBSRGuidedDropSubsets(
    const std::vector<PairDescriptor>& pairs,
    const Eigen::MatrixXd& Qb,
    int minimum_pairs = 4,
    int max_drop_steps = 6,
    int worst_axes = 3);

}  // namespace rtk_ar_selection
}  // namespace libgnss
