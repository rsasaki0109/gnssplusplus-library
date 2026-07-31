#pragma once

#include "../core/observation.hpp"

#include <Eigen/Dense>
#include <limits>
#include <vector>

namespace libgnss {
namespace rtk_ar_selection {

struct PairDescriptor {
    GNSSSystem system = GNSSSystem::GPS;
    double variance = 0.0;
    SatelliteId satellite;
    double elevation_rad = std::numeric_limits<double>::quiet_NaN();
    double snr_dbhz = std::numeric_limits<double>::quiet_NaN();
    double fractional_distance_cycles =
        std::numeric_limits<double>::quiet_NaN();
    // Absolute posterior distance from the nearest integer, expressed in
    // range units. This avoids comparing cycle residuals across frequencies
    // with different wavelengths.
    double posterior_abs_residual_m =
        std::numeric_limits<double>::quiet_NaN();
    // Earth-local azimuth is used only to identify geometrically redundant
    // (crowded) satellites. Body-frame azimuth can replace it once a causal
    // runtime attitude is available.
    double azimuth_rad = std::numeric_limits<double>::quiet_NaN();
};

std::vector<int> filterPairsByRelativeVariance(const std::vector<PairDescriptor>& pairs,
                                               double multiplier = 10.0,
                                               double minimum_threshold = 1e-4,
                                               int minimum_pairs = 4);

std::vector<std::vector<int>> buildPreferredSubsets(const std::vector<PairDescriptor>& pairs);

// Okada/Sasaki/Ando constellation fallback after the full GQEBR set:
// GQEB -> GQER -> GQE -> GQB -> GQ. QZSS is retained with GPS.
std::vector<std::vector<int>> buildPaperConstellationFallbackSubsets(
    const std::vector<PairDescriptor>& pairs);

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

// Satellite-group sequential PAR for urban conditions. All frequencies of a
// target satellite are removed together, avoiding mixed subsets that retain a
// contaminated L1 or L2 leg. Satellites are ranked without position truth by
// equal-weight ordinal ranks of ambiguity variance, posterior integer
// residual in metres, distance to the nearest integer, low elevation, low
// SNR, and azimuth crowding. The returned sequence is largest-first: one
// additional satellite is removed at each step until minimum_pairs or
// max_drop_steps is reached.
std::vector<std::vector<int>> buildSatelliteQualityDropSubsets(
    const std::vector<PairDescriptor>& pairs,
    int minimum_pairs = 4,
    int max_drop_steps = 6);

// Quality-diverse satellite PAR. The combined ordinal ranking above is
// evaluated first for backward compatibility, followed by deduplicated
// nested drop paths driven separately by variance, posterior residual,
// fractional distance, elevation, SNR, and azimuth crowding. This prevents
// one compromised metric from averaging away a useful exclusion path.
std::vector<std::vector<int>> buildSatelliteQualityDiverseDropSubsets(
    const std::vector<PairDescriptor>& pairs,
    int minimum_pairs = 4,
    int max_drop_steps = 6,
    int maximum_subsets = 32);

// Condition the narrow-lane (N1) float state on an independently fixed
// wide-lane relation w = N1 - N5. Q_w must include the uncertainty of both
// N1 and the independently derived N5 term. The cross-covariances with w are
// conservatively taken as Cov(N1,w)=Q_n1 and Cov(head,w)=Q_head_n1.
// Returns false on non-finite, singular, or dimensionally inconsistent input.
bool conditionNarrowLaneOnFixedWideLane(
    const Eigen::VectorXd& head_state,
    const Eigen::MatrixXd& Q_head_n1,
    const Eigen::VectorXd& n1_float,
    const Eigen::MatrixXd& Q_n1,
    const Eigen::VectorXd& wide_lane_float,
    const Eigen::MatrixXd& Q_wide_lane,
    const Eigen::VectorXd& fixed_wide_lane,
    Eigen::VectorXd& conditioned_head_state,
    Eigen::MatrixXd& conditioned_Q_head_n1,
    Eigen::VectorXd& conditioned_n1_float,
    Eigen::MatrixXd& conditioned_Q_n1);

}  // namespace rtk_ar_selection
}  // namespace libgnss
