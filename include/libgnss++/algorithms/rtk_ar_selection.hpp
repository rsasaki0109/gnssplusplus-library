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

// BSR-guided partial AR decimation.
//
// Drops original DD pairs that have the largest loadings on the
// decorrelated z-coordinates with the highest LD-conditional variances
// (lowest per-z bootstrap success rate). The intuition: variance-based
// drops weight raw float-ambiguity uncertainty, but the integer-rounding
// success rate is governed by the LD-decorrelated D[i]. Pairs that
// contribute most to the worst D[i] z-coords are the ones to remove.
//
// Loading score per pair i: sum_{k in worst K z-coords} |Z(i,k)| * D[k].
// Pairs are dropped greedily by descending score, producing up to
// max_drop_steps progressively smaller subsets (drop-1, drop-2, ...).
//
// `cond_var` and `decorrelation` are the LD-conditional variances and
// the unimodular Z transform from `lambdaSearchExtended`. They must be
// of size pairs.size() and pairs.size() x pairs.size() respectively.
//
// Returns an empty vector when LD info is missing or sizes mismatch.
std::vector<std::vector<int>> buildBSRGuidedDropSubsets(
    const std::vector<PairDescriptor>& pairs,
    const Eigen::VectorXd& cond_var,
    const Eigen::MatrixXd& decorrelation,
    int minimum_pairs = 4,
    int max_drop_steps = 6,
    int worst_z_count = 3);

}  // namespace rtk_ar_selection
}  // namespace libgnss
