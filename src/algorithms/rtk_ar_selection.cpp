#include <libgnss++/algorithms/rtk_ar_selection.hpp>

#include <algorithm>

namespace libgnss {
namespace rtk_ar_selection {
namespace {

std::vector<int> buildSubsetExcluding(const std::vector<PairDescriptor>& pairs,
                                      std::initializer_list<GNSSSystem> excluded) {
    std::vector<int> subset;
    subset.reserve(pairs.size());
    for (int index = 0; index < static_cast<int>(pairs.size()); ++index) {
        bool keep = true;
        for (GNSSSystem system : excluded) {
            if (pairs[index].system == system) {
                keep = false;
                break;
            }
        }
        if (keep) {
            subset.push_back(index);
        }
    }
    return subset;
}

}  // namespace

std::vector<int> filterPairsByRelativeVariance(const std::vector<PairDescriptor>& pairs,
                                               double multiplier,
                                               double minimum_threshold,
                                               int minimum_pairs) {
    if (pairs.empty()) {
        return {};
    }

    std::vector<double> variances;
    variances.reserve(pairs.size());
    for (const auto& pair : pairs) {
        variances.push_back(pair.variance);
    }
    std::sort(variances.begin(), variances.end());
    const double median_variance = variances[variances.size() / 2];
    const double threshold = std::max(median_variance * multiplier, minimum_threshold);

    std::vector<int> subset;
    subset.reserve(pairs.size());
    for (int index = 0; index < static_cast<int>(pairs.size()); ++index) {
        if (pairs[index].variance <= threshold) {
            subset.push_back(index);
        }
    }
    if (static_cast<int>(subset.size()) < minimum_pairs) {
        return {};
    }
    return subset;
}

std::vector<std::vector<int>> buildPreferredSubsets(const std::vector<PairDescriptor>& pairs) {
    return {
        buildSubsetExcluding(pairs, {GNSSSystem::GLONASS, GNSSSystem::BeiDou}),
        buildSubsetExcluding(pairs, {GNSSSystem::GLONASS}),
        buildSubsetExcluding(pairs, {GNSSSystem::BeiDou}),
    };
}

std::vector<std::vector<int>> buildBSRGuidedDropSubsets(
    const std::vector<PairDescriptor>& pairs,
    const Eigen::VectorXd& cond_var,
    const Eigen::MatrixXd& decorrelation,
    int minimum_pairs,
    int max_drop_steps,
    int worst_z_count) {
    const int n = static_cast<int>(pairs.size());
    if (n == 0) return {};
    if (cond_var.size() != n) return {};
    if (decorrelation.rows() != n || decorrelation.cols() != n) return {};
    if (worst_z_count < 1) return {};

    // Identify the K worst z-coords (largest D[k]).
    std::vector<std::pair<double, int>> ranked;
    ranked.reserve(n);
    for (int k = 0; k < n; ++k) {
        ranked.emplace_back(cond_var(k), k);
    }
    std::sort(ranked.begin(), ranked.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });

    const int K = std::min(worst_z_count, n);
    std::vector<int> worst_z;
    std::vector<double> worst_z_d;
    worst_z.reserve(K);
    worst_z_d.reserve(K);
    for (int i = 0; i < K; ++i) {
        worst_z.push_back(ranked[i].second);
        worst_z_d.push_back(ranked[i].first);
    }

    // Per-original-pair loading on the worst z-coords.
    std::vector<double> loading(n, 0.0);
    for (int i = 0; i < n; ++i) {
        for (int idx = 0; idx < K; ++idx) {
            const int k = worst_z[idx];
            loading[i] += std::abs(decorrelation(i, k)) * worst_z_d[idx];
        }
    }

    std::vector<std::vector<int>> subsets;
    std::vector<int> current_subset;
    current_subset.reserve(n);
    for (int i = 0; i < n; ++i) current_subset.push_back(i);

    const int max_steps = std::min(n - minimum_pairs, max_drop_steps);
    for (int step = 0; step < max_steps; ++step) {
        int worst_index = -1;
        double worst_load = -1.0;
        for (int i : current_subset) {
            if (loading[i] > worst_load) {
                worst_load = loading[i];
                worst_index = i;
            }
        }
        if (worst_index < 0) break;

        std::vector<int> subset;
        subset.reserve(current_subset.size());
        for (int i : current_subset) {
            if (i != worst_index) subset.push_back(i);
        }
        if (static_cast<int>(subset.size()) < minimum_pairs) break;
        subsets.push_back(subset);
        current_subset = subset;
        loading[worst_index] = -1.0;  // exclude from future picks
    }
    return subsets;
}

std::vector<std::vector<int>> buildProgressiveVarianceDropSubsets(
    const std::vector<PairDescriptor>& pairs,
    int minimum_pairs,
    int max_drop_steps) {
    std::vector<std::vector<int>> subsets;
    std::vector<int> current_subset;
    current_subset.reserve(pairs.size());
    for (int index = 0; index < static_cast<int>(pairs.size()); ++index) {
        current_subset.push_back(index);
    }

    const int max_steps = std::min(static_cast<int>(pairs.size()) - minimum_pairs, max_drop_steps);
    for (int step = 0; step < max_steps; ++step) {
        int worst_index = -1;
        double worst_variance = -1.0;
        for (int index : current_subset) {
            if (pairs[index].variance > worst_variance) {
                worst_variance = pairs[index].variance;
                worst_index = index;
            }
        }
        if (worst_index < 0) {
            break;
        }

        std::vector<int> subset;
        subset.reserve(current_subset.size());
        for (int index : current_subset) {
            if (index != worst_index) {
                subset.push_back(index);
            }
        }
        if (static_cast<int>(subset.size()) < minimum_pairs) {
            break;
        }
        subsets.push_back(subset);
        current_subset = subset;
    }
    return subsets;
}

}  // namespace rtk_ar_selection
}  // namespace libgnss
