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
