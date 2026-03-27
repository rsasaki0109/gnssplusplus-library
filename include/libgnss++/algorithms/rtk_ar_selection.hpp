#pragma once

#include "../core/observation.hpp"

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

}  // namespace rtk_ar_selection
}  // namespace libgnss
