#pragma once

#include <algorithm>
#include <cstdint>
#include <map>
#include <utility>
#include <vector>

namespace libgnss::disjoint_constellation_partition {

struct Candidate {
    int system = -1;
    int index = -1;
};

struct Result {
    std::vector<int> partition_a;
    std::vector<int> partition_b;
    std::uint64_t partition_a_system_mask = 0;
    std::uint64_t partition_b_system_mask = 0;

    bool available(int minimum_per_partition) const {
        return minimum_per_partition > 0 &&
            static_cast<int>(partition_a.size()) >= minimum_per_partition &&
            static_cast<int>(partition_b.size()) >= minimum_per_partition;
    }
};

// Keep complete constellation groups together so a target and its
// same-constellation DD reference can never be shared across partitions.
// Largest groups are assigned first to the currently smaller partition;
// constellation ID breaks ties deterministically.
inline Result partition(const std::vector<Candidate>& candidates) {
    std::map<int, std::vector<int>> by_system;
    for (const auto& candidate : candidates) {
        if (candidate.system < 0 || candidate.index < 0) continue;
        by_system[candidate.system].push_back(candidate.index);
    }

    std::vector<std::pair<int, std::vector<int>>> groups;
    groups.reserve(by_system.size());
    for (auto& [system, indexes] : by_system) {
        groups.emplace_back(system, std::move(indexes));
    }
    std::sort(groups.begin(), groups.end(), [](const auto& lhs, const auto& rhs) {
        if (lhs.second.size() != rhs.second.size()) {
            return lhs.second.size() > rhs.second.size();
        }
        return lhs.first < rhs.first;
    });

    Result result;
    for (const auto& [system, indexes] : groups) {
        auto* destination = &result.partition_a;
        auto* mask = &result.partition_a_system_mask;
        if (result.partition_b.size() < result.partition_a.size()) {
            destination = &result.partition_b;
            mask = &result.partition_b_system_mask;
        }
        destination->insert(destination->end(), indexes.begin(), indexes.end());
        if (system < 64) *mask |= std::uint64_t{1} << system;
    }
    return result;
}

}  // namespace libgnss::disjoint_constellation_partition
