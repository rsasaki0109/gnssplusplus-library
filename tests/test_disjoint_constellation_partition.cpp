#include <gtest/gtest.h>

#include <libgnss++/algorithms/disjoint_constellation_partition.hpp>

#include <algorithm>
#include <vector>

namespace {

using libgnss::disjoint_constellation_partition::Candidate;
using libgnss::disjoint_constellation_partition::partition;

TEST(DisjointConstellationPartition, KeepsWholeSystemsDisjoint) {
    const auto result = partition({
        {1, 0}, {1, 1}, {1, 2}, {1, 3},
        {2, 4}, {2, 5}, {2, 6},
        {3, 7}, {3, 8},
    });

    EXPECT_EQ(result.partition_a_system_mask & result.partition_b_system_mask, 0u);
    EXPECT_EQ(result.partition_a.size() + result.partition_b.size(), 9u);
    EXPECT_TRUE(result.available(4));

    for (int index : {0, 1, 2, 3}) {
        const bool in_a = std::find(result.partition_a.begin(),
                                    result.partition_a.end(), index) !=
            result.partition_a.end();
        const bool in_b = std::find(result.partition_b.begin(),
                                    result.partition_b.end(), index) !=
            result.partition_b.end();
        EXPECT_NE(in_a, in_b);
    }
}

TEST(DisjointConstellationPartition, IsDeterministicOnEqualSizedGroups) {
    const std::vector<Candidate> input = {
        {4, 4}, {4, 5}, {2, 0}, {2, 1}, {3, 2}, {3, 3},
    };
    const auto first = partition(input);
    const auto second = partition(input);
    EXPECT_EQ(first.partition_a, second.partition_a);
    EXPECT_EQ(first.partition_b, second.partition_b);
    EXPECT_EQ(first.partition_a_system_mask, second.partition_a_system_mask);
    EXPECT_EQ(first.partition_b_system_mask, second.partition_b_system_mask);
}

TEST(DisjointConstellationPartition, FailsClosedWithoutTwoSupportedPools) {
    const auto one_system = partition({
        {1, 0}, {1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 5},
    });
    EXPECT_FALSE(one_system.available(1));

    const auto undersized = partition({
        {1, 0}, {1, 1}, {1, 2}, {2, 3}, {2, 4}, {2, 5},
    });
    EXPECT_FALSE(undersized.available(4));
}

}  // namespace
