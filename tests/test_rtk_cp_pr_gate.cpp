#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include <libgnss++/algorithms/rtk_cp_pr_gate.hpp>

namespace {

using libgnss::rtk_cp_pr_gate::Config;
using libgnss::rtk_cp_pr_gate::EscalationTracker;
using libgnss::rtk_cp_pr_gate::Observation;

TEST(RTKCpPrGateTest, AcceptsConsistentFixedIntegersWithoutUsingPosition) {
    const std::vector<Observation> observations{
        {12.0, 31.0, 19.1},
        {-8.0, 4.0, 11.9},
        {3.0, -6.0, -9.2},
        {18.0, 25.0, 7.1},
    };
    const auto result = libgnss::rtk_cp_pr_gate::evaluate(observations, Config{});
    ASSERT_TRUE(result.valid);
    EXPECT_TRUE(result.consistent);
    EXPECT_EQ(result.checked_pairs, 4u);
    EXPECT_EQ(result.bad_pairs, 0u);
    EXPECT_NEAR(result.max_abs_innovation_m, 0.2, 1e-12);
}

TEST(RTKCpPrGateTest, RejectsCandidateWithTooManyLargeInnovations) {
    Config config;
    config.innovation_threshold_m = 10.0;
    config.max_bad_pairs = 1;
    const std::vector<Observation> observations{
        {0.0, 0.0, 0.0},
        {25.0, 0.0, 0.0},
        {-20.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
    };
    const auto result = libgnss::rtk_cp_pr_gate::evaluate(observations, config);
    ASSERT_TRUE(result.valid);
    EXPECT_FALSE(result.consistent);
    EXPECT_EQ(result.bad_pairs, 2u);
    EXPECT_DOUBLE_EQ(result.max_abs_innovation_m, 25.0);
}

TEST(RTKCpPrGateTest, SkipsNonFinitePairsAndRequiresMinimumGeometry) {
    std::vector<Observation> observations(4);
    observations[0].dd_pseudorange_m = std::numeric_limits<double>::quiet_NaN();
    const auto result = libgnss::rtk_cp_pr_gate::evaluate(observations, Config{});
    EXPECT_FALSE(result.valid);
    EXPECT_FALSE(result.consistent);
    EXPECT_EQ(result.checked_pairs, 3u);
}

TEST(RTKCpPrGateTest, RejectsInvalidConfiguration) {
    Config config;
    config.innovation_threshold_m = 0.0;
    EXPECT_FALSE(libgnss::rtk_cp_pr_gate::evaluate(std::vector<Observation>(4), config).valid);
}

TEST(RTKCpPrGateTest, EscalatesOnlyAfterConfiguredConsecutiveRejections) {
    Config config;
    config.min_pairs = 1;
    const auto bad = libgnss::rtk_cp_pr_gate::evaluate({{20.0, 0.0, 0.0}}, config);
    const auto good = libgnss::rtk_cp_pr_gate::evaluate({{0.0, 0.0, 0.0}}, config);
    EscalationTracker tracker(2);
    EXPECT_FALSE(tracker.update(bad));
    EXPECT_EQ(tracker.consecutiveRejections(), 1u);
    EXPECT_TRUE(tracker.update(bad));
    EXPECT_EQ(tracker.consecutiveRejections(), 2u);
    EXPECT_FALSE(tracker.update(good));
    EXPECT_EQ(tracker.consecutiveRejections(), 0u);
}

}  // namespace
