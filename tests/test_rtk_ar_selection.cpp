#include <gtest/gtest.h>

#include <libgnss++/algorithms/rtk_ar_selection.hpp>

using namespace libgnss;

TEST(RTKArSelectionTest, FiltersOutRelativeVarianceOutliers) {
    const std::vector<rtk_ar_selection::PairDescriptor> pairs = {
        {GNSSSystem::GPS, 0.01},
        {GNSSSystem::GPS, 0.02},
        {GNSSSystem::Galileo, 0.015},
        {GNSSSystem::BeiDou, 0.018},
        {GNSSSystem::GLONASS, 0.5},
    };

    const auto subset = rtk_ar_selection::filterPairsByRelativeVariance(pairs, 10.0, 1e-4, 4);
    ASSERT_EQ(subset.size(), 4U);
    EXPECT_EQ(subset[0], 0);
    EXPECT_EQ(subset[1], 1);
    EXPECT_EQ(subset[2], 2);
    EXPECT_EQ(subset[3], 3);
}

TEST(RTKArSelectionTest, SkipsVarianceFilterWhenTooFewPairsRemain) {
    const std::vector<rtk_ar_selection::PairDescriptor> pairs = {
        {GNSSSystem::GPS, 0.01},
        {GNSSSystem::GPS, 0.02},
        {GNSSSystem::GLONASS, 100.0},
    };

    const auto subset = rtk_ar_selection::filterPairsByRelativeVariance(pairs, 10.0, 1e-4, 4);
    EXPECT_TRUE(subset.empty());
}

TEST(RTKArSelectionTest, BuildsPreferredConstellationSubsets) {
    const std::vector<rtk_ar_selection::PairDescriptor> pairs = {
        {GNSSSystem::GPS, 0.01},
        {GNSSSystem::GLONASS, 0.02},
        {GNSSSystem::BeiDou, 0.03},
        {GNSSSystem::Galileo, 0.04},
    };

    const auto subsets = rtk_ar_selection::buildPreferredSubsets(pairs);
    ASSERT_EQ(subsets.size(), 3U);
    EXPECT_EQ(subsets[0], (std::vector<int>{0, 3}));
    EXPECT_EQ(subsets[1], (std::vector<int>{0, 2, 3}));
    EXPECT_EQ(subsets[2], (std::vector<int>{0, 1, 3}));
}

TEST(RTKArSelectionTest, BuildsProgressiveWorstVarianceDropSubsets) {
    const std::vector<rtk_ar_selection::PairDescriptor> pairs = {
        {GNSSSystem::GPS, 0.01},
        {GNSSSystem::GPS, 0.50},
        {GNSSSystem::Galileo, 0.20},
        {GNSSSystem::BeiDou, 0.40},
        {GNSSSystem::QZSS, 0.10},
    };

    const auto subsets = rtk_ar_selection::buildProgressiveVarianceDropSubsets(pairs, 4, 3);
    ASSERT_EQ(subsets.size(), 1U);
    EXPECT_EQ(subsets[0], (std::vector<int>{0, 2, 3, 4}));
}
