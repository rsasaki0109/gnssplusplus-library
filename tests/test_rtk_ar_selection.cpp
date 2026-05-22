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

TEST(RTKArSelectionTest, BSRGuidedDecimationDropsHighestLoading) {
    // 5 ambiguities. Z is identity, so loading per pair == |D[k]| for the
    // matching index. Worst z is the one with highest D (index 3).
    // Expected drop order: 3 (largest D), then 1, then 4.
    const std::vector<rtk_ar_selection::PairDescriptor> pairs = {
        {GNSSSystem::GPS, 0.01},
        {GNSSSystem::GPS, 0.02},
        {GNSSSystem::Galileo, 0.03},
        {GNSSSystem::BeiDou, 0.04},
        {GNSSSystem::QZSS, 0.05},
    };
    Eigen::VectorXd D(5);
    D << 0.05, 0.40, 0.10, 1.00, 0.20;
    Eigen::MatrixXd Z = Eigen::MatrixXd::Identity(5, 5);

    const auto subsets = rtk_ar_selection::buildBSRGuidedDropSubsets(
        pairs, D, Z, /*minimum_pairs=*/3, /*max_drop_steps=*/2,
        /*worst_z_count=*/3);
    ASSERT_EQ(subsets.size(), 2U);
    // Drop pair 3 (D=1.0) first.
    EXPECT_EQ(subsets[0], (std::vector<int>{0, 1, 2, 4}));
    // Then drop pair 1 (D=0.40, the next-largest among remaining).
    EXPECT_EQ(subsets[1], (std::vector<int>{0, 2, 4}));
}

TEST(RTKArSelectionTest, BSRGuidedDecimationFollowsZLoading) {
    // Identity D, but Z places all loading from worst-z (index 0) onto
    // pair 2. The drop order should target pair 2 even though all D[i]
    // are equal — the variance-based heuristic would not.
    const std::vector<rtk_ar_selection::PairDescriptor> pairs = {
        {GNSSSystem::GPS, 0.10},
        {GNSSSystem::GPS, 0.10},
        {GNSSSystem::Galileo, 0.10},
        {GNSSSystem::BeiDou, 0.10},
    };
    Eigen::VectorXd D(4);
    D << 1.00, 0.01, 0.01, 0.01;
    Eigen::MatrixXd Z = Eigen::MatrixXd::Identity(4, 4);
    // Concentrate worst-z (column 0) onto pair 2.
    Z(2, 0) = 5.0;

    const auto subsets = rtk_ar_selection::buildBSRGuidedDropSubsets(
        pairs, D, Z, /*minimum_pairs=*/3, /*max_drop_steps=*/1,
        /*worst_z_count=*/1);
    ASSERT_EQ(subsets.size(), 1U);
    EXPECT_EQ(subsets[0], (std::vector<int>{0, 1, 3}));
}

TEST(RTKArSelectionTest, BSRGuidedDecimationRejectsSizeMismatch) {
    const std::vector<rtk_ar_selection::PairDescriptor> pairs = {
        {GNSSSystem::GPS, 0.01},
        {GNSSSystem::GPS, 0.02},
        {GNSSSystem::Galileo, 0.03},
        {GNSSSystem::BeiDou, 0.04},
    };
    Eigen::VectorXd D(3);  // wrong size
    D << 0.05, 0.10, 0.20;
    Eigen::MatrixXd Z = Eigen::MatrixXd::Identity(3, 3);

    const auto subsets = rtk_ar_selection::buildBSRGuidedDropSubsets(
        pairs, D, Z, 3, 2, 1);
    EXPECT_TRUE(subsets.empty());
}
