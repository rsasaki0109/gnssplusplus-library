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

TEST(RTKArSelectionTest, BSRGuidedDecimationDropsHighestEigenLoading) {
    // Diagonal Qb: eigenvalues == diagonal, eigenvectors == identity. The
    // pair with the largest diagonal entry should be dropped first.
    const std::vector<rtk_ar_selection::PairDescriptor> pairs = {
        {GNSSSystem::GPS, 0.01},
        {GNSSSystem::GPS, 0.02},
        {GNSSSystem::Galileo, 0.03},
        {GNSSSystem::BeiDou, 0.04},
        {GNSSSystem::QZSS, 0.05},
    };
    Eigen::MatrixXd Qb = Eigen::MatrixXd::Zero(5, 5);
    Qb.diagonal() << 0.05, 0.40, 0.10, 1.00, 0.20;

    const auto subsets = rtk_ar_selection::buildBSRGuidedDropSubsets(
        pairs, Qb, /*minimum_pairs=*/3, /*max_drop_steps=*/2,
        /*worst_axes=*/3);
    ASSERT_EQ(subsets.size(), 2U);
    EXPECT_EQ(subsets[0], (std::vector<int>{0, 1, 2, 4}));   // dropped 3
    EXPECT_EQ(subsets[1], (std::vector<int>{0, 2, 4}));      // dropped 1
}

TEST(RTKArSelectionTest, BSRGuidedDecimationFollowsCorrelatedAxis) {
    // 4-pair Qb where pair indices 0 and 1 are perfectly correlated through
    // a single high-variance direction. Eigendecomposition exposes that the
    // top eigenvector loads on those two pairs; BSR-guided decimation then
    // drops them ahead of the lower-variance pairs (2, 3) — even though
    // pair 2 has higher *diagonal* variance than pair 0, which would make
    // the variance-based heuristic prefer dropping 2.
    Eigen::MatrixXd Qb = Eigen::MatrixXd::Zero(4, 4);
    Qb(0, 0) = 0.05;
    Qb(1, 1) = 0.05;
    Qb(0, 1) = Qb(1, 0) = 0.04999;  // near-singular pair — huge eigenvalue
    Qb(2, 2) = 0.06;
    Qb(3, 3) = 0.05;

    const std::vector<rtk_ar_selection::PairDescriptor> pairs = {
        {GNSSSystem::GPS, Qb(0, 0)},
        {GNSSSystem::GPS, Qb(1, 1)},
        {GNSSSystem::Galileo, Qb(2, 2)},
        {GNSSSystem::BeiDou, Qb(3, 3)},
    };

    const auto subsets = rtk_ar_selection::buildBSRGuidedDropSubsets(
        pairs, Qb, /*minimum_pairs=*/3, /*max_drop_steps=*/1,
        /*worst_axes=*/1);
    ASSERT_EQ(subsets.size(), 1U);
    // Either pair 0 or pair 1 should be dropped first (they share equal
    // loading on the dominant eigenvector). Pair 2, despite higher
    // diagonal variance, should NOT be the first drop.
    EXPECT_NE(subsets[0],
              (std::vector<int>{0, 1, 3}));  // dropping pair 2 — wrong
    const bool dropped_0 = (subsets[0] == std::vector<int>{1, 2, 3});
    const bool dropped_1 = (subsets[0] == std::vector<int>{0, 2, 3});
    EXPECT_TRUE(dropped_0 || dropped_1);
}

TEST(RTKArSelectionTest, BSRGuidedDecimationRejectsDimensionMismatch) {
    const std::vector<rtk_ar_selection::PairDescriptor> pairs = {
        {GNSSSystem::GPS, 0.01},
        {GNSSSystem::GPS, 0.02},
        {GNSSSystem::Galileo, 0.03},
        {GNSSSystem::BeiDou, 0.04},
    };
    Eigen::MatrixXd Qb = Eigen::MatrixXd::Identity(3, 3);  // wrong size

    const auto subsets = rtk_ar_selection::buildBSRGuidedDropSubsets(
        pairs, Qb, 3, 2, 1);
    EXPECT_TRUE(subsets.empty());
}

TEST(RTKArSelectionTest, BSRGuidedDecimationRejectsNonPsdCovariance) {
    const std::vector<rtk_ar_selection::PairDescriptor> pairs = {
        {GNSSSystem::GPS, 0.01},
        {GNSSSystem::GPS, 0.02},
        {GNSSSystem::Galileo, 0.03},
        {GNSSSystem::BeiDou, 0.04},
    };
    Eigen::MatrixXd Qb = Eigen::MatrixXd::Identity(4, 4);
    Qb(0, 0) = -0.01;

    const auto subsets = rtk_ar_selection::buildBSRGuidedDropSubsets(
        pairs, Qb, 3, 2, 1);
    EXPECT_TRUE(subsets.empty());
}
