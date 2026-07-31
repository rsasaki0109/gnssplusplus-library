#include <gtest/gtest.h>

#include <algorithm>

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

TEST(RTKArSelectionTest, BuildsPaperConstellationFallbackSequence) {
    const std::vector<rtk_ar_selection::PairDescriptor> pairs = {
        {GNSSSystem::GPS, 0.01},
        {GNSSSystem::QZSS, 0.02},
        {GNSSSystem::Galileo, 0.03},
        {GNSSSystem::BeiDou, 0.04},
        {GNSSSystem::GLONASS, 0.05},
    };

    const auto subsets =
        rtk_ar_selection::buildPaperConstellationFallbackSubsets(pairs);
    ASSERT_EQ(subsets.size(), 5U);
    EXPECT_EQ(subsets[0], (std::vector<int>{0, 1, 2, 3}));  // GQEB
    EXPECT_EQ(subsets[1], (std::vector<int>{0, 1, 2, 4}));  // GQER
    EXPECT_EQ(subsets[2], (std::vector<int>{0, 1, 2}));     // GQE
    EXPECT_EQ(subsets[3], (std::vector<int>{0, 1, 3}));     // GQB
    EXPECT_EQ(subsets[4], (std::vector<int>{0, 1}));        // GQ
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

TEST(RTKArSelectionTest, WideLaneConditioningReducesNarrowLaneVariance) {
    Eigen::VectorXd head(3);
    head << 10.0, 20.0, 30.0;
    Eigen::MatrixXd Qab = Eigen::MatrixXd::Zero(3, 2);
    Qab(0, 0) = 0.2;
    Qab(1, 1) = 0.1;
    Eigen::VectorXd n1(2);
    n1 << 4.4, -2.3;
    Eigen::MatrixXd Qn1 = Eigen::MatrixXd::Identity(2, 2);
    Eigen::VectorXd wl(2);
    wl << 1.4, -1.3;
    Eigen::MatrixXd Qwl = 2.0 * Eigen::MatrixXd::Identity(2, 2);
    Eigen::VectorXd fixed(2);
    fixed << 1.0, -1.0;
    Eigen::VectorXd conditioned_head;
    Eigen::MatrixXd conditioned_Qab;
    Eigen::VectorXd conditioned_n1;
    Eigen::MatrixXd conditioned_Qn1;

    ASSERT_TRUE(
        rtk_ar_selection::conditionNarrowLaneOnFixedWideLane(
            head, Qab, n1, Qn1, wl, Qwl, fixed, conditioned_head,
            conditioned_Qab, conditioned_n1, conditioned_Qn1));

    EXPECT_TRUE(conditioned_n1.isApprox(
        (Eigen::VectorXd(2) << 4.2, -2.15).finished(), 1e-12));
    EXPECT_TRUE(conditioned_Qn1.isApprox(
        0.5 * Eigen::MatrixXd::Identity(2, 2), 1e-12));
    EXPECT_LT(conditioned_head(0), head(0));
    EXPECT_GT(conditioned_head(1), head(1));
}

TEST(RTKArSelectionTest, WideLaneConditioningRejectsSingularCovariance) {
    Eigen::VectorXd head = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd Qab = Eigen::MatrixXd::Zero(3, 1);
    Eigen::VectorXd n1 = Eigen::VectorXd::Zero(1);
    Eigen::MatrixXd Qn1 = Eigen::MatrixXd::Identity(1, 1);
    Eigen::VectorXd wl = Eigen::VectorXd::Zero(1);
    Eigen::MatrixXd Qwl = Eigen::MatrixXd::Zero(1, 1);
    Eigen::VectorXd fixed = Eigen::VectorXd::Zero(1);
    Eigen::VectorXd conditioned_head;
    Eigen::MatrixXd conditioned_Qab;
    Eigen::VectorXd conditioned_n1;
    Eigen::MatrixXd conditioned_Qn1;

    EXPECT_FALSE(
        rtk_ar_selection::conditionNarrowLaneOnFixedWideLane(
            head, Qab, n1, Qn1, wl, Qwl, fixed, conditioned_head,
            conditioned_Qab, conditioned_n1, conditioned_Qn1));
}

TEST(RTKArSelectionTest, SatelliteQualityDropRemovesAllFrequenciesTogether) {
    const SatelliteId g01(GNSSSystem::GPS, 1);
    const SatelliteId g02(GNSSSystem::GPS, 2);
    const SatelliteId e03(GNSSSystem::Galileo, 3);
    const std::vector<rtk_ar_selection::PairDescriptor> pairs = {
        {GNSSSystem::GPS, 0.01, g01, 0.8, 45.0, 0.02},
        {GNSSSystem::GPS, 0.02, g01, 0.8, 44.0, 0.03},
        {GNSSSystem::GPS, 0.80, g02, 0.2, 24.0, 0.48},
        {GNSSSystem::GPS, 0.70, g02, 0.2, 23.0, 0.45},
        {GNSSSystem::Galileo, 0.04, e03, 0.6, 40.0, 0.08},
        {GNSSSystem::Galileo, 0.05, e03, 0.6, 39.0, 0.09},
    };
    const auto subsets =
        rtk_ar_selection::buildSatelliteQualityDropSubsets(
            pairs, /*minimum_pairs=*/2, /*max_drop_steps=*/2);
    ASSERT_EQ(subsets.size(), 2U);
    EXPECT_EQ(subsets[0], (std::vector<int>{0, 1, 4, 5}));
    EXPECT_EQ(subsets[1], (std::vector<int>{0, 1}));
}

TEST(RTKArSelectionTest, SatelliteQualityDropFailsClosedAtPairFloor) {
    const SatelliteId g01(GNSSSystem::GPS, 1);
    const SatelliteId g02(GNSSSystem::GPS, 2);
    const std::vector<rtk_ar_selection::PairDescriptor> pairs = {
        {GNSSSystem::GPS, 0.01, g01},
        {GNSSSystem::GPS, 0.02, g01},
        {GNSSSystem::GPS, 0.80, g02},
        {GNSSSystem::GPS, 0.70, g02},
    };
    EXPECT_TRUE(
        rtk_ar_selection::buildSatelliteQualityDropSubsets(
            pairs, /*minimum_pairs=*/3, /*max_drop_steps=*/2)
            .empty());
}

TEST(RTKArSelectionTest, SatelliteQualityDropUsesPosteriorResidualInMeters) {
    const SatelliteId g01(GNSSSystem::GPS, 1);
    const SatelliteId g02(GNSSSystem::GPS, 2);
    const SatelliteId g03(GNSSSystem::GPS, 3);
    std::vector<rtk_ar_selection::PairDescriptor> pairs(3);
    pairs[0].system = GNSSSystem::GPS;
    pairs[0].satellite = g01;
    pairs[0].variance = 0.1;
    pairs[0].posterior_abs_residual_m = 0.01;
    pairs[1].system = GNSSSystem::GPS;
    pairs[1].satellite = g02;
    pairs[1].variance = 0.1;
    pairs[1].posterior_abs_residual_m = 0.09;
    pairs[2].system = GNSSSystem::GPS;
    pairs[2].satellite = g03;
    pairs[2].variance = 0.1;
    pairs[2].posterior_abs_residual_m = 0.02;

    const auto subsets =
        rtk_ar_selection::buildSatelliteQualityDropSubsets(
            pairs, /*minimum_pairs=*/2, /*max_drop_steps=*/1);
    ASSERT_EQ(subsets.size(), 1U);
    EXPECT_EQ(subsets[0], (std::vector<int>{0, 2}));
}

TEST(RTKArSelectionTest, SatelliteQualityDropPenalizesAzimuthCrowding) {
    std::vector<rtk_ar_selection::PairDescriptor> pairs(4);
    const std::vector<double> azimuths = {0.0, 0.05, 2.0, 4.0};
    for (int index = 0; index < 4; ++index) {
        pairs[index].system = GNSSSystem::GPS;
        pairs[index].satellite =
            SatelliteId(GNSSSystem::GPS, index + 1);
        pairs[index].variance = 0.1;
        pairs[index].azimuth_rad = azimuths[index];
    }

    const auto subsets =
        rtk_ar_selection::buildSatelliteQualityDropSubsets(
            pairs, /*minimum_pairs=*/3, /*max_drop_steps=*/1);
    ASSERT_EQ(subsets.size(), 1U);
    // G01 and G02 are equally crowded; the stable satellite-id tie-break
    // makes G01 the deterministic first drop.
    EXPECT_EQ(subsets[0], (std::vector<int>{1, 2, 3}));
}

TEST(RTKArSelectionTest, SatelliteQualityDiverseDropKeepsDistinctMetricPaths) {
    std::vector<rtk_ar_selection::PairDescriptor> pairs(4);
    for (int index = 0; index < 4; ++index) {
        pairs[index].system = GNSSSystem::GPS;
        pairs[index].satellite =
            SatelliteId(GNSSSystem::GPS, index + 1);
        pairs[index].variance = 0.1;
        pairs[index].posterior_abs_residual_m = 0.01;
        pairs[index].snr_dbhz = 45.0;
    }
    pairs[1].variance = 10.0;
    pairs[2].posterior_abs_residual_m = 0.5;
    pairs[3].snr_dbhz = 15.0;

    const auto subsets =
        rtk_ar_selection::buildSatelliteQualityDiverseDropSubsets(
            pairs, /*minimum_pairs=*/2, /*max_drop_steps=*/1,
            /*maximum_subsets=*/16);

    EXPECT_NE(
        std::find(
            subsets.begin(), subsets.end(),
            std::vector<int>({0, 2, 3})),
        subsets.end());
    EXPECT_NE(
        std::find(
            subsets.begin(), subsets.end(),
            std::vector<int>({0, 1, 3})),
        subsets.end());
    EXPECT_NE(
        std::find(
            subsets.begin(), subsets.end(),
            std::vector<int>({0, 1, 2})),
        subsets.end());
}
