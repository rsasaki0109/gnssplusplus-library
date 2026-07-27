#include <gtest/gtest.h>

#include <libgnss++/algorithms/rtk_adaptive_noise.hpp>

#include <Eigen/Dense>

#include <cmath>
#include <vector>

namespace libgnss {
namespace {

using rtk_adaptive_noise::AdaptiveNoiseConfig;
using rtk_adaptive_noise::AdaptiveNoiseTracker;
using rtk_measurement::MeasurementKind;

AdaptiveNoiseConfig wideOpenConfig() {
    AdaptiveNoiseConfig config;
    config.min_variance_scale = 1e-6;
    config.max_variance_scale = 1e6;
    return config;
}

TEST(AdaptiveNoiseTrackerTest, ReturnsModelVarianceWithoutHistory) {
    const AdaptiveNoiseTracker tracker;
    const AdaptiveNoiseConfig config;
    EXPECT_DOUBLE_EQ(tracker.adaptedVariance(7, MeasurementKind::CODE, 0.36, config), 0.36);
    EXPECT_EQ(tracker.size(), 0u);
}

TEST(AdaptiveNoiseTrackerTest, EwmaConvergesToInnovationImpliedVariance) {
    AdaptiveNoiseTracker tracker;
    const AdaptiveNoiseConfig config = wideOpenConfig();
    const double model_variance = 0.09;
    const double hph = 0.02;
    const double ref_var = 0.01;
    const double innovation = 1.0;  // sample = 1.0 - 0.02 - 0.01 = 0.97
    const double expected_sample = innovation * innovation - hph - ref_var;

    for (int epoch = 0; epoch < 200; ++epoch) {
        tracker.update(3, MeasurementKind::CODE, innovation, hph, ref_var, model_variance,
                       static_cast<double>(epoch), config);
    }
    EXPECT_NEAR(tracker.adaptedVariance(3, MeasurementKind::CODE, model_variance, config),
                expected_sample, 1e-9);
}

TEST(AdaptiveNoiseTrackerTest, ConvergenceRateFollowsAlpha) {
    AdaptiveNoiseTracker tracker;
    AdaptiveNoiseConfig config = wideOpenConfig();
    config.alpha_code = 0.5;
    const double model_variance = 1.0;
    // One update from a fresh entry: 0.5 * model + 0.5 * sample.
    tracker.update(1, MeasurementKind::CODE, 3.0, 0.0, 0.0, model_variance, 0.0, config);
    EXPECT_NEAR(tracker.adaptedVariance(1, MeasurementKind::CODE, model_variance, config),
                0.5 * 1.0 + 0.5 * 9.0, 1e-12);

    AdaptiveNoiseTracker slow_tracker;
    config.alpha_phase = 0.9;
    slow_tracker.update(1, MeasurementKind::PHASE, 3.0, 0.0, 0.0, model_variance, 0.0, config);
    EXPECT_NEAR(slow_tracker.adaptedVariance(1, MeasurementKind::PHASE, model_variance, config),
                0.9 * 1.0 + 0.1 * 9.0, 1e-12);
}

TEST(AdaptiveNoiseTrackerTest, NegativeSampleClampsToFloor) {
    AdaptiveNoiseTracker tracker;
    AdaptiveNoiseConfig config;
    config.min_variance_scale = 0.25;
    config.max_variance_scale = 25.0;
    const double model_variance = 0.04;
    // Tiny innovation with large prior HPH makes the sample strongly negative.
    for (int epoch = 0; epoch < 50; ++epoch) {
        tracker.update(5, MeasurementKind::CODE, 0.001, 0.5, 0.1, model_variance,
                       static_cast<double>(epoch), config);
    }
    const double adapted =
        tracker.adaptedVariance(5, MeasurementKind::CODE, model_variance, config);
    EXPECT_GT(adapted, 0.0);
    EXPECT_DOUBLE_EQ(adapted, config.min_variance_scale * model_variance);
}

TEST(AdaptiveNoiseTrackerTest, LargeSampleClampsToCeiling) {
    AdaptiveNoiseTracker tracker;
    AdaptiveNoiseConfig config;
    config.min_variance_scale = 0.25;
    config.max_variance_scale = 25.0;
    const double model_variance = 0.04;
    for (int epoch = 0; epoch < 50; ++epoch) {
        tracker.update(5, MeasurementKind::CODE, 100.0, 0.0, 0.0, model_variance,
                       static_cast<double>(epoch), config);
    }
    EXPECT_DOUBLE_EQ(tracker.adaptedVariance(5, MeasurementKind::CODE, model_variance, config),
                     config.max_variance_scale * model_variance);
}

TEST(AdaptiveNoiseTrackerTest, ClampTracksModelVariance) {
    AdaptiveNoiseTracker tracker;
    AdaptiveNoiseConfig config;
    config.min_variance_scale = 0.25;
    config.max_variance_scale = 25.0;
    // Learn a large variance against a small model variance.
    tracker.update(2, MeasurementKind::CODE, 10.0, 0.0, 0.0, 0.04, 0.0, config);
    const double adapted_small =
        tracker.adaptedVariance(2, MeasurementKind::CODE, 0.04, config);
    EXPECT_DOUBLE_EQ(adapted_small, 25.0 * 0.04);
    // The same stored entry queried against a bigger model variance moves the
    // clamp window: the stored value now sits inside it.
    const double adapted_large =
        tracker.adaptedVariance(2, MeasurementKind::CODE, 10.0, config);
    EXPECT_GE(adapted_large, 0.25 * 10.0);
    EXPECT_LE(adapted_large, 25.0 * 10.0);
}

TEST(AdaptiveNoiseTrackerTest, SlipResetsPhaseOnlyAndOutagePrunes) {
    AdaptiveNoiseTracker tracker;
    const AdaptiveNoiseConfig config = wideOpenConfig();
    tracker.update(4, MeasurementKind::PHASE, 0.5, 0.0, 0.0, 1.0, 100.0, config);
    tracker.update(4, MeasurementKind::CODE, 0.5, 0.0, 0.0, 1.0, 100.0, config);
    ASSERT_EQ(tracker.size(), 2u);

    tracker.resetKey(4, MeasurementKind::PHASE);
    EXPECT_EQ(tracker.size(), 1u);
    // The phase entry is gone; the code entry survives the slip.
    EXPECT_DOUBLE_EQ(tracker.adaptedVariance(4, MeasurementKind::PHASE, 1.0, config), 1.0);
    EXPECT_NE(tracker.adaptedVariance(4, MeasurementKind::CODE, 1.0, config), 1.0);

    tracker.pruneStale(103.0, 5.0);
    EXPECT_EQ(tracker.size(), 1u);
    tracker.pruneStale(106.0, 5.0);
    EXPECT_EQ(tracker.size(), 0u);
}

TEST(AdaptiveNoiseTrackerTest, ClearRemovesEverything) {
    AdaptiveNoiseTracker tracker;
    const AdaptiveNoiseConfig config = wideOpenConfig();
    tracker.update(1, MeasurementKind::PHASE, 0.5, 0.0, 0.0, 1.0, 0.0, config);
    tracker.update(2, MeasurementKind::CODE, 0.5, 0.0, 0.0, 1.0, 0.0, config);
    tracker.clear();
    EXPECT_EQ(tracker.size(), 0u);
}

TEST(AdaptiveNoiseTrackerTest, IgnoresNonFiniteAndNonPositiveInputs) {
    AdaptiveNoiseTracker tracker;
    const AdaptiveNoiseConfig config = wideOpenConfig();
    tracker.update(1, MeasurementKind::CODE, std::nan(""), 0.0, 0.0, 1.0, 0.0, config);
    tracker.update(1, MeasurementKind::CODE, 0.5, std::nan(""), 0.0, 1.0, 0.0, config);
    tracker.update(1, MeasurementKind::CODE, 0.5, 0.0, 0.0, 0.0, 0.0, config);
    tracker.update(1, MeasurementKind::CODE, 0.5, 0.0, 0.0, -1.0, 0.0, config);
    EXPECT_EQ(tracker.size(), 0u);
}

TEST(AdaptiveNoiseTrackerTest, MeanVarianceScaleReportsPerKind) {
    AdaptiveNoiseTracker tracker;
    AdaptiveNoiseConfig config;
    config.min_variance_scale = 0.25;
    config.max_variance_scale = 25.0;
    EXPECT_DOUBLE_EQ(tracker.meanVarianceScale(MeasurementKind::PHASE), 1.0);
    // sample = 4, alpha_code 0.5, fresh entry from model 1.0 -> 2.5.
    tracker.update(1, MeasurementKind::CODE, 2.0, 0.0, 0.0, 1.0, 0.0, config);
    EXPECT_NEAR(tracker.meanVarianceScale(MeasurementKind::CODE), 2.5, 1e-12);
    EXPECT_DOUBLE_EQ(tracker.meanVarianceScale(MeasurementKind::PHASE), 1.0);
}

TEST(AdaptiveNoiseTrackerTest, AssembledDdCovarianceStaysPositiveDefinite) {
    // Adapted satellite variances stay >= floor > 0; combined with the
    // ref_var * 11' + diag(sat_var) block structure the assembled DD
    // covariance must remain positive definite.
    AdaptiveNoiseTracker tracker;
    AdaptiveNoiseConfig config;
    config.min_variance_scale = 0.25;
    config.max_variance_scale = 25.0;
    const double model_variance = 0.09;
    std::vector<double> sat_variances;
    // Feed wildly different innovation streams to a handful of keys.
    const double innovations[] = {0.001, 5.0, 0.3, 12.0, 0.05, 2.0};
    for (int key = 0; key < 6; ++key) {
        for (int epoch = 0; epoch < 20; ++epoch) {
            tracker.update(key, MeasurementKind::CODE, innovations[key], 0.05, 0.02,
                           model_variance, static_cast<double>(epoch), config);
        }
        sat_variances.push_back(
            tracker.adaptedVariance(key, MeasurementKind::CODE, model_variance, config));
    }

    const std::vector<int> block_sizes{3, 3};
    const std::vector<double> ref_variances(6, model_variance);
    const Eigen::MatrixXd covariance = rtk_measurement::buildDoubleDifferenceCovariance(
        block_sizes, ref_variances, sat_variances, 6);

    const Eigen::LDLT<Eigen::MatrixXd> ldlt(covariance);
    ASSERT_EQ(ldlt.info(), Eigen::Success);
    EXPECT_GT(ldlt.vectorD().minCoeff(), 0.0);
}

}  // namespace
}  // namespace libgnss
