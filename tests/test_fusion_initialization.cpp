#include <gtest/gtest.h>

#include <libgnss++/fusion/fusion_initialization.hpp>

#include <cmath>
#include <limits>

namespace libgnss {
namespace {

std::vector<ImuSample> makeStationaryWindow(const Eigen::Vector3d& up_body_true,
                                            const Eigen::Vector3d& accel_bias_true,
                                            const Eigen::Vector3d& gyro_bias_true, int count,
                                            double gravity_mps2) {
    std::vector<ImuSample> window;
    window.reserve(count);
    GNSSTime time(2100, 500.0);
    for (int i = 0; i < count; ++i) {
        ImuSample sample;
        sample.time = time;
        sample.accel_raw = gravity_mps2 * up_body_true + accel_bias_true;
        sample.gyro_raw_radps = gyro_bias_true;
        window.push_back(sample);
        time = time + 0.01;
    }
    return window;
}

TEST(FusionInitializationTest, AlignStaticRecoversExactTiltAndGyroBiasWithZeroAccelBias) {
    constexpr double kGravity = 9.80665;
    // With zero accel bias, the mean specific force direction is exactly
    // up_body_true, so alignStatic's FromTwoVectors recipe recovers the
    // ground-truth (yaw-free) attitude exactly, not just approximately.
    const Eigen::Vector3d up_body_true = Eigen::Vector3d(0.05, -0.03, 1.0).normalized();
    const Eigen::Quaterniond true_attitude =
        Eigen::Quaterniond::FromTwoVectors(up_body_true, Eigen::Vector3d::UnitZ()).normalized();

    const Eigen::Vector3d accel_bias_true = Eigen::Vector3d::Zero();
    const Eigen::Vector3d gyro_bias_true(0.001, -0.0015, 0.0008);

    const auto window =
        makeStationaryWindow(up_body_true, accel_bias_true, gyro_bias_true, 200, kGravity);

    const NominalState aligned =
        fusion_initialization::alignStatic(window, Eigen::Vector3d(1.0, 2.0, 3.0), kGravity);

    EXPECT_TRUE(aligned.position_enu.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0), 1e-12));
    // Note: Eigen's isApprox() against an exact zero vector requires exact
    // bit-equality (its relative tolerance formula degenerates to
    // `diff.norm() <= 0` when the comparand's norm is zero), so
    // near-but-not-exactly-zero results are compared via norm() instead.
    EXPECT_LT(aligned.velocity_enu.norm(), 1e-12);
    EXPECT_TRUE(aligned.attitude_body_to_enu.isApprox(true_attitude, 1e-9));
    EXPECT_TRUE(aligned.gyro_bias.isApprox(gyro_bias_true, 1e-12));
    EXPECT_LT(aligned.accel_bias.norm(), 1e-9);
}

TEST(FusionInitializationTest, AlignStaticApproximatelyRecoversTiltAndAccelBiasWhenBiasIsNonzero) {
    constexpr double kGravity = 9.80665;
    // With a nonzero accel bias, the *sensed* specific-force direction is no
    // longer exactly up_body_true (the bias perturbs it), so both the
    // recovered attitude and accel_bias are only approximately correct --
    // this is the same coarse-leveling limitation reference_notes.md 4.4
    // documents (gravity is removed only along the *sensed* direction).
    // Gyro bias recovery is still exact (stationary => raw gyro is pure
    // bias, independent of the accel channel).
    const Eigen::Vector3d up_body_true = Eigen::Vector3d(0.05, -0.03, 1.0).normalized();
    const Eigen::Vector3d accel_bias_true(0.01, -0.02, 0.005);
    const Eigen::Vector3d gyro_bias_true(0.001, -0.0015, 0.0008);

    const auto window =
        makeStationaryWindow(up_body_true, accel_bias_true, gyro_bias_true, 200, kGravity);

    const NominalState aligned =
        fusion_initialization::alignStatic(window, Eigen::Vector3d(1.0, 2.0, 3.0), kGravity);

    EXPECT_TRUE(aligned.gyro_bias.isApprox(gyro_bias_true, 1e-12));
    // Coarse leveling only removes gravity along the *sensed* direction, so
    // for a bias not well-aligned with up_body_true it may not reduce the
    // error by much -- this is a documented limitation (reference_notes.md
    // 4.4), not a bug. Sanity-bound the result well above the observed
    // error so this remains a regression check against gross breakage
    // (e.g. the correction increasing the error, or diverging) rather than
    // an unrealistic precision claim.
    EXPECT_LT((aligned.accel_bias - accel_bias_true).norm(), 0.03);
    EXPECT_LT(aligned.accel_bias.norm(), 0.05);
}

TEST(FusionInitializationTest, AlignStaticHandlesEmptyWindow) {
    const NominalState aligned =
        fusion_initialization::alignStatic({}, Eigen::Vector3d(4.0, 5.0, 6.0), 9.80665);
    EXPECT_TRUE(aligned.position_enu.isApprox(Eigen::Vector3d(4.0, 5.0, 6.0), 1e-12));
}

TEST(FusionInitializationTest, VelocityToRpyMatchesUpstreamOffsetAndFiniteOutput) {
    const std::vector<Eigen::Vector3d> velocities = {
        Eigen::Vector3d(0.0, 2.0, 0.0),  // North: 90 deg course + 180 = -90 deg
        Eigen::Vector3d(2.0, 0.0, 0.0),  // East
    };
    fusion_initialization::VelocityHeadingConfig config;
    config.smooth_window = 1;
    const auto result = fusion_initialization::velocityToRpy(velocities, config);

    ASSERT_TRUE(result.ok) << result.error;
    ASSERT_EQ(result.rpy_rad.size(), velocities.size());
    EXPECT_NEAR(result.rpy_rad[0].x(), 0.0, 1e-12);
    EXPECT_NEAR(result.rpy_rad[0].y(), 0.0, 1e-12);
    EXPECT_NEAR(result.rpy_rad[0].z(), -M_PI / 2.0, 1e-12);
    EXPECT_NEAR(result.rpy_rad[1].z(), M_PI, 1e-12);
    for (const auto& rpy : result.rpy_rad) EXPECT_TRUE(rpy.allFinite());
}

TEST(FusionInitializationTest, VelocityToRpyUsesMatlabEvenWindowAndShrunkEndpoints) {
    std::vector<Eigen::Vector3d> velocities;
    for (double east = 1.0; east <= 5.0; east += 1.0) {
        velocities.emplace_back(east, 1.0, 0.0);
    }
    fusion_initialization::VelocityHeadingConfig config;
    config.smooth_window = 4;
    const auto result = fusion_initialization::velocityToRpy(velocities, config);

    ASSERT_TRUE(result.ok) << result.error;
    ASSERT_EQ(result.smoothed_velocity_enu.size(), 5u);
    // MATLAB movmean centers an even window about the current and previous
    // elements: [1,2] at the leading endpoint and [3,4,5] as the trailing
    // three available samples.
    EXPECT_NEAR(result.smoothed_velocity_enu[0].x(), 1.5, 1e-12);
    EXPECT_NEAR(result.smoothed_velocity_enu[1].x(), 2.0, 1e-12);
    EXPECT_NEAR(result.smoothed_velocity_enu[2].x(), 2.5, 1e-12);
    EXPECT_NEAR(result.smoothed_velocity_enu[3].x(), 3.5, 1e-12);
    EXPECT_NEAR(result.smoothed_velocity_enu[4].x(), 4.0, 1e-12);
}

TEST(FusionInitializationTest, VelocityToRpyLinearFillsInteriorAndNearestFillsEndpoints) {
    const std::vector<Eigen::Vector3d> velocities = {
        Eigen::Vector3d::Zero(),            // leading endpoint gap
        Eigen::Vector3d(2.0, 0.0, 0.0),     // course 0 deg
        Eigen::Vector3d::Zero(),            // interior gap
        Eigen::Vector3d(0.0, 2.0, 0.0),     // course 90 deg
        Eigen::Vector3d::Zero(),            // trailing endpoint gap
    };
    fusion_initialization::VelocityHeadingConfig config;
    config.smooth_window = 1;
    config.velocity_threshold_mps = 0.5;
    const auto result = fusion_initialization::velocityToRpy(velocities, config);

    ASSERT_TRUE(result.ok) << result.error;
    EXPECT_EQ(result.low_speed_count, 3u);
    EXPECT_EQ(result.linear_fill_count, 1u);
    EXPECT_EQ(result.nearest_fill_count, 2u);
    // Courses are filled before +180/wrap: the interior 45-degree course
    // becomes -135 degrees, while both edge gaps use their nearest course.
    EXPECT_NEAR(result.rpy_rad[0].z(), M_PI, 1e-12);
    EXPECT_NEAR(result.rpy_rad[2].z(), -3.0 * M_PI / 4.0, 1e-12);
    EXPECT_NEAR(result.rpy_rad[4].z(), -M_PI / 2.0, 1e-12);
}

TEST(FusionInitializationTest, VelocityToRpyInterpolatesRawCourseAcrossBranchBeforeWrap) {
    const double one_degree = M_PI / 180.0;
    const std::vector<Eigen::Vector3d> velocities = {
        Eigen::Vector3d(-std::cos(one_degree), std::sin(one_degree), 0.0),
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d(-std::cos(one_degree), -std::sin(one_degree), 0.0),
    };
    fusion_initialization::VelocityHeadingConfig config;
    config.smooth_window = 1;
    const auto result = fusion_initialization::velocityToRpy(velocities, config);

    ASSERT_TRUE(result.ok) << result.error;
    // +179 and -179 are interpolated as raw atan2 degrees through 0, then
    // offset by 180 and wrapped, yielding the +180 boundary at the gap.
    EXPECT_NEAR(result.rpy_rad[1].z(), M_PI, 1e-12);
}

TEST(FusionInitializationTest, VelocityToRpyFailsClosedForAllLowSpeedOrNonFiniteInput) {
    fusion_initialization::VelocityHeadingConfig config;
    config.smooth_window = 1;
    auto all_low = fusion_initialization::velocityToRpy(
        {Eigen::Vector3d::Zero(), Eigen::Vector3d(0.1, 0.0, 0.0)}, config);
    EXPECT_FALSE(all_low.ok);
    EXPECT_FALSE(all_low.error.empty());

    auto nonfinite = fusion_initialization::velocityToRpy(
        {Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0)}, config);
    EXPECT_FALSE(nonfinite.ok);
    EXPECT_FALSE(nonfinite.error.empty());
}

TEST(FusionInitializationTest, TryAlignHeadingBelowSpeedThresholdDoesNothing) {
    FusionState state;
    state.nominal.attitude_body_to_enu = Eigen::Quaterniond::Identity();
    const bool applied = fusion_initialization::tryAlignHeading(
        state, Eigen::Vector3d(0.2, 0.1, 0.0), /*min_speed_mps=*/1.0, /*heading_sigma_deg=*/5.0);
    EXPECT_FALSE(applied);
    EXPECT_TRUE(state.nominal.attitude_body_to_enu.isApprox(Eigen::Quaterniond::Identity(), 1e-12));
}

TEST(FusionInitializationTest, TryAlignHeadingRecoversTrueCourseAboveThreshold) {
    FusionState state;
    state.nominal.attitude_body_to_enu = Eigen::Quaterniond::Identity();
    state.covariance = Eigen::Matrix<double, 15, 15>::Identity();

    const Eigen::Vector3d gnss_velocity_enu(3.0, 4.0, 0.0);  // speed = 5 m/s
    const bool applied = fusion_initialization::tryAlignHeading(
        state, gnss_velocity_enu, /*min_speed_mps=*/1.0, /*heading_sigma_deg=*/5.0);
    ASSERT_TRUE(applied);

    const Eigen::Vector3d forward_enu = state.nominal.attitude_body_to_enu * Eigen::Vector3d::UnitX();
    const double resulting_heading = std::atan2(forward_enu.x(), forward_enu.y());
    const double target_heading = std::atan2(gnss_velocity_enu.x(), gnss_velocity_enu.y());
    EXPECT_NEAR(resulting_heading, target_heading, 1e-9);

    const double expected_yaw_sigma_rad = 5.0 * M_PI / 180.0;
    EXPECT_NEAR(state.covariance(fusion_index::ATTITUDE + 2, fusion_index::ATTITUDE + 2),
               expected_yaw_sigma_rad * expected_yaw_sigma_rad, 1e-12);
}

TEST(FusionInitializationTest, TryAlignHeadingCorrectsExistingYawTowardTrueCourse) {
    FusionState state;
    // Start with a deliberately wrong heading (attitude points North, i.e.
    // 0 deg heading) while GNSS course is due East (90 deg heading).
    state.nominal.attitude_body_to_enu = Eigen::Quaterniond::Identity();
    state.covariance = Eigen::Matrix<double, 15, 15>::Identity();

    const Eigen::Vector3d gnss_velocity_enu(10.0, 0.0, 0.0);  // due East
    ASSERT_TRUE(fusion_initialization::tryAlignHeading(state, gnss_velocity_enu, 1.0, 5.0));

    const Eigen::Vector3d forward_enu = state.nominal.attitude_body_to_enu * Eigen::Vector3d::UnitX();
    // Forward should now point East (1,0,0), matching the GNSS course.
    EXPECT_TRUE(forward_enu.isApprox(Eigen::Vector3d(1.0, 0.0, 0.0), 1e-9));
}

// --- HeadingAlignmentTracker ------------------------------------------------
//
// Regression tests for the PPC nagoya/run1 root cause: the old
// tryAlignHeading() latched on a *single* GNSS-velocity epoch above
// min_speed_mps. On nagoya/run1 that first qualifying epoch happened while
// the vehicle was backing out of a parking space -- GNSS course-over-ground
// tracked the direction of travel (roughly opposite the vehicle's true nose
// heading during reverse), so the filter latched a ~180 deg-wrong yaw with a
// tight sigma and never recovered. These tests exercise the new
// multi-epoch-consistency gate in isolation.

TEST(HeadingAlignmentTrackerTest, NotReadyBelowMinSamples) {
    fusion_initialization::HeadingAlignmentTracker tracker;
    GNSSTime time(2100, 500.0);
    // Two consistent, above-threshold samples -- one short of the default
    // min_samples=3 used by the caller (LooseCouplingProcessor).
    tracker.addSample(time, Eigen::Vector3d(5.0, 0.0, 0.0), 1.0, 5.0);
    time = time + 1.0;
    tracker.addSample(time, Eigen::Vector3d(5.0, 0.0, 0.0), 1.0, 5.0);

    double mean_course = 0.0, scatter_deg = 0.0;
    EXPECT_EQ(tracker.sampleCount(), 2u);
    EXPECT_FALSE(tracker.ready(/*min_samples=*/3, /*max_scatter_deg=*/10.0, &mean_course, &scatter_deg));
}

TEST(HeadingAlignmentTrackerTest, DropsSamplesBelowSpeedThreshold) {
    fusion_initialization::HeadingAlignmentTracker tracker;
    GNSSTime time(2100, 500.0);
    // Below min_speed_mps=1.0 -- must not be buffered at all.
    tracker.addSample(time, Eigen::Vector3d(0.2, 0.1, 0.0), 1.0, 5.0);
    EXPECT_EQ(tracker.sampleCount(), 0u);
}

TEST(HeadingAlignmentTrackerTest, ReadyWithConsistentCourseRecoversMeanAndSmallScatter) {
    fusion_initialization::HeadingAlignmentTracker tracker;
    GNSSTime time(2100, 500.0);
    // Due-East course (atan2(x=5,y=0) = 90 deg heading) with small jitter
    // across several samples.
    const std::vector<Eigen::Vector3d> velocities = {
        Eigen::Vector3d(5.0, 0.05, 0.0),
        Eigen::Vector3d(5.0, -0.05, 0.0),
        Eigen::Vector3d(5.0, 0.02, 0.0),
        Eigen::Vector3d(5.0, -0.02, 0.0),
    };
    for (const auto& v : velocities) {
        tracker.addSample(time, v, 1.0, 5.0);
        time = time + 1.0;
    }

    double mean_course_rad = 0.0, scatter_deg = 0.0;
    ASSERT_TRUE(tracker.ready(3, 10.0, &mean_course_rad, &scatter_deg));
    EXPECT_NEAR(mean_course_rad, 90.0 * M_PI / 180.0, 2.0 * M_PI / 180.0);
    EXPECT_LT(scatter_deg, 2.0);
}

TEST(HeadingAlignmentTrackerTest, NotReadyWithScatteredCourseEvenWithEnoughSamples) {
    fusion_initialization::HeadingAlignmentTracker tracker;
    GNSSTime time(2100, 500.0);
    // Courses spread over ~60 deg -- e.g. a vehicle still maneuvering/turning
    // rather than moving in one settled direction. Enough samples, but the
    // scatter gate must still block the latch.
    const std::vector<double> course_deg = {60.0, 90.0, 120.0, 30.0, 150.0};
    for (double deg : course_deg) {
        const double rad = deg * M_PI / 180.0;
        tracker.addSample(time, Eigen::Vector3d(5.0 * std::sin(rad), 5.0 * std::cos(rad), 0.0), 1.0, 10.0);
        time = time + 1.0;
    }

    double mean_course_rad = 0.0, scatter_deg = 0.0;
    EXPECT_FALSE(tracker.ready(3, 10.0, &mean_course_rad, &scatter_deg));
    EXPECT_GT(scatter_deg, 10.0);
}

TEST(HeadingAlignmentTrackerTest, DropsSamplesOutsideTrailingWindow) {
    fusion_initialization::HeadingAlignmentTracker tracker;
    GNSSTime time(2100, 500.0);
    // Two old samples, then a big time gap, then one recent sample -- with a
    // 5 s window the old ones must have fallen out by the time the third is
    // added, leaving too few samples to be ready.
    tracker.addSample(time, Eigen::Vector3d(5.0, 0.0, 0.0), 1.0, 5.0);
    time = time + 1.0;
    tracker.addSample(time, Eigen::Vector3d(5.0, 0.0, 0.0), 1.0, 5.0);
    time = time + 20.0;  // well outside the 5 s window
    tracker.addSample(time, Eigen::Vector3d(5.0, 0.0, 0.0), 1.0, 5.0);

    EXPECT_EQ(tracker.sampleCount(), 1u);
    double mean_course_rad = 0.0, scatter_deg = 0.0;
    EXPECT_FALSE(tracker.ready(3, 10.0, &mean_course_rad, &scatter_deg));
}

TEST(HeadingAlignmentTrackerTest, ResetClearsBuffer) {
    fusion_initialization::HeadingAlignmentTracker tracker;
    GNSSTime time(2100, 500.0);
    tracker.addSample(time, Eigen::Vector3d(5.0, 0.0, 0.0), 1.0, 5.0);
    time = time + 1.0;
    tracker.addSample(time, Eigen::Vector3d(5.0, 0.0, 0.0), 1.0, 5.0);
    ASSERT_EQ(tracker.sampleCount(), 2u);

    tracker.reset();
    EXPECT_EQ(tracker.sampleCount(), 0u);
}

// A sustained-but-wrong course (e.g. the whole "reverse out of parking"
// window) is, by design, just as internally consistent as a correct one --
// the scatter check alone cannot distinguish them. This documents that
// limitation explicitly: LooseCouplingProcessor's post-latch NIS-based
// recovery (fusion_processor.hpp Config::heading_recovery_*, exercised in
// test_fusion_processor_synthetic.cpp) is what catches this case once real
// motion disagrees with the wrong latch.
TEST(HeadingAlignmentTrackerTest, ConsistentCourseIsReadyEvenIfItWouldBePhysicallyWrong) {
    fusion_initialization::HeadingAlignmentTracker tracker;
    GNSSTime time(2100, 500.0);
    for (int i = 0; i < 4; ++i) {
        tracker.addSample(time, Eigen::Vector3d(0.0, -5.0, 0.0), 1.0, 5.0);  // consistently "South"
        time = time + 1.0;
    }
    double mean_course_rad = 0.0, scatter_deg = 0.0;
    ASSERT_TRUE(tracker.ready(3, 10.0, &mean_course_rad, &scatter_deg));
    EXPECT_NEAR(mean_course_rad, M_PI, 2.0 * M_PI / 180.0);
}

}  // namespace
}  // namespace libgnss
