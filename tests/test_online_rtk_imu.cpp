#include <gtest/gtest.h>
#include <libgnss++/fusion/online_rtk_imu.hpp>
#include <libgnss++/fusion/attitude.hpp>
#include <limits>
#include <stdexcept>

using namespace libgnss;
namespace {
OnlineRtkImuProcessor::Config configuration() {
    OnlineRtkImuProcessor::Config config;
    config.base_position_ecef = Vector3d(6378137.0, 0.0, 0.0);
    config.fusion.align_static_window_s = 0.02;
    config.fusion.zupt_enable = false;
    return config;
}
GNSSTime time(double tow) { return GNSSTime(2200, tow); }
ObservationData epoch(double tow) { return ObservationData(time(tow)); }
ImuSample imu(double tow) {
    ImuSample sample;
    sample.time = time(tow);
    sample.accel_raw = Vector3d(0.0, 0.0, 9.80665);
    return sample;
}
TEST(OnlineRtkImuTest, FutureSourcesAndBackwardReceptionAreRejectedWithoutQueueMutation) {
    OnlineRtkImuProcessor processor(configuration());
    processor.pushBase(epoch(10.0), time(10.0));
    EXPECT_THROW(processor.pushBase(epoch(11.0), time(10.0)), std::invalid_argument);
    EXPECT_THROW(processor.pushImu(imu(11.0), time(10.0)), std::invalid_argument);
    EXPECT_THROW(processor.processRover(epoch(11.0), time(10.0)), std::invalid_argument);
    EXPECT_THROW(processor.pushNavigation(NavigationData{}, time(9.0)), std::invalid_argument);
    EXPECT_EQ(processor.pendingBase(), 1U);
    EXPECT_EQ(processor.pendingImu(), 0U);
    EXPECT_EQ(processor.diagnostics().rover_epochs, 0U);
}
TEST(OnlineRtkImuTest, ReceivedFutureBaseCannotInterpolateEarlierRover) {
    OnlineRtkImuProcessor processor(configuration());
    processor.pushBase(epoch(10.0), time(10.0));
    processor.pushBase(epoch(11.0), time(11.0));
    const auto first = processor.processRover(epoch(10.2), time(11.0));
    EXPECT_FALSE(first.exact_base_available);
    EXPECT_EQ(first.reason, "missing_exact_base");
    EXPECT_EQ(processor.pendingBase(), 1U);
    EXPECT_EQ(processor.diagnostics().expired_base_epochs, 1U);
    const auto second = processor.processRover(epoch(11.0), time(11.0));
    EXPECT_TRUE(second.exact_base_available);
    EXPECT_EQ(processor.pendingBase(), 0U);
}
TEST(OnlineRtkImuTest, FutureImuStaysQueuedUntilItsEpoch) {
    OnlineRtkImuProcessor processor(configuration());
    for (double t : {10.0, 10.01, 10.02, 10.03}) processor.pushImu(imu(t), time(10.03));
    const auto first = processor.processRover(epoch(10.02), time(10.03));
    EXPECT_EQ(first.imu_consumed, 3U);
    EXPECT_EQ(processor.pendingImu(), 1U);
    EXPECT_FALSE(first.fused.isValid());
    EXPECT_FALSE(first.attitude_available);
    EXPECT_FALSE(first.attitude_body_to_enu.coeffs().allFinite());
    EXPECT_FALSE(first.ecef_to_attitude_enu.allFinite());
    const auto second = processor.processRover(epoch(10.03), time(10.03));
    EXPECT_EQ(second.imu_consumed, 1U);
    EXPECT_EQ(processor.pendingImu(), 0U);
}
TEST(OnlineRtkImuTest, LateBaseAndImuCannotReviseAnEmittedEpoch) {
    OnlineRtkImuProcessor processor(configuration());
    const auto first = processor.processRover(epoch(10.0), time(10.1));
    processor.pushBase(epoch(10.0), time(10.2));
    EXPECT_FALSE(processor.pushImu(imu(10.0), time(10.2)));
    EXPECT_EQ(processor.pendingBase(), 0U);
    EXPECT_EQ(processor.pendingImu(), 0U);
    EXPECT_EQ(processor.diagnostics().expired_base_epochs, 1U);
    EXPECT_EQ(processor.diagnostics().late_imu_dropped, 1U);
    EXPECT_FALSE(first.exact_base_available);
    EXPECT_DOUBLE_EQ(first.input_age_s, 10.1 - 10.0);
    EXPECT_THROW(processor.processRover(epoch(10.0), time(10.2)), std::invalid_argument);
}
TEST(OnlineRtkImuTest, GapReinitializationDoesNotReuseAlignmentOrTimeUpdate) {
    OnlineRtkImuProcessor processor(configuration());
    for (double t : {10.0, 10.01, 10.02, 10.03}) processor.pushImu(imu(t), time(t));
    processor.processRover(epoch(10.03), time(10.03));
    processor.pushImu(imu(10.5), time(10.5));
    const auto after_gap = processor.processRover(epoch(10.5), time(10.5));
    EXPECT_EQ(after_gap.reset_generation, 1U);
    EXPECT_EQ(after_gap.reason, "imu_gap_reset");
    EXPECT_FALSE(after_gap.fusion_initialized);
    EXPECT_FALSE(after_gap.fused.isValid());
    EXPECT_FALSE(after_gap.attitude_available);
    EXPECT_FALSE(after_gap.heading_aligned);
    EXPECT_FALSE(after_gap.rpy_frd_ned_deg.allFinite());
    EXPECT_FALSE(after_gap.tight_time_update_supplied);
    EXPECT_EQ(processor.diagnostics().imu_gap_resets, 1U);
}
TEST(OnlineRtkImuTest, AttitudeConventionMatchesIndependentAerospaceRotations) {
    Matrix3d enu_to_ned;
    enu_to_ned << 0, 1, 0, 1, 0, 0, 0, 0, -1;
    const Matrix3d flu_to_frd = Vector3d(1, -1, -1).asDiagonal();
    for (const Vector3d rpy : {Vector3d(0, 0, 0), Vector3d(17, -23, 359), Vector3d(-45, 35, 90)}) {
        const double rad = std::acos(-1.0) / 180.0;
        const Matrix3d body_frd_to_ned = (Eigen::AngleAxisd(rpy.z()*rad, Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rpy.y()*rad, Vector3d::UnitY()) *
            Eigen::AngleAxisd(rpy.x()*rad, Vector3d::UnitX())).toRotationMatrix();
        Eigen::Quaterniond q(enu_to_ned.transpose() * body_frd_to_ned * flu_to_frd);
        auto actual = attitude::fluEnuToFrdNedRpyDegrees(q);
        EXPECT_NEAR(actual.x(), rpy.x(), 1e-10);
        EXPECT_NEAR(actual.y(), rpy.y(), 1e-10);
        EXPECT_NEAR(std::remainder(actual.z()-rpy.z(), 360.0), 0.0, 1e-10);
        q.coeffs() *= -1;
        actual = attitude::fluEnuToFrdNedRpyDegrees(q);
        EXPECT_NEAR(actual.x(), rpy.x(), 1e-10);
        EXPECT_NEAR(actual.y(), rpy.y(), 1e-10);
        EXPECT_NEAR(std::remainder(actual.z()-rpy.z(), 360.0), 0.0, 1e-10);
    }
}
TEST(OnlineRtkImuTest, StaleImuAndRoverOutageResetOncePerDiscontinuity) {
    OnlineRtkImuProcessor processor(configuration());
    processor.pushImu(imu(10.0), time(10.0));
    processor.processRover(epoch(10.0), time(10.0));
    const auto stale = processor.processRover(epoch(10.2), time(10.2));
    EXPECT_EQ(stale.reason, "imu_stale_reset");
    EXPECT_EQ(stale.reset_generation, 1U);
    processor.processRover(epoch(10.4), time(10.4));
    EXPECT_EQ(processor.diagnostics().imu_gap_resets, 1U);
    const auto outage = processor.processRover(epoch(14.0), time(14.5));
    EXPECT_EQ(outage.reason, "rover_gap_reset");
    EXPECT_EQ(outage.reset_generation, 2U);
    EXPECT_EQ(processor.diagnostics().rover_gap_resets, 1U);
    EXPECT_DOUBLE_EQ(outage.input_age_s, 0.5);
}
TEST(OnlineRtkImuTest, CapacityAndInvalidImuFailBeforeAcceptance) {
    auto config = configuration();
    config.max_pending_imu = 1;
    config.max_pending_base = 1;
    OnlineRtkImuProcessor processor(config);
    auto invalid = imu(10.0);
    invalid.gyro_raw_radps.x() = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(processor.pushImu(invalid, time(10.0)), std::invalid_argument);
    EXPECT_EQ(processor.pendingImu(), 0U);
    processor.pushImu(imu(10.0), time(10.0));
    EXPECT_THROW(processor.pushImu(imu(10.01), time(10.01)), std::length_error);
    processor.pushBase(epoch(10.0), time(10.0));
    EXPECT_THROW(processor.pushBase(epoch(10.0), time(10.0)), std::invalid_argument);
    EXPECT_THROW(processor.pushBase(epoch(10.01), time(10.01)), std::length_error);
}
TEST(OnlineRtkImuTest, ResetClearsPendingDataAndPreservesReceptionWatermark) {
    OnlineRtkImuProcessor processor(configuration());
    processor.pushImu(imu(10.0), time(10.0));
    processor.pushBase(epoch(10.0), time(10.0));
    processor.reset(time(10.1));
    EXPECT_EQ(processor.pendingBase(), 0U);
    EXPECT_EQ(processor.pendingImu(), 0U);
    EXPECT_EQ(processor.diagnostics().reset_generation, 1U);
    EXPECT_THROW(processor.pushBase(epoch(10.0), time(10.0)), std::invalid_argument);
    processor.pushBase(epoch(10.0), time(10.1));
    EXPECT_TRUE(processor.processRover(epoch(10.0), time(10.1)).exact_base_available);
}
TEST(OnlineRtkImuTest, SamePrefixIsUnaffectedByDifferentSuffixes) {
    OnlineRtkImuProcessor left(configuration()), right(configuration());
    for (double t : {10.0, 10.01, 10.02}) {
        left.pushImu(imu(t), time(t));
        right.pushImu(imu(t), time(t));
    }
    const auto a = left.processRover(epoch(10.02), time(10.04));
    const auto b = right.processRover(epoch(10.02), time(10.04));
    left.pushBase(epoch(11.0), time(11.0));
    right.reset(time(11.0));
    EXPECT_EQ(a.imu_consumed, b.imu_consumed);
    EXPECT_EQ(a.reset_generation, b.reset_generation);
    EXPECT_EQ(a.exact_base_available, b.exact_base_available);
    EXPECT_EQ(a.rtk.status, b.rtk.status);
    EXPECT_EQ(a.fused.status, b.fused.status);
    EXPECT_EQ(a.reason, b.reason);
    EXPECT_DOUBLE_EQ(a.input_age_s, b.input_age_s);
    // Position parity on usable solutions additionally needs raw PPC replay.
}
TEST(OnlineRtkImuTest, OptInVehicleConstraintWaitsForObservedHeading) {
    auto control_config = configuration().fusion;
    control_config.lever_arm_body.setZero();
    auto candidate_config = control_config;
    candidate_config.nhc_enable = true;
    candidate_config.nhc_require_heading_alignment = true;
    LooseCouplingProcessor control(control_config), candidate(candidate_config);
    for (double t : {10.0, 10.01, 10.02, 10.03}) {
        control.processImuSample(imu(t));
        candidate.processImuSample(imu(t));
        EXPECT_NEAR((control.state().covariance-candidate.state().covariance).norm(), 0, 1e-12);
    }
    for (double t : {10.04, 10.05, 10.06}) {
        control.processImuSample(imu(t));
        candidate.processImuSample(imu(t));
        PositionSolution fix;
        fix.time = time(t);
        fix.status = SolutionStatus::SPP;
        fix.num_satellites = 8;
        fix.position_ecef = Vector3d(6378137, 0, 0);
        fix.position_covariance = Matrix3d::Identity();
        fix.has_velocity = true;
        fix.velocity_ecef = Vector3d(0, 0, 5); // North at equator/Greenwich
        fix.velocity_covariance = Matrix3d::Identity();
        control.processGnssSolution(fix);
        candidate.processGnssSolution(fix);
        EXPECT_NEAR((control.state().covariance-candidate.state().covariance).norm(), 0, 1e-12);
        EXPECT_NEAR((control.state().nominal.velocity_enu-candidate.state().nominal.velocity_enu).norm(), 0, 1e-12);
    }
    ASSERT_TRUE(control.isHeadingAligned());
    ASSERT_TRUE(candidate.isHeadingAligned());
    control.processImuSample(imu(10.07));
    candidate.processImuSample(imu(10.07));
    EXPECT_GT((control.state().covariance-candidate.state().covariance).norm(), 1e-7);
}
TEST(OnlineRtkImuTest, VelocityConsistencyCandidateDefaultsAreOff) {
    const OnlineRtkImuProcessor::Config config;
    EXPECT_FALSE(config.independent_doppler_velocity);
    EXPECT_FALSE(config.fusion.reanchor_velocity_on_heading_latch);
    EXPECT_EQ(config.fusion.float_position_reanchor_after_rejections, 0);
    EXPECT_EQ(config.fusion.max_position_update_nis_per_observation, 0.0);
    EXPECT_EQ(config.fusion.max_velocity_update_nis_per_observation, 0.0);
    EXPECT_EQ(config.fusion.position_reanchor_after_gnss_gap_s, 0.0);
    EXPECT_EQ(config.rtk.reported_covariance_mode,
              RTKProcessor::RTKConfig::ReportedCovarianceMode::LEGACY_FIXED_SIGMA);
}
TEST(OnlineRtkImuTest, HeadingLatchReanchorsVelocityAndClearsItsCrossCovariance) {
    auto control_config = configuration().fusion;
    control_config.lever_arm_body.setZero();
    auto candidate_config = control_config;
    candidate_config.reanchor_velocity_on_heading_latch = true;
    LooseCouplingProcessor control(control_config), candidate(candidate_config);
    for (double t : {10.0, 10.01, 10.02, 10.03}) {
        control.processImuSample(imu(t));
        candidate.processImuSample(imu(t));
    }
    // Noisy but consistent northward course so velocity differs from each
    // measurement; ECEF +z at lon 0, lat 0 is local north.
    const double speeds[] = {5.0, 5.3, 4.7};
    const double stamps[] = {10.04, 10.05, 10.06};
    for (int i = 0; i < 3; ++i) {
        const double t = stamps[i];
        control.processImuSample(imu(t));
        candidate.processImuSample(imu(t));
        PositionSolution fix;
        fix.time = time(t);
        fix.status = SolutionStatus::SPP;
        fix.num_satellites = 8;
        fix.position_ecef = Vector3d(6378137, 0, 0);
        fix.position_covariance = Matrix3d::Identity();
        fix.has_velocity = true;
        fix.velocity_ecef = Vector3d(0, 0, speeds[i]);
        fix.velocity_covariance = 0.04 * Matrix3d::Identity();
        control.processGnssSolution(fix);
        candidate.processGnssSolution(fix);
        if (i < 2) {  // identical until the latch epoch
            ASSERT_FALSE(control.isHeadingAligned());
            ASSERT_FALSE(candidate.isHeadingAligned());
            EXPECT_NEAR((control.state().covariance-candidate.state().covariance).norm(), 0, 1e-12);
            EXPECT_NEAR((control.state().nominal.velocity_enu-candidate.state().nominal.velocity_enu).norm(), 0, 1e-12);
        }
    }
    ASSERT_TRUE(control.isHeadingAligned());
    ASSERT_TRUE(candidate.isHeadingAligned());
    const Vector3d measured(0.0, 4.7, 0.0);
    EXPECT_GT((control.state().nominal.velocity_enu-measured).norm(), 1e-3);
    EXPECT_NEAR((candidate.state().nominal.velocity_enu-measured).norm(), 0, 1e-9);
    const auto& cov = candidate.state().covariance;
    constexpr int v = fusion_index::VELOCITY;
    EXPECT_NEAR((cov.block<3, 3>(v, v) - 0.04 * Matrix3d::Identity()).norm(), 0, 1e-9);
    auto cross = [](const auto& c) {
        return c.template block<3, 3>(v, 0).norm() + c.template block<3, 9>(v, 6).norm();
    };
    EXPECT_NEAR(cross(cov), 0, 1e-12);
    EXPECT_GT(cross(control.state().covariance), 1e-9);
    EXPECT_GT(cov.diagonal().minCoeff(), 0.0);
}
TEST(OnlineRtkImuTest, RoverGapKeepsInertialFiltersDefaultsOff) {
    const OnlineRtkImuProcessor::Config config;
    EXPECT_FALSE(config.rover_gap_keeps_inertial_filters);
    EXPECT_FALSE(config.fusion.heading_latch_direction_test);
    EXPECT_EQ(OnlineRtkImuProcessor::Diagnostics().rover_gap_rtk_resets, 0U);
}
namespace {
ImuSample movingImu(double tow) {
    auto sample = imu(tow);
    // Static through the alignment window (before 10.1 s), then a
    // forward acceleration: the mechanized velocity grows, a re-alignment
    // would zero it.
    if (tow > 10.1) sample.accel_raw.x() = 0.5;
    return sample;
}
// Streams continuous IMU through 10.00 s, a rover epoch at 10.5, then more IMU
// and a rover epoch 2.5 s later (> max_rover_gap_s). `skip_imu_from/_to`
// (when set) removes IMU samples in between so that the IMU itself has a gap.
struct RoverGapRun {
    OnlineRtkImuProcessor::Output gap_epoch;
    OnlineRtkImuProcessor::Diagnostics diagnostics;
    Vector3d fused_velocity_enu;
    Vector3d reference_velocity_enu;
    double fused_time_tow = 0.0;
    double reference_time_tow = 0.0;
    bool fused_initialized = false;
};
RoverGapRun runRoverGap(bool keep_inertial, double skip_imu_from = 0.0, double skip_imu_to = 0.0) {
    auto config = configuration();
    config.rover_gap_keeps_inertial_filters = keep_inertial;
    OnlineRtkImuProcessor processor(config);
    // Reference: the same fusion configuration fed every consumed sample.
    LooseCouplingProcessor reference(config.fusion);
    auto push = [&](double t) {
        if (t > skip_imu_from && t < skip_imu_to) return;
        const auto sample = movingImu(t);
        processor.pushImu(sample, time(t));
        reference.processImuSample(sample);
    };
    for (int i = 0; i <= 50; ++i) push(10.0 + 0.01 * i);
    processor.processRover(epoch(10.5), time(10.5));
    for (int i = 51; i <= 300; ++i) push(10.0 + 0.01 * i);
    RoverGapRun run;
    run.gap_epoch = processor.processRover(epoch(13.0), time(13.0));
    run.diagnostics = processor.diagnostics();
    run.fused_velocity_enu = processor.fusionFilter().state().nominal.velocity_enu;
    run.reference_velocity_enu = reference.state().nominal.velocity_enu;
    run.fused_time_tow = processor.fusionFilter().state().nominal.time.tow;
    run.reference_time_tow = reference.state().nominal.time.tow;
    run.fused_initialized = processor.fusionFilter().isInitialized();
    return run;
}
}  // namespace
TEST(OnlineRtkImuTest, RoverGapKeepsFusedFilterWhenImuIsContinuous) {
    const auto run = runRoverGap(true);
    EXPECT_EQ(run.gap_epoch.reason, "rover_gap_rtk_reset");
    EXPECT_EQ(run.gap_epoch.reset_generation, 0U);
    EXPECT_EQ(run.diagnostics.reset_generation, 0U);
    EXPECT_EQ(run.diagnostics.rover_gap_rtk_resets, 1U);
    EXPECT_EQ(run.diagnostics.rover_gap_resets, 0U);
    EXPECT_EQ(run.diagnostics.imu_gap_resets, 0U);
    EXPECT_EQ(run.gap_epoch.imu_consumed, 250U);
    EXPECT_TRUE(run.fused_initialized);
    // The fused filter is the uninterrupted one: it kept its 3 s of velocity.
    EXPECT_GT(run.reference_velocity_enu.norm(), 1.0);
    EXPECT_EQ(run.fused_velocity_enu, run.reference_velocity_enu);
    EXPECT_EQ(run.fused_time_tow, run.reference_time_tow);
}
TEST(OnlineRtkImuTest, RoverGapResetsEverythingWhenOptionIsOff) {
    const auto run = runRoverGap(false);
    EXPECT_EQ(run.gap_epoch.reason, "rover_gap_reset");
    EXPECT_EQ(run.gap_epoch.reset_generation, 1U);
    EXPECT_EQ(run.diagnostics.reset_generation, 1U);
    EXPECT_EQ(run.diagnostics.rover_gap_resets, 1U);
    EXPECT_EQ(run.diagnostics.rover_gap_rtk_resets, 0U);
    // Re-aligned on the backlog: the continuous velocity history is lost.
    EXPECT_GT((run.fused_velocity_enu - run.reference_velocity_enu).norm(), 0.5);
}
TEST(OnlineRtkImuTest, ImuGapStillResetsEverythingWithRoverGapOptionOn) {
    // The IMU itself has a 1 s hole (11.0 .. 12.0) inside the rover gap.
    const auto run = runRoverGap(true, 11.0, 12.0);
    EXPECT_EQ(run.gap_epoch.reason, "imu_gap_reset");
    EXPECT_EQ(run.gap_epoch.reset_generation, 1U);
    EXPECT_EQ(run.diagnostics.rover_gap_rtk_resets, 1U);
    EXPECT_EQ(run.diagnostics.rover_gap_resets, 0U);
    EXPECT_EQ(run.diagnostics.imu_gap_resets, 1U);
    EXPECT_EQ(run.diagnostics.reset_generation, 1U);
    EXPECT_GT((run.fused_velocity_enu - run.reference_velocity_enu).norm(), 0.5);
}

// velocity_consistency_v6: gyro bias carried across fused-filter resets.
namespace {
const Vector3d kGyroBefore(0.010, -0.020, -0.0114);
const Vector3d kGyroAfter(0.030, 0.040, 0.0875);
ImuSample gyroImu(double tow, const Vector3d& gyro) {
    auto sample = imu(tow);
    sample.gyro_raw_radps = gyro;
    return sample;
}
struct GyroCarryRun {
    Vector3d bias_before_reset = Vector3d::Zero();
    bool was_initialized_before_reset = false;
    OnlineRtkImuProcessor::Output reset_epoch;
    OnlineRtkImuProcessor::Diagnostics diagnostics;
    bool fused_initialized = false;
    bool fused_seeded = false;
    bool fused_pending_seed = false;
    Vector3d fused_bias = Vector3d::Zero();
    Vector3d fused_window_mean = Vector3d::Zero();
    bool has_prior = false;
    bool prior_seeded = false;
    bool prior_pending_seed = false;
    bool prior_initialized = false;
    Vector3d prior_bias = Vector3d::Zero();
};
enum class GapKind { ImuGap, ImuStale, RoverGap };
// Samples through `first_end` carry kGyroBefore; the first epoch is processed
// there. The reset then comes from an IMU hole (ImuGap), a rover epoch after
// stale IMU (ImuStale), or a rover-only gap with continuous IMU (RoverGap).
// `first_start` = first_end disables the pre-gap initialization (one sample).
GyroCarryRun runGyroCarry(bool carry, GapKind kind, bool prior = false, bool pre_gap_initialized = true) {
    auto config = configuration();
    config.carry_gyro_bias_across_reset = carry;
    if (prior) config.rtk_prior_fusion = config.fusion;
    OnlineRtkImuProcessor processor(config);
    auto push = [&](double t, const Vector3d& gyro) { processor.pushImu(gyroImu(t, gyro), time(t)); };
    if (pre_gap_initialized) {
        for (int i = 0; i <= 50; ++i) push(10.0 + 0.01 * i, kGyroBefore);
        processor.processRover(epoch(10.5), time(10.5));
    } else {
        push(10.0, kGyroBefore);  // a single sample cannot initialize
        processor.processRover(epoch(10.0), time(10.0));
    }
    GyroCarryRun run;
    run.was_initialized_before_reset = processor.fusionFilter().isInitialized();
    run.bias_before_reset = processor.fusionFilter().state().nominal.gyro_bias;
    const double last = pre_gap_initialized ? 10.5 : 10.0;
    if (kind == GapKind::ImuGap) {
        // 1.5 s IMU hole, then new data; the epoch is after it.
        for (int i = 0; i <= 50; ++i) push(last + 1.5 + 0.01 * i, kGyroAfter);
        run.reset_epoch = processor.processRover(epoch(last + 2.0), time(last + 2.0));
    } else if (kind == GapKind::ImuStale) {
        // An epoch arrives with no IMU since `last`: stale reset, then fresh IMU.
        const auto stale = processor.processRover(epoch(last + 1.0), time(last + 1.0));
        EXPECT_EQ(stale.reason, "imu_stale_reset");
        for (int i = 0; i <= 50; ++i) push(last + 1.5 + 0.01 * i, kGyroAfter);
        run.reset_epoch = processor.processRover(epoch(last + 2.0), time(last + 2.0));
    } else {
        // Continuous IMU, rover epoch 2.5 s later (> max_rover_gap_s).
        for (int i = 1; i <= 250; ++i) push(last + 0.01 * i, kGyroAfter);
        run.reset_epoch = processor.processRover(epoch(last + 2.5), time(last + 2.5));
    }
    run.diagnostics = processor.diagnostics();
    const auto& fused = processor.fusionFilter();
    run.fused_initialized = fused.isInitialized();
    run.fused_seeded = fused.lastInitializationGyroBiasSeeded();
    run.fused_pending_seed = fused.hasPendingGyroBiasSeed();
    run.fused_bias = fused.state().nominal.gyro_bias;
    run.fused_window_mean = fused.lastInitializationWindowGyroBias();
    if (const auto* p = processor.priorFusionFilter()) {
        run.has_prior = true;
        run.prior_initialized = p->isInitialized();
        run.prior_seeded = p->lastInitializationGyroBiasSeeded();
        run.prior_pending_seed = p->hasPendingGyroBiasSeed();
        run.prior_bias = p->state().nominal.gyro_bias;
    }
    return run;
}
}  // namespace

TEST(OnlineRtkImuTest, CarryGyroBiasAcrossResetDefaultsOff) {
    const OnlineRtkImuProcessor::Config config;
    EXPECT_FALSE(config.carry_gyro_bias_across_reset);
}

TEST(OnlineRtkImuTest, GyroBiasCarriedAcrossImuGapReset) {
    const auto run = runGyroCarry(true, GapKind::ImuGap);
    ASSERT_TRUE(run.was_initialized_before_reset);
    EXPECT_NEAR((run.bias_before_reset - kGyroBefore).norm(), 0.0, 1e-12);
    EXPECT_EQ(run.reset_epoch.reason, "imu_gap_reset");
    EXPECT_EQ(run.diagnostics.imu_gap_resets, 1U);
    ASSERT_TRUE(run.fused_initialized);
    EXPECT_TRUE(run.fused_seeded);
    EXPECT_FALSE(run.fused_pending_seed);
    EXPECT_EQ(run.fused_bias, run.bias_before_reset);
    EXPECT_NEAR((run.fused_window_mean - kGyroAfter).norm(), 0.0, 1e-12);
}

TEST(OnlineRtkImuTest, GyroBiasCarriedAcrossImuStaleReset) {
    const auto run = runGyroCarry(true, GapKind::ImuStale);
    ASSERT_TRUE(run.was_initialized_before_reset);
    EXPECT_EQ(run.diagnostics.imu_gap_resets, 1U);
    ASSERT_TRUE(run.fused_initialized);
    EXPECT_TRUE(run.fused_seeded);
    EXPECT_EQ(run.fused_bias, run.bias_before_reset);
    EXPECT_NEAR((run.fused_window_mean - kGyroAfter).norm(), 0.0, 1e-12);
}

TEST(OnlineRtkImuTest, GyroBiasCarriedAcrossRoverGapResetWhenFusedFilterIsRecreated) {
    const auto run = runGyroCarry(true, GapKind::RoverGap);
    ASSERT_TRUE(run.was_initialized_before_reset);
    EXPECT_EQ(run.reset_epoch.reason, "rover_gap_reset");
    EXPECT_EQ(run.diagnostics.rover_gap_resets, 1U);
    ASSERT_TRUE(run.fused_initialized);
    EXPECT_TRUE(run.fused_seeded);
    EXPECT_EQ(run.fused_bias, run.bias_before_reset);
    EXPECT_NEAR((run.fused_window_mean - kGyroAfter).norm(), 0.0, 1e-12);
}

TEST(OnlineRtkImuTest, GyroBiasNotSeededWhenOldFilterWasNeverInitialized) {
    for (const auto kind : {GapKind::ImuGap, GapKind::ImuStale}) {
        const auto run = runGyroCarry(true, kind, false, false);
        EXPECT_FALSE(run.was_initialized_before_reset);
        EXPECT_EQ(run.diagnostics.imu_gap_resets, 1U);
        ASSERT_TRUE(run.fused_initialized);
        EXPECT_FALSE(run.fused_seeded);
        EXPECT_FALSE(run.fused_pending_seed);
        EXPECT_NEAR((run.fused_bias - kGyroAfter).norm(), 0.0, 1e-12);
    }
}

TEST(OnlineRtkImuTest, GyroBiasIsWindowMeanWhenCarryOptionIsOff) {
    for (const auto kind : {GapKind::ImuGap, GapKind::ImuStale, GapKind::RoverGap}) {
        const auto run = runGyroCarry(false, kind);
        ASSERT_TRUE(run.was_initialized_before_reset);
        ASSERT_TRUE(run.fused_initialized);
        EXPECT_FALSE(run.fused_seeded);
        EXPECT_FALSE(run.fused_pending_seed);
        EXPECT_NEAR((run.fused_bias - kGyroAfter).norm(), 0.0, 1e-12);
        EXPECT_GT((run.fused_bias - run.bias_before_reset).norm(), 0.01);
    }
}

TEST(OnlineRtkImuTest, RtkPriorFilterIsNeverSeeded) {
    for (const auto kind : {GapKind::ImuGap, GapKind::ImuStale, GapKind::RoverGap}) {
        const auto run = runGyroCarry(true, kind, true);
        ASSERT_TRUE(run.has_prior);
        ASSERT_TRUE(run.fused_initialized);
        ASSERT_TRUE(run.prior_initialized);
        EXPECT_TRUE(run.fused_seeded);
        EXPECT_EQ(run.fused_bias, run.bias_before_reset);
        EXPECT_FALSE(run.prior_seeded);
        EXPECT_FALSE(run.prior_pending_seed);
        EXPECT_NEAR((run.prior_bias - kGyroAfter).norm(), 0.0, 1e-12);
    }
}
}
