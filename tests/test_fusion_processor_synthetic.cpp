#include <gtest/gtest.h>

#include <libgnss++/fusion/fusion_processor.hpp>

#include <cmath>
#include <vector>

#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/fusion/mechanization.hpp>

namespace libgnss {
namespace {

constexpr double kGravity = 9.80665;
const Eigen::Vector3d kGravityEnu(0.0, 0.0, -kGravity);
constexpr double kDt = 0.01;  // 100 Hz

// Synthetic multi-minute trajectory: stationary (for alignment) -> straight
// acceleration -> straight cruise -> in-place turn (yaw only) -> cruise
// through a simulated GNSS dropout -> cruise with GNSS restored. Truth is
// generated with the *same* mechanization::propagate() the processor itself
// uses (fed a piecewise-constant per-phase specific-force/turn-rate command,
// so it is exactly self-consistent), then a constant sensor bias is added
// only to the copy of each sample handed to the processor -- exactly like a
// real (bias-free-model, biased-sensor) IMU.
struct PhaseCommand {
    double end_time_s;
    Eigen::Vector3d horizontal_accel_enu;  // commanded ENU horizontal acceleration
    double turn_rate_radps;                // commanded yaw rate about body/ENU Up
};

const std::vector<PhaseCommand> kPhases = {
    {3.0, Eigen::Vector3d::Zero(), 0.0},                       // P0: stationary (alignment)
    {8.0, Eigen::Vector3d(1.0, 0.0, 0.0), 0.0},                 // P1: accelerate to 5 m/s East
    {28.0, Eigen::Vector3d::Zero(), 0.0},                       // P2: cruise straight
    {38.0, Eigen::Vector3d::Zero(), 0.05},                      // P3: in-place turn
    {58.0, Eigen::Vector3d::Zero(), 0.0},                       // P4: cruise (dropout inside this phase)
    {70.0, Eigen::Vector3d::Zero(), 0.0},                       // P5: cruise, GNSS restored
};

constexpr double kDropoutStart = 40.0;
constexpr double kDropoutEnd = 50.0;

PhaseCommand commandAt(double t) {
    for (const auto& phase : kPhases) {
        if (t < phase.end_time_s) return phase;
    }
    return kPhases.back();
}

TEST(FusionProcessorSyntheticTest, AppliesTightlyCoupledDDRowsToLiveINSState) {
    LooseCouplingProcessor::Config config;
    config.align_static_window_s = 0.1;
    config.zupt_enable = false;
    LooseCouplingProcessor processor(config);

    GNSSTime time(2200, 100000.0);
    for (int i = 0; i < 20; ++i) {
        ImuSample sample;
        sample.time = time;
        sample.accel_raw = Eigen::Vector3d(0.0, 0.0, kGravity);
        sample.gyro_raw_radps.setZero();
        processor.processImuSample(sample);
        time = time + kDt;
    }
    ASSERT_TRUE(processor.isInitialized());

    PositionSolution anchor;
    anchor.time = time;
    anchor.status = SolutionStatus::FIXED;
    anchor.num_satellites = 10;
    anchor.position_ecef = geodetic2ecef(35.6 * M_PI / 180.0,
                                         139.7 * M_PI / 180.0, 50.0);
    anchor.position_covariance = Eigen::Matrix3d::Identity();
    processor.processGnssSolution(anchor);
    ASSERT_TRUE(processor.isOriginSet());

    dd_imu_bridge::DDObservation row;
    row.key = {3, 0, 0};
    row.key.satellite_system = static_cast<int>(GNSSSystem::GPS);
    row.key.reference_satellite_prn = 20;
    row.key.reference_satellite_system = static_cast<int>(GNSSSystem::GPS);
    row.geometry_enu = Eigen::RowVector3d::UnitX();
    row.code_residual_m = 1.0;
    row.code_variance_m2 = 0.25;
    row.carrier_residual_m = 0.19 * 12.1;
    row.carrier_variance_m2 = 0.0025;
    row.wavelength_m = 0.19;
    row.elevation_rad = 0.8;
    row.lock_count = 100;

    const double before_x = processor.state().nominal.position_enu.x();
    const auto result = processor.processTightlyCoupledDD({row}, &anchor);
    EXPECT_TRUE(result.update.ok);
    EXPECT_EQ(result.update.observation_count, 2);
    EXPECT_GT(processor.state().nominal.position_enu.x(), before_x);
    const Eigen::Matrix3d rotation = processor.ecefToLocalEnuRotation();
    EXPECT_TRUE((rotation * rotation.transpose()).isApprox(Eigen::Matrix3d::Identity(), 1e-12));
}

TEST(FusionProcessorSyntheticTest, TracksSyntheticTrajectoryThroughTurnAndDropout) {
    const double origin_lat = 35.6 * M_PI / 180.0;
    const double origin_lon = 139.7 * M_PI / 180.0;
    const double origin_height = 50.0;
    const Eigen::Vector3d origin_ecef = geodetic2ecef(origin_lat, origin_lon, origin_height);

    const Eigen::Vector3d true_accel_bias(0.02, -0.01, 0.015);
    const Eigen::Vector3d true_gyro_bias(0.001, -0.001, 0.0008);

    LooseCouplingProcessor::Config config;
    config.align_static_window_s = 2.0;
    config.zupt_enable = true;
    config.nhc_enable = false;
    LooseCouplingProcessor processor(config);

    NominalState truth;
    truth.attitude_body_to_enu = Eigen::Quaterniond::Identity();
    truth.position_enu.setZero();
    truth.velocity_enu.setZero();

    GNSSTime time(2200, 400000.0);
    double t = 0.0;
    double next_gnss_time = 1.0;  // first GNSS fix at t=1s (after alignment starts)
    double sum_squared_error = 0.0;
    int error_samples = 0;
    double final_error_m = 0.0;

    const double total_duration_s = kPhases.back().end_time_s;
    const int total_steps = static_cast<int>(total_duration_s / kDt);

    for (int step = 0; step < total_steps; ++step) {
        const PhaseCommand phase = commandAt(t);

        ImuSample truth_sample;
        truth_sample.time = time;
        truth_sample.accel_raw =
            truth.attitude_body_to_enu.conjugate() * (phase.horizontal_accel_enu - kGravityEnu);
        truth_sample.gyro_raw_radps = Eigen::Vector3d(0.0, 0.0, phase.turn_rate_radps);

        truth = mechanization::propagate(truth, truth_sample, kDt, kGravityEnu);

        ImuSample sensor_sample = truth_sample;
        sensor_sample.accel_raw += true_accel_bias;
        sensor_sample.gyro_raw_radps += true_gyro_bias;
        processor.processImuSample(sensor_sample);

        t += kDt;
        time = time + kDt;

        const bool in_dropout = (t >= kDropoutStart && t < kDropoutEnd);
        if (t >= next_gnss_time && !in_dropout) {
            PositionSolution solution;
            solution.time = time;
            solution.status = SolutionStatus::FIXED;
            solution.num_satellites = 12;
            solution.position_ecef = origin_ecef + enu2ecef(truth.position_enu, origin_lat, origin_lon);
            solution.position_covariance = (0.02 * 0.02) * Eigen::Matrix3d::Identity();
            solution.velocity_ecef = enu2ecef(truth.velocity_enu, origin_lat, origin_lon);
            solution.velocity_covariance = (0.05 * 0.05) * Eigen::Matrix3d::Identity();
            solution.has_velocity = true;
            processor.processGnssSolution(solution);
            next_gnss_time += 1.0;
        }

        // Track position error every second (regardless of GNSS
        // availability) once the processor has aligned and seen its first
        // fix, so the RMSE reflects steady-state + dropout-coasting
        // performance rather than the initial alignment transient.
        if (processor.isOriginSet() && t >= 10.0 &&
            std::fmod(t, 1.0) < kDt * 0.5) {
            const PositionSolution fused = processor.toPositionSolution();
            const Eigen::Vector3d truth_ecef =
                origin_ecef + enu2ecef(truth.position_enu, origin_lat, origin_lon);
            const double error = (fused.position_ecef - truth_ecef).norm();
            sum_squared_error += error * error;
            ++error_samples;
            final_error_m = error;
        }
    }

    ASSERT_TRUE(processor.isOriginSet());
    ASSERT_GT(error_samples, 0);
    const double rmse_m = std::sqrt(sum_squared_error / error_samples);

    EXPECT_LT(rmse_m, 3.0) << "position RMSE too high: " << rmse_m << " m";
    EXPECT_LT(final_error_m, 3.0) << "final position error too high: " << final_error_m << " m";

    // Bias states should stay in the neighborhood of their true (injected)
    // values -- a loose bound, since gyro-bias observability through
    // GNSS position/velocity alone (via the indirect attitude<->bias
    // coupling in Fc) is weak over a ~1-minute run; this is a
    // did-it-blow-up regression check, not a tight convergence guarantee
    // (full validation is deferred to the dataset-level sign-off phase).
    // Bound loosened slightly (0.02 -> 0.035) after the heading-alignment
    // fix (PPC nagoya/run1 investigation): the multi-epoch consistency gate
    // (Config::align_heading_min_samples) now takes a couple of extra GNSS
    // epochs to latch heading versus the old single-epoch latch, so yaw
    // stays at its large initial uncertainty slightly longer, which shifts
    // (but does not blow up) the gyro-bias trajectory over this short run.
    const Eigen::Vector3d gyro_bias_error = processor.state().nominal.gyro_bias - true_gyro_bias;
    EXPECT_LT(gyro_bias_error.norm(), 0.035);
}

// Regression test for docs/design.md's root-cause finding: neither
// SPPProcessor nor RTKProcessor used to populate PositionSolution's
// has_velocity, so LooseCouplingProcessor::processGnssSolution()'s velocity
// update and GNSS-course heading alignment (fusion_initialization::
// tryAlignHeading(), gated on has_velocity) never fired -- heading stayed
// at whatever alignStatic() left it (unobservable at rest) for an entire
// run. Directly exercises the has_velocity gate: a moving-speed GNSS
// solution with has_velocity=false must NOT collapse the yaw uncertainty,
// while the same solution with has_velocity=true must.
TEST(FusionProcessorSyntheticTest, HeadingAlignmentOnlyFiresWhenSolutionHasVelocity) {
    const Eigen::Vector3d origin_ecef = geodetic2ecef(35.6 * M_PI / 180.0, 139.7 * M_PI / 180.0, 50.0);

    auto buildAlignedProcessor = [&](const LooseCouplingProcessor::Config& config) {
        LooseCouplingProcessor processor(config);
        GNSSTime time(2200, 400000.0);
        ImuSample sample;
        sample.time = time;
        sample.accel_raw = Eigen::Vector3d(0.0, 0.0, kGravity);
        sample.gyro_raw_radps = Eigen::Vector3d::Zero();
        // Feed a short stationary window so alignStatic() fires (see
        // config.align_static_window_s below), leaving yaw at its large
        // initial uncertainty (heading is unobservable at rest).
        for (int i = 0; i < 400; ++i) {
            sample.time = time;
            processor.processImuSample(sample);
            time = time + kDt;
        }
        return processor;
    };

    LooseCouplingProcessor::Config config;
    config.align_static_window_s = 2.0;
    config.zupt_enable = false;
    config.nhc_enable = false;
    config.align_velocity_threshold_mps = 1.0;
    config.align_heading_sigma_deg = 5.0;
    // This test's focus is narrowly the has_velocity gate (see doc comment
    // above), not the multi-epoch consistency gate added for the PPC
    // nagoya/run1 fix (HeadingAlignmentTrackerTest and
    // FusionProcessorSyntheticTest.RecoversFromConsistentButWrongInitialHeadingLatch
    // cover that separately) -- relax min_samples to 1 so a single
    // has_velocity=true epoch is still sufficient to latch here.
    config.align_heading_min_samples = 1;

    constexpr int kAttitudeYawIndex = fusion_index::ATTITUDE + 2;
    const double aligned_variance_rad2 =
        (config.align_heading_sigma_deg * M_PI / 180.0) * (config.align_heading_sigma_deg * M_PI / 180.0);

    // Case 1: has_velocity = false -- yaw uncertainty must remain large
    // (well above the post-alignment target variance).
    {
        LooseCouplingProcessor processor = buildAlignedProcessor(config);
        ASSERT_FALSE(processor.isOriginSet());  // no GNSS fix consumed yet

        PositionSolution solution;
        solution.time = GNSSTime(2200, 400000.0 + 4.0);
        solution.status = SolutionStatus::FIXED;
        solution.num_satellites = 10;
        solution.position_ecef = origin_ecef;
        solution.position_covariance = (0.02 * 0.02) * Eigen::Matrix3d::Identity();
        solution.velocity_ecef = Eigen::Vector3d(5.0, 0.0, 0.0);  // would exceed threshold if used
        solution.velocity_covariance = (0.05 * 0.05) * Eigen::Matrix3d::Identity();
        solution.has_velocity = false;  // <-- the bug this fixes: must gate on this
        processor.processGnssSolution(solution);

        const double yaw_variance = processor.state().covariance(kAttitudeYawIndex, kAttitudeYawIndex);
        EXPECT_GT(yaw_variance, aligned_variance_rad2 * 10.0)
            << "heading alignment must not fire without has_velocity";
    }

    // Case 2: has_velocity = true and above the alignment speed threshold --
    // yaw uncertainty must collapse to the configured post-alignment sigma.
    {
        LooseCouplingProcessor processor = buildAlignedProcessor(config);

        PositionSolution solution;
        solution.time = GNSSTime(2200, 400000.0 + 4.0);
        solution.status = SolutionStatus::FIXED;
        solution.num_satellites = 10;
        solution.position_ecef = origin_ecef;
        solution.position_covariance = (0.02 * 0.02) * Eigen::Matrix3d::Identity();
        solution.velocity_ecef = Eigen::Vector3d(5.0, 0.0, 0.0);
        solution.velocity_covariance = (0.05 * 0.05) * Eigen::Matrix3d::Identity();
        solution.has_velocity = true;
        processor.processGnssSolution(solution);

        const double yaw_variance = processor.state().covariance(kAttitudeYawIndex, kAttitudeYawIndex);
        EXPECT_NEAR(yaw_variance, aligned_variance_rad2, 1e-9)
            << "heading alignment must fire once has_velocity is set and speed exceeds threshold";
    }
}

// Regression test for the PPC nagoya/run1 root cause (see
// fusion_processor.hpp Config::heading_recovery_*): a heading latch that is
// internally *consistent* (several agreeing GNSS-course samples) can still
// be flat-out wrong -- e.g. a vehicle backing out of a parking spot reports
// a GNSS course roughly opposite its true nose heading for several
// consecutive epochs, which the multi-epoch consistency gate alone cannot
// catch (see HeadingAlignmentTrackerTest.
// ConsistentCourseIsReadyEvenIfItWouldBePhysicallyWrong in
// test_fusion_initialization.cpp).
//
// This exercises the complementary recovery *detection* mechanism -- the
// "real health metric" the always-yes "Heading aligned" printout is replaced
// with (isHeadingConverged(), and the yaw-covariance re-inflation that backs
// it). It deliberately does NOT assert that the yaw estimate fully
// reconverges to some tight final tolerance: PPC nagoya/run1 validation (and
// repeated attempts at a favorable controlled synthetic case, including a
// nonzero lever arm plus a continuous sustained turn to maximize yaw
// observability) showed re-inflating covariance and letting the ordinary
// Kalman updates take it from there does not reliably converge in bounded
// time -- it can just as easily random-walk to a different wrong heading,
// which is exactly why this mechanism defaults to *disabled*
// (Config::heading_recovery_min_bad_epochs = 0) and is not relied on for the
// PPC dataset fix (that fix is the multi-epoch consistency gate above,
// validated separately). What IS reliable, and worth a regression test, is
// that the detector itself correctly flags a persistently-wrong latch as
// unhealthy (isHeadingConverged() goes false) rather than reporting the old
// code's meaningless permanent "yes".
TEST(FusionProcessorSyntheticTest, DetectsAndFlagsAConsistentButWrongInitialHeadingLatch) {
    LooseCouplingProcessor::Config config;
    config.align_static_window_s = 2.0;
    config.zupt_enable = false;
    config.nhc_enable = false;
    // Zero lever arm deliberately: this keeps the GNSS position/velocity
    // updates' attitude-Jacobian block exactly zero (see
    // fusion_measurement::buildGnssVelocityUpdate), so the only channel
    // through which a wrong yaw can be detected/corrected is the ordinary
    // process-model cross-covariance built up during the acceleration phase
    // below -- the same "heading becomes observable during accelerations"
    // mechanism reference_notes.md and docs/design.md describe, isolated
    // from the (separately-tested, dataset-validated-harmful-by-default)
    // lever-arm/angular-rate coupling.
    config.align_velocity_threshold_mps = 1.0;
    config.align_heading_window_s = 5.0;
    config.align_heading_min_samples = 3;
    config.align_heading_max_course_scatter_deg = 10.0;
    config.align_heading_sigma_deg = 5.0;
    config.heading_recovery_nis_threshold = 30.0;
    config.heading_recovery_min_bad_epochs = 4;
    config.heading_recovery_cooldown_epochs = 10;
    LooseCouplingProcessor processor(config);

    const double origin_lat = 35.6 * M_PI / 180.0;
    const double origin_lon = 139.7 * M_PI / 180.0;
    const double origin_height = 50.0;
    const Eigen::Vector3d origin_ecef = geodetic2ecef(origin_lat, origin_lon, origin_height);

    constexpr int kAttitudeYawIndex = fusion_index::ATTITUDE + 2;
    const double aligned_variance_rad2 =
        (config.align_heading_sigma_deg * M_PI / 180.0) * (config.align_heading_sigma_deg * M_PI / 180.0);

    NominalState truth;
    truth.attitude_body_to_enu = Eigen::Quaterniond::Identity();  // true forward = East (ENU x)
    truth.position_enu.setZero();
    truth.velocity_enu.setZero();

    GNSSTime time(2200, 400000.0);
    double t = 0.0;

    // Truth trajectory: stationary for alignment, then a straight
    // acceleration (East) during which the wrong latch is forced and its
    // IMU-mechanization error accumulates in the velocity residual (heading
    // is unobservable at constant velocity, same reasoning docs/design.md
    // and reference_notes.md document).
    constexpr double kStationaryEnd = 3.0;
    constexpr double kAccelerateEnd = 23.0;
    constexpr double kHorizontalAccel = 0.5;  // m/s^2

    bool latched_wrong_once = false;
    bool observed_unconverged_after_wrong_latch = false;
    bool recovered_once = false;  // yaw covariance observed re-inflated after the wrong latch

    double next_gnss_time = 1.0;
    const int total_steps = static_cast<int>(kAccelerateEnd / kDt);
    for (int step = 0; step < total_steps; ++step) {
        Eigen::Vector3d accel_enu = Eigen::Vector3d::Zero();
        if (t >= kStationaryEnd) {
            accel_enu = Eigen::Vector3d(kHorizontalAccel, 0.0, 0.0);
        }

        ImuSample truth_sample;
        truth_sample.time = time;
        truth_sample.accel_raw = truth.attitude_body_to_enu.conjugate() * (accel_enu - kGravityEnu);
        truth_sample.gyro_raw_radps = Eigen::Vector3d::Zero();
        truth = mechanization::propagate(truth, truth_sample, kDt, kGravityEnu);

        processor.processImuSample(truth_sample);  // bias-free sensor, kept simple on purpose

        t += kDt;
        time = time + kDt;

        if (t >= next_gnss_time) {
            PositionSolution solution;
            solution.time = time;
            solution.status = SolutionStatus::FIXED;
            solution.num_satellites = 12;
            solution.position_ecef = origin_ecef + enu2ecef(truth.position_enu, origin_lat, origin_lon);
            solution.position_covariance = (0.02 * 0.02) * Eigen::Matrix3d::Identity();

            // Until the processor latches for the first time, report a
            // velocity rotated 90 deg from the truth -- self-consistent
            // (so the scatter gate alone does not catch it) but wrong,
            // exactly the "reverse-out-of-parking" failure mode this fix
            // targets. Once latched, switch to truthful GNSS velocity for
            // the remainder of the run.
            Eigen::Vector3d reported_velocity_enu = truth.velocity_enu;
            if (!latched_wrong_once) {
                reported_velocity_enu =
                    Eigen::Vector3d(-truth.velocity_enu.y(), truth.velocity_enu.x(), 0.0);
            }

            solution.velocity_ecef = enu2ecef(reported_velocity_enu, origin_lat, origin_lon);
            solution.velocity_covariance = (0.05 * 0.05) * Eigen::Matrix3d::Identity();
            solution.has_velocity = true;
            processor.processGnssSolution(solution);
            next_gnss_time += 1.0;

            const double yaw_variance = processor.state().covariance(kAttitudeYawIndex, kAttitudeYawIndex);

            if (!latched_wrong_once && processor.isHeadingAligned()) {
                latched_wrong_once = true;
            } else if (latched_wrong_once && !recovered_once) {
                if (!processor.isHeadingConverged()) {
                    observed_unconverged_after_wrong_latch = true;
                }
                if (yaw_variance > aligned_variance_rad2 * 10.0) {
                    // Covariance was re-inflated well above the tight
                    // post-latch sigma -- recovery fired (the nominal
                    // attitude and heading_aligned_ are deliberately left
                    // untouched by design, see Config::heading_recovery_*
                    // doc comment).
                    recovered_once = true;
                }
            }
        }
    }

    ASSERT_TRUE(latched_wrong_once) << "test setup did not even reach the initial (wrong) latch";
    EXPECT_TRUE(observed_unconverged_after_wrong_latch)
        << "isHeadingConverged() never flagged the wrong latch as unhealthy -- the replacement for "
           "the old always-yes 'Heading aligned' printout is not doing its job";
    EXPECT_TRUE(recovered_once) << "processor never re-inflated yaw covariance via velocity NIS";
}

}  // namespace
}  // namespace libgnss
