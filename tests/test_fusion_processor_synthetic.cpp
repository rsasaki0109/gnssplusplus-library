#include <gtest/gtest.h>

#include <libgnss++/fusion/fusion_processor.hpp>

#include <cmath>
#include <limits>
#include <vector>

#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/fusion/attitude.hpp>
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

TEST(FusionProcessorSyntheticTest, ReportsAcceptedGnssPositionCorrectionBeforeVelocityUpdate) {
    LooseCouplingProcessor::Config config;
    config.align_static_window_s = 0.1;
    config.zupt_enable = false;
    LooseCouplingProcessor processor(config);

    GNSSTime time(2200, 100000.0);
    for (int i = 0; i < 20; ++i) {
        ImuSample sample;
        sample.time = time;
        sample.accel_raw = Eigen::Vector3d(0.0, 0.0, kGravity);
        processor.processImuSample(sample);
        time = time + kDt;
    }
    ASSERT_TRUE(processor.isInitialized());
    EXPECT_FALSE(processor.lastGnssPositionUpdateApplied());

    PositionSolution solution;
    solution.time = time;
    solution.status = SolutionStatus::FIXED;
    solution.num_satellites = 10;
    solution.position_ecef = geodetic2ecef(
        35.6 * M_PI / 180.0, 139.7 * M_PI / 180.0, 50.0);
    solution.position_covariance =
        0.01 * Eigen::Matrix3d::Identity();
    processor.processGnssSolution(solution);
    ASSERT_TRUE(processor.lastGnssPositionUpdateApplied());

    solution.position_ecef +=
        processor.ecefToLocalEnuRotation().transpose() *
        Eigen::Vector3d(1.0, 0.0, 0.0);
    processor.processGnssSolution(solution);
    EXPECT_TRUE(processor.lastGnssPositionUpdateApplied());
    EXPECT_GT(processor.lastGnssPositionCorrectionEnu().x(), 0.0);
    EXPECT_TRUE(processor.lastGnssPositionCorrectionEnu().allFinite());
}

TEST(FusionProcessorSyntheticTest, AntennaFrameOutputAppliesLeverArmToPositionVelocityAndCovariance) {
    LooseCouplingProcessor::Config config;
    config.align_static_window_s = 0.1;
    config.zupt_enable = false;
    config.nhc_enable = false;
    config.lever_arm_body = Eigen::Vector3d(0.31, 0.0, -0.55);
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

    const double lat = 35.6 * M_PI / 180.0;
    const double lon = 139.7 * M_PI / 180.0;
    PositionSolution anchor;
    anchor.time = time;
    anchor.status = SolutionStatus::FIXED;
    anchor.num_satellites = 10;
    anchor.position_ecef = geodetic2ecef(lat, lon, 50.0);
    anchor.position_covariance = Eigen::Matrix3d::Identity();
    processor.processGnssSolution(anchor);
    ASSERT_TRUE(processor.isOriginSet());

    // Leave a nonzero, known angular rate in the processor so the velocity
    // lever-arm term and its attitude Jacobian are exercised as well.
    const Eigen::Vector3d angular_rate_body(0.0, 0.0, 0.2);
    ImuSample motion;
    motion.time = time;
    motion.accel_raw = Eigen::Vector3d(0.0, 0.0, kGravity);
    motion.gyro_raw_radps = angular_rate_body;
    processor.processImuSample(motion);

    const PositionSolution imu_solution = processor.toPositionSolution();
    const PositionSolution antenna_solution = processor.toAntennaPositionSolution();
    const FusionState& state = processor.state();
    const Eigen::Matrix3d ecef_to_enu = processor.ecefToLocalEnuRotation();
    const Eigen::Matrix3d enu_to_ecef = ecef_to_enu.transpose();
    const Eigen::Matrix3d rotation = state.nominal.attitude_body_to_enu.toRotationMatrix();
    const Eigen::Vector3d lever_arm = config.lever_arm_body;
    const Eigen::Vector3d corrected_angular_rate_body =
        angular_rate_body - state.nominal.gyro_bias;
    const Eigen::Vector3d lever_velocity_body = corrected_angular_rate_body.cross(lever_arm);

    const Eigen::Vector3d expected_position_ecef =
        imu_solution.position_ecef + enu_to_ecef * (rotation * lever_arm);
    const Eigen::Vector3d expected_velocity_ecef =
        imu_solution.velocity_ecef + enu_to_ecef * (rotation * lever_velocity_body);
    EXPECT_TRUE(antenna_solution.position_ecef.isApprox(expected_position_ecef, 1e-9));
    EXPECT_TRUE(antenna_solution.velocity_ecef.isApprox(expected_velocity_ecef, 1e-9))
        << "actual=" << antenna_solution.velocity_ecef.transpose()
        << " expected=" << expected_velocity_ecef.transpose()
        << " imu=" << imu_solution.velocity_ecef.transpose();
    double expected_lat = 0.0;
    double expected_lon = 0.0;
    double expected_height = 0.0;
    ecef2geodetic(expected_position_ecef, expected_lat, expected_lon, expected_height);
    EXPECT_NEAR(antenna_solution.position_geodetic.latitude, expected_lat, 1e-12);
    EXPECT_NEAR(antenna_solution.position_geodetic.longitude, expected_lon, 1e-12);
    EXPECT_NEAR(antenna_solution.position_geodetic.height, expected_height, 1e-8);
    EXPECT_TRUE(antenna_solution.has_velocity);
    EXPECT_NE(antenna_solution.position_ecef, imu_solution.position_ecef);
    EXPECT_NE(antenna_solution.velocity_ecef, imu_solution.velocity_ecef);

    Eigen::MatrixXd position_h = Eigen::MatrixXd::Zero(3, fusion_index::SIZE);
    position_h.block<3, 3>(0, fusion_index::POSITION) = Eigen::Matrix3d::Identity();
    position_h.block<3, 3>(0, fusion_index::ATTITUDE) =
        -rotation * attitude::skew(lever_arm);
    const Eigen::Matrix3d expected_position_covariance_enu =
        position_h * state.covariance * position_h.transpose();

    Eigen::MatrixXd velocity_h = Eigen::MatrixXd::Zero(3, fusion_index::SIZE);
    velocity_h.block<3, 3>(0, fusion_index::VELOCITY) = Eigen::Matrix3d::Identity();
    velocity_h.block<3, 3>(0, fusion_index::ATTITUDE) =
        -rotation * attitude::skew(lever_velocity_body);
    const Eigen::Matrix3d expected_velocity_covariance_enu =
        velocity_h * state.covariance * velocity_h.transpose();

    EXPECT_TRUE(antenna_solution.position_covariance.isApprox(
        enu_to_ecef * expected_position_covariance_enu * ecef_to_enu, 1e-12));
    EXPECT_TRUE(antenna_solution.velocity_covariance.isApprox(
        enu_to_ecef * expected_velocity_covariance_enu * ecef_to_enu, 1e-12))
        << "actual=\n" << antenna_solution.velocity_covariance
        << " expected=\n"
        << enu_to_ecef * expected_velocity_covariance_enu * ecef_to_enu;
}

TEST(FusionProcessorSyntheticTest, FixedPositionGateReanchorsPositionOnly) {
    LooseCouplingProcessor::Config config;
    config.align_static_window_s = 0.1;
    config.zupt_enable = false;
    config.nhc_enable = false;
    config.lever_arm_body = Eigen::Vector3d(0.31, 0.0, -0.55);
    config.max_position_update_nis_per_observation = 0.1;
    config.max_velocity_update_nis_per_observation = 0.1;
    config.max_consecutive_gate_rejections = 2;
    config.position_updates_require_fixed = true;
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

    const Eigen::Vector3d origin_ecef =
        geodetic2ecef(35.6 * M_PI / 180.0, 139.7 * M_PI / 180.0, 50.0);
    PositionSolution anchor;
    anchor.time = time;
    anchor.status = SolutionStatus::FIXED;
    anchor.num_satellites = 10;
    anchor.position_ecef = origin_ecef;
    // Give the first anchor enough measurement covariance to absorb the
    // initial lever-arm offset without tripping the intentionally strict
    // regression gate below.
    anchor.position_covariance = 100.0 * Eigen::Matrix3d::Identity();
    processor.processGnssSolution(anchor);
    ASSERT_TRUE(processor.isOriginSet());

    const FusionState before_rejections = processor.state();
    const Eigen::Matrix3d ecef_to_enu = processor.ecefToLocalEnuRotation();
    const Eigen::Matrix3d enu_to_ecef = ecef_to_enu.transpose();
    const Eigen::Matrix3d rotation =
        before_rejections.nominal.attitude_body_to_enu.toRotationMatrix();
    // The default library re-anchor bound is intentionally unbounded. A
    // returning, accurate FIX can be more than the old 20 m arbitrary cap
    // away after a long outage; the FIX patience gate is the trust condition.
    const Eigen::Vector3d desired_antenna_position_enu =
        before_rejections.nominal.position_enu +
        rotation * config.lever_arm_body + Eigen::Vector3d(25.0, -2.0, 1.0);

    PositionSolution fixed_outlier = anchor;
    fixed_outlier.position_covariance = 0.01 * Eigen::Matrix3d::Identity();
    fixed_outlier.position_ecef =
        origin_ecef + enu_to_ecef * desired_antenna_position_enu;
    fixed_outlier.time = time + kDt;

    // The first gate rejection only arms the patience counter. It must not
    // change the nominal state or the position correction telemetry.
    processor.processGnssSolution(fixed_outlier);
    EXPECT_FALSE(processor.lastGnssPositionUpdateApplied());
    EXPECT_FALSE(processor.lastGnssPositionReanchored());
    EXPECT_TRUE(processor.state().nominal.position_enu.isApprox(
        before_rejections.nominal.position_enu, 1e-12));

    // The second rejection reaches the configured patience and invokes the
    // fixed-position-only re-anchor. No ungated EKF gain is used here.
    const Eigen::Vector3d velocity_before = processor.state().nominal.velocity_enu;
    const Eigen::Quaterniond attitude_before =
        processor.state().nominal.attitude_body_to_enu;
    const Eigen::Vector3d accel_bias_before = processor.state().nominal.accel_bias;
    const Eigen::Vector3d gyro_bias_before = processor.state().nominal.gyro_bias;
    const Eigen::Matrix3d attitude_covariance =
        before_rejections.covariance.block<3, 3>(fusion_index::ATTITUDE,
                                                 fusion_index::ATTITUDE);
    const Eigen::Matrix3d position_attitude_jacobian =
        -rotation * attitude::skew(config.lever_arm_body);
    const Eigen::Matrix3d expected_position_covariance =
        0.01 * Eigen::Matrix3d::Identity() +
        position_attitude_jacobian * attitude_covariance *
            position_attitude_jacobian.transpose();

    fixed_outlier.time = fixed_outlier.time + kDt;
    processor.processGnssSolution(fixed_outlier);
    ASSERT_TRUE(processor.lastGnssPositionUpdateApplied());
    ASSERT_TRUE(processor.lastGnssPositionReanchored());
    EXPECT_TRUE(processor.state().nominal.position_enu.isApprox(
        desired_antenna_position_enu - rotation * config.lever_arm_body, 1e-10));
    EXPECT_TRUE(processor.lastGnssPositionCorrectionEnu().isApprox(
        processor.state().nominal.position_enu - before_rejections.nominal.position_enu,
        1e-10));
    EXPECT_TRUE(processor.state().nominal.velocity_enu.isApprox(velocity_before, 1e-12));
    EXPECT_TRUE(processor.state().nominal.attitude_body_to_enu.coeffs().isApprox(
        attitude_before.coeffs(), 1e-12));
    EXPECT_TRUE(processor.state().nominal.accel_bias.isApprox(accel_bias_before, 1e-12));
    EXPECT_TRUE(processor.state().nominal.gyro_bias.isApprox(gyro_bias_before, 1e-12));
    EXPECT_TRUE((processor.state().covariance.block<3, 3>(fusion_index::POSITION,
                                                          fusion_index::POSITION)
                     .isApprox(expected_position_covariance, 1e-9)));
    EXPECT_TRUE((processor.state().covariance.block<3, 12>(fusion_index::POSITION, 3)
                     .isZero(1e-12)));
    EXPECT_TRUE((processor.state().covariance.block<12, 3>(3, fusion_index::POSITION)
                     .isZero(1e-12)));
    EXPECT_TRUE(processor.state().covariance.allFinite());

    // A following same-position FIXED solution is handled by the ordinary
    // gated update after the counter reset, not by another forced correction.
    fixed_outlier.time = fixed_outlier.time + kDt;
    processor.processGnssSolution(fixed_outlier);
    EXPECT_TRUE(processor.lastGnssPositionUpdateApplied());
    EXPECT_FALSE(processor.lastGnssPositionReanchored());

    // A velocity gate rejection never invokes the position re-anchor path and
    // does not force an update after its own consecutive-rejection threshold.
    const Eigen::Vector3d velocity_before_rejection = processor.state().nominal.velocity_enu;
    PositionSolution velocity_outlier = fixed_outlier;
    velocity_outlier.has_velocity = true;
    velocity_outlier.velocity_ecef =
        enu_to_ecef * Eigen::Vector3d(10.0, 0.0, 0.0);
    velocity_outlier.velocity_covariance = 0.01 * Eigen::Matrix3d::Identity();
    velocity_outlier.time = velocity_outlier.time + kDt;
    processor.processGnssSolution(velocity_outlier);
    velocity_outlier.time = velocity_outlier.time + kDt;
    processor.processGnssSolution(velocity_outlier);
    EXPECT_FALSE(processor.lastGnssPositionReanchored());
    EXPECT_TRUE(processor.state().nominal.velocity_enu.isApprox(
        velocity_before_rejection, 1e-10));

    // Arm one FIX rejection, then send FLOAT positions. FLOAT positions are
    // never eligible for recovery, and must reset the FIX patience even when
    // position_updates_require_fixed skips their normal EKF update entirely.
    PositionSolution fixed_before_float = fixed_outlier;
    fixed_before_float.position_ecef += enu_to_ecef * Eigen::Vector3d(8.0, 0.0, 0.0);
    fixed_before_float.position_covariance = 0.01 * Eigen::Matrix3d::Identity();
    fixed_before_float.time = fixed_outlier.time + kDt;
    processor.processGnssSolution(fixed_before_float);
    EXPECT_FALSE(processor.lastGnssPositionReanchored());

    // FLOAT positions are never eligible for the position-only recovery,
    // even after the same consecutive-rejection threshold is reached.
    PositionSolution float_outlier = fixed_outlier;
    float_outlier.status = SolutionStatus::FLOAT;
    float_outlier.position_ecef += enu_to_ecef * Eigen::Vector3d(8.0, 0.0, 0.0);
    float_outlier.position_covariance = 0.01 * Eigen::Matrix3d::Identity();
    const Eigen::Vector3d position_before_float = processor.state().nominal.position_enu;
    for (int i = 0; i < 2; ++i) {
        float_outlier.time = float_outlier.time + kDt;
        processor.processGnssSolution(float_outlier);
    }
    EXPECT_FALSE(processor.lastGnssPositionUpdateApplied());
    EXPECT_FALSE(processor.lastGnssPositionReanchored());
    EXPECT_TRUE(processor.state().nominal.position_enu.isApprox(position_before_float, 1e-12));

    // One FIX rejection after FLOAT must only arm the counter; a stale
    // pre-FLOAT rejection would have incorrectly triggered a re-anchor here.
    float_outlier.status = SolutionStatus::FIXED;
    float_outlier.time = float_outlier.time + kDt;
    processor.processGnssSolution(float_outlier);
    EXPECT_FALSE(processor.lastGnssPositionReanchored());
}

TEST(FusionProcessorSyntheticTest, FixedPositionReanchorDisabledWhenPatienceIsZero) {
    LooseCouplingProcessor::Config config;
    config.align_static_window_s = 0.1;
    config.zupt_enable = false;
    config.max_position_update_nis_per_observation = 0.1;
    config.max_consecutive_gate_rejections = 0;
    LooseCouplingProcessor processor(config);

    GNSSTime time(2200, 100000.0);
    for (int i = 0; i < 20; ++i) {
        ImuSample sample;
        sample.time = time;
        sample.accel_raw = Eigen::Vector3d(0.0, 0.0, kGravity);
        processor.processImuSample(sample);
        time = time + kDt;
    }
    ASSERT_TRUE(processor.isInitialized());

    PositionSolution solution;
    solution.time = time;
    solution.status = SolutionStatus::FIXED;
    solution.num_satellites = 10;
    const Eigen::Vector3d origin_ecef =
        geodetic2ecef(35.6 * M_PI / 180.0, 139.7 * M_PI / 180.0, 50.0);
    solution.position_ecef = origin_ecef;
    solution.position_covariance = Eigen::Matrix3d::Identity();
    processor.processGnssSolution(solution);
    const Eigen::Vector3d position_before = processor.state().nominal.position_enu;

    solution.position_ecef += processor.ecefToLocalEnuRotation().transpose() *
                             Eigen::Vector3d(5.0, 0.0, 0.0);
    solution.position_covariance = 0.01 * Eigen::Matrix3d::Identity();
    for (int i = 0; i < 3; ++i) {
        solution.time = solution.time + kDt;
        processor.processGnssSolution(solution);
    }
    EXPECT_FALSE(processor.lastGnssPositionUpdateApplied());
    EXPECT_FALSE(processor.lastGnssPositionReanchored());
    EXPECT_TRUE(processor.state().nominal.position_enu.isApprox(position_before, 1e-12));
}

TEST(FusionProcessorSyntheticTest, VelocityGateReanchorsVelocityOnlyWithLeverArm) {
    LooseCouplingProcessor::Config config;
    config.align_static_window_s = 0.1;
    config.zupt_enable = false;
    config.nhc_enable = false;
    config.lever_arm_body = Eigen::Vector3d(0.31, 0.0, -0.55);
    config.max_position_update_nis_per_observation = 0.0;
    config.max_velocity_update_nis_per_observation = 0.1;
    config.max_consecutive_velocity_gate_rejections = 2;
    config.max_gnss_velocity_reanchor_mps = 5.0;
    config.position_updates_require_fixed = true;
    config.align_velocity_threshold_mps = 100.0;
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

    const Eigen::Vector3d origin_ecef =
        geodetic2ecef(35.6 * M_PI / 180.0, 139.7 * M_PI / 180.0, 50.0);
    PositionSolution anchor;
    anchor.time = time;
    anchor.status = SolutionStatus::FIXED;
    anchor.num_satellites = 10;
    anchor.position_ecef = origin_ecef;
    anchor.position_covariance = 100.0 * Eigen::Matrix3d::Identity();
    processor.processGnssSolution(anchor);
    ASSERT_TRUE(processor.isOriginSet());

    const Eigen::Vector3d angular_rate_body(0.0, 0.0, 0.2);
    ImuSample motion;
    motion.time = time + kDt;
    motion.accel_raw = Eigen::Vector3d(0.0, 0.0, kGravity);
    motion.gyro_raw_radps = angular_rate_body;
    processor.processImuSample(motion);

    const FusionState before = processor.state();
    const Eigen::Matrix3d ecef_to_enu = processor.ecefToLocalEnuRotation();
    const Eigen::Matrix3d enu_to_ecef = ecef_to_enu.transpose();
    const Eigen::Matrix3d rotation =
        before.nominal.attitude_body_to_enu.toRotationMatrix();
    const Eigen::Vector3d target_velocity_enu(2.0, -1.0, 0.5);
    const Eigen::Vector3d corrected_angular_rate_body =
        angular_rate_body - before.nominal.gyro_bias;
    const Eigen::Vector3d lever_velocity_body =
        corrected_angular_rate_body.cross(config.lever_arm_body);

    PositionSolution velocity_solution = anchor;
    velocity_solution.status = SolutionStatus::FLOAT;
    velocity_solution.time = motion.time + kDt;
    velocity_solution.position_ecef = origin_ecef + enu_to_ecef *
        (before.nominal.position_enu + rotation * config.lever_arm_body);
    velocity_solution.position_covariance = 100.0 * Eigen::Matrix3d::Identity();
    velocity_solution.velocity_ecef = enu_to_ecef *
        (target_velocity_enu + rotation * lever_velocity_body);
    velocity_solution.velocity_covariance =
        0.01 * Eigen::Matrix3d::Identity();
    velocity_solution.has_velocity = true;

    processor.processGnssSolution(velocity_solution);
    EXPECT_FALSE(processor.lastGnssVelocityReanchored());
    EXPECT_TRUE(processor.state().nominal.velocity_enu.isApprox(
        before.nominal.velocity_enu, 1e-8))
        << "actual=" << processor.state().nominal.velocity_enu.transpose()
        << " before=" << before.nominal.velocity_enu.transpose();

    const Eigen::Vector3d position_before = processor.state().nominal.position_enu;
    const Eigen::Quaterniond attitude_before =
        processor.state().nominal.attitude_body_to_enu;
    const Eigen::Vector3d accel_bias_before = processor.state().nominal.accel_bias;
    const Eigen::Vector3d gyro_bias_before = processor.state().nominal.gyro_bias;
    velocity_solution.time = velocity_solution.time + kDt;
    processor.processGnssSolution(velocity_solution);

    ASSERT_TRUE(processor.lastGnssVelocityReanchored());
    EXPECT_TRUE(processor.state().nominal.velocity_enu.isApprox(
        target_velocity_enu, 1e-10));
    EXPECT_TRUE(processor.lastGnssVelocityCorrectionEnu().isApprox(
        target_velocity_enu - before.nominal.velocity_enu, 1e-10));
    EXPECT_TRUE(processor.state().nominal.position_enu.isApprox(
        position_before, 1e-8));
    EXPECT_TRUE(processor.state().nominal.attitude_body_to_enu.coeffs().isApprox(
        attitude_before.coeffs(), 1e-8));
    EXPECT_TRUE(processor.state().nominal.accel_bias.isApprox(accel_bias_before, 1e-8));
    EXPECT_TRUE(processor.state().nominal.gyro_bias.isApprox(gyro_bias_before, 1e-6));

    const Eigen::Matrix3d attitude_covariance =
        before.covariance.block<3, 3>(fusion_index::ATTITUDE,
                                      fusion_index::ATTITUDE);
    const Eigen::Matrix3d velocity_attitude_jacobian =
        -rotation * attitude::skew(lever_velocity_body);
    const Eigen::Matrix3d expected_velocity_covariance =
        0.01 * Eigen::Matrix3d::Identity() +
        velocity_attitude_jacobian * attitude_covariance *
            velocity_attitude_jacobian.transpose();
    const Eigen::Matrix3d velocity_covariance_after =
        processor.state().covariance.block(
            fusion_index::VELOCITY, fusion_index::VELOCITY, 3, 3);
    const Eigen::MatrixXd velocity_cross_position_after =
        processor.state().covariance.block(
            fusion_index::VELOCITY, fusion_index::POSITION, 3, 3);
    const Eigen::MatrixXd velocity_cross_attitude_bias_after =
        processor.state().covariance.block(
            fusion_index::VELOCITY, fusion_index::ATTITUDE, 3, 9);
    const Eigen::MatrixXd velocity_cross_position_transpose_after =
        processor.state().covariance.block(
            fusion_index::POSITION, fusion_index::VELOCITY, 3, 3);
    const Eigen::MatrixXd velocity_cross_attitude_bias_transpose_after =
        processor.state().covariance.block(
            fusion_index::ATTITUDE, fusion_index::VELOCITY, 9, 3);
    EXPECT_TRUE(velocity_covariance_after.isApprox(
        expected_velocity_covariance, 1e-9));
    EXPECT_TRUE(velocity_cross_position_after.isZero(1e-12));
    EXPECT_TRUE(velocity_cross_attitude_bias_after.isZero(1e-12));
    EXPECT_TRUE(velocity_cross_position_transpose_after.isZero(1e-12));
    EXPECT_TRUE(velocity_cross_attitude_bias_transpose_after.isZero(1e-12));
    EXPECT_TRUE(processor.state().covariance.allFinite());
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> velocity_covariance_eig(
        velocity_covariance_after);
    ASSERT_EQ(velocity_covariance_eig.info(), Eigen::Success);
    EXPECT_GE(velocity_covariance_eig.eigenvalues().minCoeff(), 0.0);

    // A malformed Doppler covariance cannot be promoted to a recovery target
    // merely because regularizeCovariance3x3() has a conservative fallback.
    velocity_solution.velocity_covariance(0, 0) =
        std::numeric_limits<double>::quiet_NaN();
    velocity_solution.time = velocity_solution.time + kDt;
    processor.processGnssSolution(velocity_solution);
    EXPECT_FALSE(processor.lastGnssVelocityReanchored());
}

TEST(FusionProcessorSyntheticTest, FixedPositionReanchorRejectsUnboundedCorrection) {
    LooseCouplingProcessor::Config config;
    config.align_static_window_s = 0.1;
    config.zupt_enable = false;
    config.max_position_update_nis_per_observation = 0.1;
    config.max_consecutive_gate_rejections = 2;
    config.max_fixed_position_reanchor_m = 1.0;
    LooseCouplingProcessor processor(config);

    GNSSTime time(2200, 100000.0);
    for (int i = 0; i < 20; ++i) {
        ImuSample sample;
        sample.time = time;
        sample.accel_raw = Eigen::Vector3d(0.0, 0.0, kGravity);
        processor.processImuSample(sample);
        time = time + kDt;
    }
    ASSERT_TRUE(processor.isInitialized());

    PositionSolution solution;
    solution.time = time;
    solution.status = SolutionStatus::FIXED;
    solution.num_satellites = 10;
    const Eigen::Vector3d origin_ecef =
        geodetic2ecef(35.6 * M_PI / 180.0, 139.7 * M_PI / 180.0, 50.0);
    solution.position_ecef = origin_ecef;
    solution.position_covariance = Eigen::Matrix3d::Identity();
    processor.processGnssSolution(solution);
    const Eigen::Vector3d position_before = processor.state().nominal.position_enu;

    solution.position_ecef += processor.ecefToLocalEnuRotation().transpose() *
                             Eigen::Vector3d(5.0, 0.0, 0.0);
    solution.position_covariance = 0.01 * Eigen::Matrix3d::Identity();
    for (int i = 0; i < 2; ++i) {
        solution.time = solution.time + kDt;
        processor.processGnssSolution(solution);
    }
    EXPECT_FALSE(processor.lastGnssPositionUpdateApplied());
    EXPECT_FALSE(processor.lastGnssPositionReanchored());
    EXPECT_TRUE(processor.state().nominal.position_enu.isApprox(position_before, 1e-12));
}

TEST(FusionProcessorSyntheticTest, AppliesTightlyCoupledDDRowsToLiveINSState) {
    LooseCouplingProcessor::Config config;
    config.align_static_window_s = 0.1;
    config.zupt_enable = false;
    // This test exercises the committed joint code/carrier path explicitly;
    // production remains shadow-only unless this research flag is enabled.
    config.tight_dd_commit_carrier_updates = true;
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
    row.wavelength_m = 0.19;
    // Keep the synthetic ambiguity exactly integral so the test isolates the
    // processor integration path instead of exercising an innovation boundary.
    row.carrier_residual_m = row.wavelength_m * 12.0;
    row.carrier_variance_m2 = 0.0025;
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
