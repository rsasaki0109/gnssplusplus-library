#include <gtest/gtest.h>

#include <libgnss++/fusion/fusion_processor.hpp>

#include <cmath>
#include <limits>
#include <memory>
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

// velocity_consistency_v2: lockout-proof recovery of the precise-class
// position gate. Shared harness: zero lever arm so antenna == IMU position.
class FloatGateRecoveryHarness {
public:
    explicit FloatGateRecoveryHarness(int reanchor_after, double gap_reanchor_s = 0.0,
                                      bool require_prefit_gate_pass = false) {
        config_.align_static_window_s = 0.1;
        config_.zupt_enable = false;
        config_.nhc_enable = false;
        config_.lever_arm_body.setZero();
        config_.max_position_update_nis_per_observation = 9.0;
        config_.max_consecutive_gate_rejections = 0;  // existing FIXED path off
        config_.float_position_reanchor_after_rejections = reanchor_after;
        config_.position_reanchor_after_gnss_gap_s = gap_reanchor_s;
        config_.reanchor_requires_prefit_gate_pass = require_prefit_gate_pass;
        processor_ = std::make_unique<LooseCouplingProcessor>(config_);
        for (int i = 0; i < 20; ++i) {
            ImuSample sample;
            sample.time = time_;
            sample.accel_raw = Eigen::Vector3d(0.0, 0.0, kGravity);
            sample.gyro_raw_radps.setZero();
            processor_->processImuSample(sample);
            time_ = time_ + kDt;
        }
        origin_ecef_ = geodetic2ecef(35.6 * M_PI / 180.0, 139.7 * M_PI / 180.0, 50.0);
        // Origin-setting coarse epoch at the state position.
        send(SolutionStatus::SPP, Eigen::Vector3d::Zero(), 5.0);
    }
    // ENU antenna offset from the origin -> solution of the given class.
    void send(SolutionStatus status, const Eigen::Vector3d& enu, double sigma_m,
              bool prefit_gate_exceeded = false) {
        time_ = time_ + 0.2;
        PositionSolution solution;
        solution.float_prefit_gate_exceeded = prefit_gate_exceeded;
        solution.time = time_;
        solution.status = status;
        solution.num_satellites = 10;
        solution.position_covariance = sigma_m * sigma_m * Eigen::Matrix3d::Identity();
        const Eigen::Matrix3d enu_to_ecef = rotationOrIdentity().transpose();
        solution.position_ecef = origin_ecef_ + enu_to_ecef * enu;
        processor_->processGnssSolution(solution);
    }
    void skip(double seconds) { time_ = time_ + seconds; }
    LooseCouplingProcessor& processor() { return *processor_; }
    Eigen::Vector3d antennaEnu() const { return processor_->state().nominal.position_enu; }

private:
    Eigen::Matrix3d rotationOrIdentity() const {
        return processor_ ? processor_->ecefToLocalEnuRotation() : Eigen::Matrix3d::Identity();
    }
    LooseCouplingProcessor::Config config_;
    std::unique_ptr<LooseCouplingProcessor> processor_;
    GNSSTime time_{2200, 100000.0};
    Eigen::Vector3d origin_ecef_ = Eigen::Vector3d::Zero();
};

TEST(FusionProcessorSyntheticTest, FloatGateRecoveryDefaultsOffAndKeepsFloatRejected) {
    EXPECT_EQ(LooseCouplingProcessor::Config().float_position_reanchor_after_rejections, 0);
    FloatGateRecoveryHarness h(0);
    ASSERT_TRUE(h.processor().isOriginSet());
    for (int i = 0; i < 8; ++i) {
        h.send(SolutionStatus::SPP, Eigen::Vector3d(12.0, 0.0, 0.0), 5.0);
        h.send(SolutionStatus::FLOAT, Eigen::Vector3d(25.0, 0.0, 0.0), 0.1);
        EXPECT_FALSE(h.processor().lastGnssPositionUpdateApplied()) << i;
        EXPECT_FALSE(h.processor().lastGnssPositionReanchored()) << i;
    }
}

TEST(FusionProcessorSyntheticTest, FloatGateRecoveryReanchorsDespiteInterleavedCoarseAcceptance) {
    FloatGateRecoveryHarness h(3);
    ASSERT_TRUE(h.processor().isOriginSet());
    const Eigen::Vector3d float_enu(25.0, 0.0, 0.0);
    for (int i = 0; i < 2; ++i) {
        h.send(SolutionStatus::SPP, Eigen::Vector3d(12.0, 0.0, 0.0), 5.0);
        // The coarse update is accepted and resets the shared streak counter,
        // which is exactly what starves the existing recovery.
        EXPECT_TRUE(h.processor().lastGnssPositionUpdateApplied());
        h.send(SolutionStatus::FLOAT, float_enu, 0.1);
        EXPECT_FALSE(h.processor().lastGnssPositionUpdateApplied()) << i;
        EXPECT_FALSE(h.processor().lastGnssPositionReanchored()) << i;
    }
    const auto before = h.processor().state();
    h.send(SolutionStatus::SPP, Eigen::Vector3d(12.0, 0.0, 0.0), 5.0);
    const Eigen::Vector3d attitude_before_vel = h.processor().state().nominal.velocity_enu;
    const Eigen::Quaterniond attitude_before = h.processor().state().nominal.attitude_body_to_enu;
    const Eigen::Vector3d gyro_before = h.processor().state().nominal.gyro_bias;
    h.send(SolutionStatus::FLOAT, float_enu, 0.1);
    EXPECT_TRUE(h.processor().lastGnssPositionReanchored());
    EXPECT_TRUE(h.processor().lastGnssPositionUpdateApplied());
    EXPECT_NEAR((h.antennaEnu() - float_enu).norm(), 0.0, 1e-6);
    // Position only: velocity, attitude and biases untouched, position
    // cross-covariances cleared, position covariance = GNSS covariance.
    EXPECT_TRUE(h.processor().state().nominal.velocity_enu.isApprox(attitude_before_vel, 1e-12));
    EXPECT_TRUE(h.processor().state().nominal.attitude_body_to_enu.isApprox(attitude_before, 1e-12));
    EXPECT_TRUE(h.processor().state().nominal.gyro_bias.isApprox(gyro_before, 1e-12));
    const auto& cov = h.processor().state().covariance;
    constexpr int p = fusion_index::POSITION;
    const double cross_norm = cov.block<3, 12>(p, 3).norm();
    const double position_variance = cov(p, p);
    EXPECT_NEAR(cross_norm, 0.0, 1e-12);
    EXPECT_NEAR(position_variance, 0.01, 0.01);
    (void)before;
    // Counter restarted: the next rejected FLOAT does not re-anchor again
    // (state is now at the FLOAT position, so move the FLOAT away again).
    h.send(SolutionStatus::SPP, float_enu, 5.0);
    h.send(SolutionStatus::FLOAT, float_enu + Eigen::Vector3d(30.0, 0.0, 0.0), 0.1);
    EXPECT_FALSE(h.processor().lastGnssPositionReanchored());
}

TEST(FusionProcessorSyntheticTest, FloatGateRecoveryRefusesFloatInconsistentWithCoarse) {
    FloatGateRecoveryHarness h(3);
    // A drifting float, 140 m from both the state and the coarse position.
    for (int i = 0; i < 8; ++i) {
        h.send(SolutionStatus::SPP, Eigen::Vector3d(2.0, 0.0, 0.0), 5.0);
        h.send(SolutionStatus::FLOAT, Eigen::Vector3d(140.0, 0.0, 0.0), 0.1);
        EXPECT_FALSE(h.processor().lastGnssPositionReanchored()) << i;
        EXPECT_FALSE(h.processor().lastGnssPositionUpdateApplied()) << i;
    }
    EXPECT_LT(h.antennaEnu().norm(), 20.0);
}

TEST(FusionProcessorSyntheticTest, FloatGateRecoveryNeedsRecentCoarseAndAcceptedFloatResetsStreak) {
    FloatGateRecoveryHarness stale(2);
    stale.skip(5.0);  // coarse origin epoch is now stale (> 1 s)
    for (int i = 0; i < 6; ++i) {
        stale.send(SolutionStatus::FLOAT, Eigen::Vector3d(25.0, 0.0, 0.0), 0.1);
        stale.skip(0.8);  // 1 s cadence, never a fresh coarse epoch
        EXPECT_FALSE(stale.processor().lastGnssPositionReanchored()) << i;
    }
    FloatGateRecoveryHarness reset(3);
    for (int round = 0; round < 3; ++round) {
        reset.send(SolutionStatus::SPP, Eigen::Vector3d(12.0, 0.0, 0.0), 5.0);
        reset.send(SolutionStatus::FLOAT, Eigen::Vector3d(25.0, 0.0, 0.0), 0.1);
        EXPECT_FALSE(reset.processor().lastGnssPositionReanchored());
        reset.send(SolutionStatus::SPP, Eigen::Vector3d(12.0, 0.0, 0.0), 5.0);
        reset.send(SolutionStatus::FLOAT, Eigen::Vector3d(25.0, 0.0, 0.0), 0.1);
        EXPECT_FALSE(reset.processor().lastGnssPositionReanchored());
        // A FLOAT the gate accepts (consistent with the state) resets the streak.
        reset.send(SolutionStatus::FLOAT, reset.antennaEnu(), 1.0);
        EXPECT_TRUE(reset.processor().lastGnssPositionUpdateApplied());
        EXPECT_FALSE(reset.processor().lastGnssPositionReanchored());
    }
}

// velocity_consistency_v4: a FLOAT/FIXED position rejected by the NIS gate after
// a GNSS-absence longer than the configured horizon re-anchors at once; the
// same rejection in steady state does not.
TEST(FusionProcessorSyntheticTest, PostGapReanchorDefaultsOff) {
    EXPECT_EQ(LooseCouplingProcessor::Config().position_reanchor_after_gnss_gap_s, 0.0);
    FloatGateRecoveryHarness h(0);
    h.send(SolutionStatus::SPP, Eigen::Vector3d(0.0, 0.0, 0.0), 5.0);
    h.skip(10.0);
    h.send(SolutionStatus::FLOAT, Eigen::Vector3d(25.0, 0.0, 0.0), 0.1);
    EXPECT_FALSE(h.processor().lastGnssPositionUpdateApplied());
    EXPECT_FALSE(h.processor().lastGnssPositionReanchored());
}

TEST(FusionProcessorSyntheticTest, PostGapReanchorAppliesOnlyAfterTheHorizon) {
    FloatGateRecoveryHarness h(0, 1.0);
    // Steady state: SPP updates every 0.2 s keep the prior verified, so a
    // rejected FLOAT is not re-anchored (the existing NIS gate stays in force).
    for (int i = 0; i < 4; ++i) {
        h.send(SolutionStatus::SPP, Eigen::Vector3d(0.0, 0.0, 0.0), 5.0);
        EXPECT_TRUE(h.processor().lastGnssPositionUpdateApplied());
        h.send(SolutionStatus::FLOAT, Eigen::Vector3d(25.0, 0.0, 0.0), 0.1);
        EXPECT_FALSE(h.processor().lastGnssPositionUpdateApplied()) << i;
        EXPECT_FALSE(h.processor().lastGnssPositionReanchored()) << i;
    }
    // A silence shorter than the horizon (0.4 s + two 0.2 s send steps = 0.8 s
    // since the last accepted update) is not a gap.
    h.skip(0.4);
    h.send(SolutionStatus::FLOAT, Eigen::Vector3d(25.0, 0.0, 0.0), 0.1);
    EXPECT_FALSE(h.processor().lastGnssPositionReanchored());
    // A 10 s GNSS absence: the rejected FLOAT is trusted over the prior.
    h.skip(10.0);
    const Eigen::Vector3d velocity_before = h.processor().state().nominal.velocity_enu;
    const Eigen::Quaterniond attitude_before = h.processor().state().nominal.attitude_body_to_enu;
    h.send(SolutionStatus::FLOAT, Eigen::Vector3d(25.0, 0.0, 0.0), 0.1);
    EXPECT_TRUE(h.processor().lastGnssPositionReanchored());
    EXPECT_TRUE(h.processor().lastGnssPositionUpdateApplied());
    EXPECT_NEAR((h.antennaEnu() - Eigen::Vector3d(25.0, 0.0, 0.0)).norm(), 0.0, 1e-6);
    // Position only, covariance = the measurement's, cross terms cleared.
    EXPECT_TRUE(h.processor().state().nominal.velocity_enu.isApprox(velocity_before, 1e-12));
    EXPECT_TRUE(h.processor().state().nominal.attitude_body_to_enu.isApprox(attitude_before, 1e-12));
    const auto& cov = h.processor().state().covariance;
    constexpr int p = fusion_index::POSITION;
    const double cross_norm = cov.block<3, 12>(p, 3).norm();
    EXPECT_NEAR(cross_norm, 0.0, 1e-12);
    EXPECT_NEAR(cov(p, p), 0.01, 0.01);
    // The reference clock restarted: an immediate further rejected FLOAT is
    // steady state again and is not re-anchored.
    h.send(SolutionStatus::FLOAT, Eigen::Vector3d(60.0, 0.0, 0.0), 0.1);
    EXPECT_FALSE(h.processor().lastGnssPositionReanchored());
}

TEST(FusionProcessorSyntheticTest, PostGapReanchorIgnoresCoarseClassAndAcceptedUpdates) {
    FloatGateRecoveryHarness h(0, 1.0);
    h.skip(10.0);
    // A consistent FLOAT after the gap is an ordinary accepted update.
    h.send(SolutionStatus::FLOAT, h.antennaEnu(), 1.0);
    EXPECT_TRUE(h.processor().lastGnssPositionUpdateApplied());
    EXPECT_FALSE(h.processor().lastGnssPositionReanchored());
    // A rejected coarse (SPP) update after a gap is never re-anchored.
    h.skip(10.0);
    h.send(SolutionStatus::SPP, Eigen::Vector3d(500.0, 0.0, 0.0), 1.0);
    EXPECT_FALSE(h.processor().lastGnssPositionReanchored());
}

// velocity_consistency_v8 (l): reanchor_requires_prefit_gate_pass. Both
// re-anchors refuse a solution flagged float_prefit_gate_exceeded.
TEST(FusionProcessorSyntheticTest, ReanchorPrefitGateDefaultsOffAndFlagIsIgnored) {
    EXPECT_FALSE(LooseCouplingProcessor::Config().reanchor_requires_prefit_gate_pass);
    EXPECT_FALSE(PositionSolution{}.float_prefit_gate_exceeded);
    FloatGateRecoveryHarness h(0, 1.0);  // option off
    h.send(SolutionStatus::SPP, Eigen::Vector3d(0.0, 0.0, 0.0), 5.0);
    h.skip(10.0);
    h.send(SolutionStatus::FLOAT, Eigen::Vector3d(25.0, 0.0, 0.0), 0.1, /*prefit_gate_exceeded=*/true);
    EXPECT_TRUE(h.processor().lastGnssPositionReanchored());
    EXPECT_FALSE(h.processor().lastGnssPositionReanchorRefusedByPrefitGate());
    EXPECT_NEAR((h.antennaEnu() - Eigen::Vector3d(25.0, 0.0, 0.0)).norm(), 0.0, 1e-6);
}

TEST(FusionProcessorSyntheticTest, PostGapReanchorRefusesFlaggedSolutionOnlyWhenRequired) {
    FloatGateRecoveryHarness h(0, 1.0, /*require_prefit_gate_pass=*/true);
    h.send(SolutionStatus::SPP, Eigen::Vector3d(0.0, 0.0, 0.0), 5.0);
    h.skip(10.0);
    const Eigen::Vector3d before = h.antennaEnu();
    h.send(SolutionStatus::FLOAT, Eigen::Vector3d(25.0, 0.0, 0.0), 0.1, true);
    EXPECT_FALSE(h.processor().lastGnssPositionReanchored());
    EXPECT_FALSE(h.processor().lastGnssPositionUpdateApplied());
    EXPECT_TRUE(h.processor().lastGnssPositionReanchorRefusedByPrefitGate());
    EXPECT_LT((h.antennaEnu() - before).norm(), 1.0);
    // The gap is still open: an unflagged solution is re-anchored at once.
    h.send(SolutionStatus::FLOAT, Eigen::Vector3d(25.0, 0.0, 0.0), 0.1, false);
    EXPECT_TRUE(h.processor().lastGnssPositionReanchored());
    EXPECT_FALSE(h.processor().lastGnssPositionReanchorRefusedByPrefitGate());
    EXPECT_NEAR((h.antennaEnu() - Eigen::Vector3d(25.0, 0.0, 0.0)).norm(), 0.0, 1e-6);
}

TEST(FusionProcessorSyntheticTest, RejectionPatienceReanchorRefusesFlaggedSolutionOnlyWhenRequired) {
    for (const bool required : {false, true}) {
        FloatGateRecoveryHarness h(3, 0.0, required);
        const Eigen::Vector3d float_enu(25.0, 0.0, 0.0);
        for (int i = 0; i < 2; ++i) {
            h.send(SolutionStatus::SPP, Eigen::Vector3d(12.0, 0.0, 0.0), 5.0);
            h.send(SolutionStatus::FLOAT, float_enu, 0.1, true);
        }
        h.send(SolutionStatus::SPP, Eigen::Vector3d(12.0, 0.0, 0.0), 5.0);
        h.send(SolutionStatus::FLOAT, float_enu, 0.1, true);
        EXPECT_EQ(h.processor().lastGnssPositionReanchored(), !required) << required;
        EXPECT_EQ(h.processor().lastGnssPositionReanchorRefusedByPrefitGate(), required) << required;
        if (required) {
            // The streak is not consumed: the next unflagged solution re-anchors.
            h.send(SolutionStatus::SPP, Eigen::Vector3d(12.0, 0.0, 0.0), 5.0);
            h.send(SolutionStatus::FLOAT, float_enu, 0.1, false);
            EXPECT_TRUE(h.processor().lastGnssPositionReanchored());
            EXPECT_NEAR((h.antennaEnu() - float_enu).norm(), 0.0, 1e-6);
        }
    }
}

// velocity_consistency_v5: direction test at the heading latch. A paired
// "off"/"on" processor sees identical input; the body moves along +X
// (body_accel_x > 0) or -X (< 0) of the filter's own pre-motion attitude R0,
// so the GNSS course is the direction of travel and the true body +X course
// is the course of R0 * UnitX.
class LatchDirectionHarness {
public:
    LatchDirectionHarness(double body_accel_x, bool vibrate, bool zupt_enable)
        : body_accel_x_(body_accel_x), vibrate_(vibrate) {
        LooseCouplingProcessor::Config config;
        config.align_static_window_s = 0.5;
        config.zupt_enable = zupt_enable;
        config.lever_arm_body.setZero();
        off_ = std::make_unique<LooseCouplingProcessor>(config);
        config.heading_latch_direction_test = true;
        on_ = std::make_unique<LooseCouplingProcessor>(config);
    }

    // Runs until both processors latched (or 10 s) and records the body +X
    // course at each processor's own latch epoch.
    void run() {
        const double t0 = 100000.0;
        Eigen::Matrix3d r0 = Eigen::Matrix3d::Identity();
        for (int i = 0; i <= 1000; ++i) {
            const double t = i * kDt;
            const bool moving = t > 3.0 + 1e-9;
            if (!moving && i == 300) r0 = on_->state().nominal.attitude_body_to_enu.toRotationMatrix();
            const double a_x = moving ? body_accel_x_ : 0.0;
            ImuSample sample;
            sample.time = GNSSTime(2200, t0 + t);
            const double vib = vibrate_ ? ((i % 2 == 0) ? 1.5 : -1.5) : 0.0;
            sample.accel_raw = Eigen::Vector3d(a_x, 0.0, kGravity + vib);
            off_->processImuSample(sample);
            on_->processImuSample(sample);
            if (i % 10 != 0 || !on_->isInitialized()) continue;
            const double age = moving ? t - 3.0 : 0.0;
            const Eigen::Vector3d velocity_enu = r0 * Eigen::Vector3d(body_accel_x_ * age, 0.0, 0.0);
            const Eigen::Vector3d position_enu =
                r0 * Eigen::Vector3d(0.5 * body_accel_x_ * age * age, 0.0, 0.0);
            PositionSolution fix;
            fix.time = sample.time;
            fix.status = SolutionStatus::SPP;
            fix.num_satellites = 8;
            // ENU -> ECEF at lat 0, lon 0: east = y, north = z, up = x.
            fix.position_ecef = Eigen::Vector3d(6378137.0 + position_enu.z(), position_enu.x(), position_enu.y());
            fix.position_covariance = Eigen::Matrix3d::Identity();
            fix.has_velocity = true;
            fix.velocity_ecef = Eigen::Vector3d(velocity_enu.z(), velocity_enu.x(), velocity_enu.y());
            fix.velocity_covariance = 0.04 * Eigen::Matrix3d::Identity();
            const bool off_before = off_->isHeadingAligned();
            const bool on_before = on_->isHeadingAligned();
            off_->processGnssSolution(fix);
            on_->processGnssSolution(fix);
            if (!off_before && off_->isHeadingAligned()) {
                off_latch_time_ = sample.time.tow;
                off_course_deg_ = bodyXCourseDeg(*off_);
            }
            if (!on_before && on_->isHeadingAligned()) {
                on_latch_time_ = sample.time.tow;
                on_course_deg_ = bodyXCourseDeg(*on_);
            }
            if (!off_->isHeadingAligned() && !on_->isHeadingAligned()) {
                // Before the latch the option is observationally inert.
                EXPECT_EQ((off_->state().covariance - on_->state().covariance).norm(), 0.0);
                EXPECT_EQ((off_->state().nominal.velocity_enu - on_->state().nominal.velocity_enu).norm(), 0.0);
                EXPECT_EQ(off_->state().nominal.attitude_body_to_enu.coeffs(),
                          on_->state().nominal.attitude_body_to_enu.coeffs());
            }
            if (off_->isHeadingAligned() && on_->isHeadingAligned()) break;
        }
        true_body_x_course_deg_ = courseDeg(r0.col(0));
        travel_course_deg_ = courseDeg(body_accel_x_ >= 0.0 ? r0.col(0) : Eigen::Vector3d(-r0.col(0)));
    }

    static double courseDeg(const Eigen::Vector3d& enu) {
        return std::atan2(enu.x(), enu.y()) * 180.0 / M_PI;
    }
    static double bodyXCourseDeg(const LooseCouplingProcessor& processor) {
        return courseDeg(processor.state().nominal.attitude_body_to_enu.toRotationMatrix().col(0));
    }
    static double angleDiffDeg(double a, double b) { return std::remainder(a - b, 360.0); }

    LooseCouplingProcessor& off() { return *off_; }
    LooseCouplingProcessor& on() { return *on_; }
    double offCourseDeg() const { return off_course_deg_; }
    double onCourseDeg() const { return on_course_deg_; }
    double offLatchTime() const { return off_latch_time_; }
    double onLatchTime() const { return on_latch_time_; }
    double trueBodyXCourseDeg() const { return true_body_x_course_deg_; }
    double travelCourseDeg() const { return travel_course_deg_; }

private:
    double body_accel_x_;
    bool vibrate_;
    std::unique_ptr<LooseCouplingProcessor> off_, on_;
    double off_course_deg_ = std::numeric_limits<double>::quiet_NaN();
    double on_course_deg_ = std::numeric_limits<double>::quiet_NaN();
    double off_latch_time_ = -1.0, on_latch_time_ = -1.0;
    double true_body_x_course_deg_ = 0.0, travel_course_deg_ = 0.0;
};

TEST(FusionProcessorSyntheticTest, HeadingLatchDirectionTestDefaultsOff) {
    EXPECT_FALSE(LooseCouplingProcessor::Config().heading_latch_direction_test);
    LooseCouplingProcessor processor{LooseCouplingProcessor::Config()};
    EXPECT_FALSE(processor.longitudinalVelocityValid());
    EXPECT_EQ(processor.longitudinalVelocityMps(), 0.0);
    EXPECT_FALSE(processor.lastLatchDirectionFlipped());
    EXPECT_EQ(processor.latchDirectionFlipCount(), 0U);
}

TEST(FusionProcessorSyntheticTest, HeadingLatchDirectionTestFlipsAReverseStart) {
    for (const bool zupt_enable : {true, false}) {
        LatchDirectionHarness h(-1.0, /*vibrate=*/false, zupt_enable);
        h.run();
        ASSERT_TRUE(h.off().isHeadingAligned()) << zupt_enable;
        ASSERT_TRUE(h.on().isHeadingAligned()) << zupt_enable;
        // Latch time is unchanged by the option.
        EXPECT_EQ(h.offLatchTime(), h.onLatchTime());
        // Option off: heading = course of travel, 180 deg from the true heading.
        EXPECT_NEAR(LatchDirectionHarness::angleDiffDeg(h.offCourseDeg(), h.travelCourseDeg()), 0.0, 0.1);
        EXPECT_NEAR(std::abs(LatchDirectionHarness::angleDiffDeg(h.offCourseDeg(), h.trueBodyXCourseDeg())),
                    180.0, 0.1);
        // Option on: course + 180 deg, i.e. the true body +X direction.
        EXPECT_NEAR(LatchDirectionHarness::angleDiffDeg(h.onCourseDeg(), h.trueBodyXCourseDeg()), 0.0, 0.1);
        EXPECT_NEAR(std::abs(LatchDirectionHarness::angleDiffDeg(h.onCourseDeg(), h.offCourseDeg())), 180.0, 0.1);
        EXPECT_TRUE(h.on().longitudinalVelocityValid());
        EXPECT_LT(h.on().lastLatchLongitudinalVelocityMps(), -0.3);
        EXPECT_TRUE(h.on().lastLatchDirectionFlipped());
        EXPECT_EQ(h.on().latchDirectionFlipCount(), 1U);
        EXPECT_FALSE(h.off().lastLatchDirectionFlipped());
        EXPECT_EQ(h.off().latchDirectionFlipCount(), 0U);
        EXPECT_FALSE(h.off().longitudinalVelocityValid());
    }
}

TEST(FusionProcessorSyntheticTest, HeadingLatchDirectionTestKeepsAForwardStart) {
    LatchDirectionHarness h(1.0, /*vibrate=*/false, /*zupt_enable=*/true);
    h.run();
    ASSERT_TRUE(h.off().isHeadingAligned());
    ASSERT_TRUE(h.on().isHeadingAligned());
    EXPECT_EQ(h.offLatchTime(), h.onLatchTime());
    EXPECT_NEAR(LatchDirectionHarness::angleDiffDeg(h.offCourseDeg(), h.trueBodyXCourseDeg()), 0.0, 0.1);
    EXPECT_NEAR(LatchDirectionHarness::angleDiffDeg(h.onCourseDeg(), h.trueBodyXCourseDeg()), 0.0, 0.1);
    EXPECT_TRUE(h.on().longitudinalVelocityValid());
    EXPECT_GT(h.on().lastLatchLongitudinalVelocityMps(), 0.3);
    EXPECT_FALSE(h.on().lastLatchDirectionFlipped());
    EXPECT_EQ(h.on().latchDirectionFlipCount(), 0U);
    // With nothing flipped, the whole state is identical to the option off.
    EXPECT_EQ(h.off().state().nominal.attitude_body_to_enu.coeffs(),
              h.on().state().nominal.attitude_body_to_enu.coeffs());
    EXPECT_EQ((h.off().state().covariance - h.on().state().covariance).norm(), 0.0);
}

TEST(FusionProcessorSyntheticTest, HeadingLatchDirectionTestNeedsAStationarySampleAfterInitialization) {
    // Constant vibration keeps the ZUPT stationarity condition from ever
    // holding after initialization, so v_long is never validated and the
    // (otherwise negative) integral is not trusted: no flip.
    LatchDirectionHarness h(-1.0, /*vibrate=*/true, /*zupt_enable=*/true);
    h.run();
    ASSERT_TRUE(h.off().isHeadingAligned());
    ASSERT_TRUE(h.on().isHeadingAligned());
    EXPECT_FALSE(h.on().longitudinalVelocityValid());
    EXPECT_LT(h.on().longitudinalVelocityMps(), 0.0);
    EXPECT_FALSE(h.on().lastLatchDirectionFlipped());
    EXPECT_EQ(h.on().latchDirectionFlipCount(), 0U);
    EXPECT_TRUE(std::isnan(h.on().lastLatchLongitudinalVelocityMps()));
    EXPECT_NEAR(LatchDirectionHarness::angleDiffDeg(h.onCourseDeg(), h.offCourseDeg()), 0.0, 1e-9);
    EXPECT_EQ(h.offLatchTime(), h.onLatchTime());
}

TEST(FusionProcessorSyntheticTest, LongitudinalVelocityRemovesGravityOfATiltedMount) {
    // A constant tilted specific force (|f| = g) is gravity only: the
    // body-forward kinematic acceleration is 0, whatever the pitch. The
    // z-axis vibration keeps the stationarity condition from resetting v_long.
    LooseCouplingProcessor::Config config;
    config.align_static_window_s = 0.5;
    config.heading_latch_direction_test = true;
    LooseCouplingProcessor processor(config);
    const double pitch = 10.0 * M_PI / 180.0;
    for (int i = 0; i <= 500; ++i) {
        ImuSample sample;
        sample.time = GNSSTime(2200, 100000.0 + i * kDt);
        const double vib = (i % 2 == 0) ? 1.5 : -1.5;
        sample.accel_raw = Eigen::Vector3d(-kGravity * std::sin(pitch), 0.0, kGravity * std::cos(pitch) + vib);
        processor.processImuSample(sample);
    }
    ASSERT_TRUE(processor.isInitialized());
    EXPECT_FALSE(processor.longitudinalVelocityValid());
    EXPECT_NEAR(processor.longitudinalVelocityMps(), 0.0, 0.05);
}

// velocity_consistency_v6: the gyro-bias seed replaces only the window-mean
// gyro bias of the next static-window initialization.
namespace {
const Eigen::Vector3d kWindowGyro(0.012, -0.021, 0.0875);
// Stops at the initializing sample (stop_at_init) so the state compared is the
// initialization itself, before any propagation with the differing bias.
void feedTiltedWindow(LooseCouplingProcessor& processor, double t0, int samples,
                      const Eigen::Vector3d& gyro, bool stop_at_init = true) {
    // Tilted (roll ~ 5 deg, pitch ~ 3 deg) specific force with a nonzero
    // accel bias so attitude and accel bias are both nontrivial.
    const Eigen::Vector3d accel(-0.51, 0.86, 9.76);
    for (int i = 0; i < samples; ++i) {
        ImuSample sample;
        sample.time = GNSSTime(2200, t0 + i * kDt);
        sample.accel_raw = accel;
        sample.gyro_raw_radps = gyro;
        processor.processImuSample(sample);
        if (stop_at_init && processor.isInitialized()) return;
    }
}
LooseCouplingProcessor::Config seedConfig() {
    LooseCouplingProcessor::Config config;
    config.align_static_window_s = 0.5;
    config.zupt_enable = false;
    return config;
}
}  // namespace

TEST(FusionProcessorSyntheticTest, GyroBiasSeedIsOffByDefault) {
    LooseCouplingProcessor processor(seedConfig());
    EXPECT_FALSE(processor.hasPendingGyroBiasSeed());
    EXPECT_FALSE(processor.lastInitializationGyroBiasSeeded());
    EXPECT_FALSE(processor.lastInitializationWindowGyroBias().allFinite());
    feedTiltedWindow(processor, 100000.0, 60, kWindowGyro);
    ASSERT_TRUE(processor.isInitialized());
    EXPECT_FALSE(processor.lastInitializationGyroBiasSeeded());
    EXPECT_FALSE(processor.lastInitializationWindowGyroBias().allFinite());
    EXPECT_NEAR((processor.state().nominal.gyro_bias - kWindowGyro).norm(), 0.0, 1e-12);
}

TEST(FusionProcessorSyntheticTest, GyroBiasSeedReplacesWindowMeanOnly) {
    LooseCouplingProcessor control(seedConfig());
    LooseCouplingProcessor seeded(seedConfig());
    const Eigen::Vector3d seed(-0.0004, 0.0031, -0.0114);
    seeded.seedGyroBiasForNextInitialization(seed);
    EXPECT_TRUE(seeded.hasPendingGyroBiasSeed());
    EXPECT_FALSE(seeded.isInitialized());
    feedTiltedWindow(control, 100000.0, 60, kWindowGyro);
    feedTiltedWindow(seeded, 100000.0, 60, kWindowGyro);
    ASSERT_TRUE(control.isInitialized());
    ASSERT_TRUE(seeded.isInitialized());
    const auto& c = control.state().nominal;
    const auto& n = seeded.state().nominal;
    EXPECT_NEAR((c.gyro_bias - kWindowGyro).norm(), 0.0, 1e-12);
    EXPECT_EQ(n.gyro_bias, seed);
    // Everything else equals the unseeded initialization exactly.
    EXPECT_EQ(n.attitude_body_to_enu.coeffs(), c.attitude_body_to_enu.coeffs());
    EXPECT_EQ(n.accel_bias, c.accel_bias);
    EXPECT_GT(c.accel_bias.norm(), 1e-3);
    EXPECT_EQ(n.velocity_enu, c.velocity_enu);
    EXPECT_EQ(n.position_enu, c.position_enu);
    EXPECT_EQ(n.time.tow, c.time.tow);
    EXPECT_EQ(seeded.state().covariance, control.state().covariance);
    // Diagnostics: seeded, with the replaced window mean recorded.
    EXPECT_TRUE(seeded.lastInitializationGyroBiasSeeded());
    EXPECT_NEAR((seeded.lastInitializationWindowGyroBias() - kWindowGyro).norm(), 0.0, 1e-12);
    EXPECT_FALSE(control.lastInitializationGyroBiasSeeded());
}

TEST(FusionProcessorSyntheticTest, GyroBiasSeedIsConsumedByTheInitialization) {
    LooseCouplingProcessor processor(seedConfig());
    const Eigen::Vector3d seed(0.001, 0.002, 0.003);
    processor.seedGyroBiasForNextInitialization(seed);
    feedTiltedWindow(processor, 100000.0, 60, kWindowGyro);
    ASSERT_TRUE(processor.isInitialized());
    EXPECT_FALSE(processor.hasPendingGyroBiasSeed());
    EXPECT_EQ(processor.state().nominal.gyro_bias, seed);
    // A seed set after initialization is never applied to the running filter.
    const auto before = processor.state().nominal.gyro_bias;
    processor.seedGyroBiasForNextInitialization(Eigen::Vector3d(9.0, 9.0, 9.0));
    feedTiltedWindow(processor, 100001.0, 20, kWindowGyro, false);
    EXPECT_LT((processor.state().nominal.gyro_bias - before).norm(), 1e-3);
    // The next initialization of a fresh processor without a seed uses the
    // window mean (initialization is once per instance; a reset builds a new one).
    LooseCouplingProcessor next(seedConfig());
    feedTiltedWindow(next, 100000.0, 60, kWindowGyro);
    EXPECT_NEAR((next.state().nominal.gyro_bias - kWindowGyro).norm(), 0.0, 1e-12);
    EXPECT_FALSE(next.lastInitializationGyroBiasSeeded());
}

}  // namespace
}  // namespace libgnss
