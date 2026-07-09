#pragma once

#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/solution.hpp>
#include <libgnss++/core/types.hpp>
#include <libgnss++/io/imu.hpp>

#include <cstddef>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace libgnss {

/**
 * @brief Selects which numerical backend FGOProcessor::optimizeProblem uses.
 *
 * `Eigen` (default) is the native dense/sparse Gauss-Newton-ish solver
 * implemented directly in fgo.cpp. `GTSAM` delegates to a GTSAM
 * NonlinearFactorGraph + LevenbergMarquardtOptimizer built in
 * fgo_gtsam_backend.cpp (only available when the library is built with
 * GTSAM found, i.e. GNSSPP_HAS_GTSAM is defined). Selecting GTSAM when the
 * library was built without it is a no-op: optimizeProblem falls back to the
 * Eigen backend.
 */
enum class FGOBackend {
    Eigen,
    GTSAM,
};

/**
 * @brief Batch pseudorange factor-graph optimizer.
 *
 * This is the native Eigen backend for the GNSS FGO pipeline. It keeps the
 * factor/problem representation explicit so a GTSAM backend can be added later
 * without changing callers that prepare GNSS factors from RINEX/navigation data.
 */
class FGOProcessor {
public:
    struct FGOConfig {
        FGOBackend backend = FGOBackend::Eigen;
        int max_iterations = 8;
        double convergence_threshold_m = 1e-4;
        double relative_cost_convergence_threshold = 0.0;
        double absolute_cost_convergence_threshold = 0.0;
        double pseudorange_sigma_m = 3.0;
        double pseudorange_elevation_sigma_power = 1.0;
        double min_elevation_deg = 10.0;
        double min_snr_dbhz = 0.0;
        double double_difference_reference_min_snr_dbhz = -1.0;
        double double_difference_base_min_snr_dbhz = -1.0;
        int min_satellites_per_epoch = 4;
        int min_output_double_difference_carrier_factors_per_epoch = 0;
        double max_float_seed_position_divergence_m = 0.0;
        double max_float_position_jump_m = 0.0;

        bool use_spp_seed = true;
        bool use_pseudorange_factors = true;
        bool use_motion_factors = true;
        bool use_position_motion_factors = true;
        bool use_clock_motion_factors = true;
        bool use_tdcp_factors = true;
        bool use_carrier_phase_factors = false;
        bool use_double_difference_factors = false;
        bool use_single_difference_doppler_factors = false;
        bool use_single_difference_tdcp_factors = false;
        bool use_velocity_states = false;
        bool use_velocity_motion_factors = false;
        bool use_ambiguity_between_factors = false;
        bool linearize_double_difference_factors_at_seed = false;
        bool reset_double_difference_ambiguities_each_epoch = false;
        bool use_inter_system_biases = true;
        bool use_ambiguity_priors = true;
        bool fix_ambiguities = false;
        bool prefer_double_difference_ambiguity_fixing = true;
        bool use_lambda_ambiguity_fix = true;
        bool use_epoch_lambda_fixed_output = false;
        bool use_partial_lambda_ambiguity_fix = true;
        bool use_robust_loss = true;
        double motion_sigma_m = 50.0;
        double clock_motion_sigma_m = 300.0;
        double velocity_prior_sigma_mps = 100.0;
        double velocity_motion_sigma_m = 0.01;
        double ambiguity_between_sigma_cycles = 0.001;
        double position_prior_sigma_m = 0.0;
        double clock_prior_sigma_m = 0.0;
        double tdcp_sigma_m = 0.03;
        double carrier_phase_sigma_m = 0.01;
        double single_difference_doppler_sigma_mps = 0.2;
        double single_difference_tdcp_sigma_m = 0.003;
        double double_difference_pseudorange_sigma_m = 1.0;
        double double_difference_carrier_sigma_m = 0.02;
        double pseudorange_huber_threshold_sigma = 4.0;
        double carrier_phase_huber_threshold_sigma = 4.0;
        double tdcp_huber_threshold_sigma = 4.0;
        double ambiguity_prior_sigma_m = 1000.0;
        double fixed_ambiguity_sigma_m = 0.003;
        double ambiguity_fix_max_fractional_cycles = 0.15;
        double lambda_ratio_threshold = 3.0;
        int min_fixed_ambiguities = 4;
        int max_lambda_ambiguities = 12;
        double max_tdcp_gap_s = 2.0;
        double base_epoch_match_tolerance_s = 0.02;
        double base_interpolation_max_gap_s = 1.2;
        bool reject_tdcp_loss_of_lock = true;
        bool reject_rover_carrier_loss_of_lock = false;
        bool reject_tdcp_code_phase_jump = true;
        double tdcp_code_phase_jump_threshold_m = 10.0;

        bool use_ionosphere_model = true;
        bool use_troposphere_model = true;
        bool use_multi_constellation = true;
        bool collect_lambda_debug = false;

        // --- Phase 2 milestone 2a (docs/gtsam_backend_design.md) ---
        // When true AND backend == FGOBackend::GTSAM, the GTSAM backend keys
        // the rover state as a body gtsam::Pose3 (translation + attitude) per
        // epoch instead of a bare gtsam::Point3, and builds the DD factors
        // with the lever-arm '...FactorArm' variants (antenna_ecef =
        // pose.translation() + pose.rotation() * pose3_lever_arm_body_m).
        // Attitude is unobservable from GNSS DD alone (no IMU until
        // milestone 2b) and is pinned near its identity seed by a dedicated
        // rotation-only prior; only the recovered ANTENNA position is a
        // validated output of this mode. Ignored by the native Eigen backend
        // and by the GTSAM backend when false (existing Point3 path,
        // unchanged). Default OFF.
        bool use_pose3_state = false;
        // Body-frame (FLU) translation from the body/IMU origin to the GNSS
        // antenna, used only when use_pose3_state is true. Zero means the
        // Pose3 state's translation IS the antenna (degenerate lever arm).
        Vector3d pose3_lever_arm_body_m = Vector3d::Zero();

        // --- Phase 2 milestone 2b: IMU tight coupling ---
        // When true AND use_pose3_state AND the GTSAM backend AND the problem
        // carries a valid ImuInput, the GTSAM backend augments each epoch with
        // a Vector3 velocity node V(i) and an imuBias::ConstantBias node B(i),
        // links consecutive epochs with a gtsam::CombinedImuFactor built from
        // PreintegratedCombinedMeasurements, and interprets the per-epoch Pose3
        // as body-in-nav (local ENU) with the DD '...FactorArm' consuming it
        // through ecef_T_nav (the same Pose3 is shared with the IMU factor).
        // Attitude/velocity become observable, so the per-epoch 2a rotation pin
        // is dropped in favour of first-state priors (attitude/velocity/bias)
        // for gauge. Ignored unless use_pose3_state is also set. Default OFF.
        bool use_imu = false;

        // --- Phase 2 milestone 2c: incremental fixed-lag smoother ---
        // When true AND use_imu (implies use_pose3_state, GTSAM backend), the
        // GTSAM backend streams the graph through a
        // gtsam::IncrementalFixedLagSmoother instead of a single batch LM
        // solve: each epoch's Pose3/Vel/Bias + factors are added incrementally
        // with key timestamps, and states older than fixed_lag_smoother_lag_s
        // (relative to the newest epoch) are marginalized. This bounds memory
        // and makes per-epoch LAMBDA feasible at full-dataset scale (the batch
        // gtsam::Marginals hit bad_alloc at ~25k vars). Ignored unless use_imu.
        // Default OFF (batch LM path unchanged).
        bool use_fixed_lag_smoother = false;
        // Smoother lag in seconds. States (and ambiguity/clock nodes) whose
        // last timestamp is older than this window are marginalized out.
        double fixed_lag_smoother_lag_s = 5.0;

        // --- Phase 2 milestone 2d: NHC + ZUPT pseudo-measurements ---
        // Applied per-epoch in the IMU-coupled fixed-lag path (gated), mirror
        // inuex35 buildfactor/nhc.py + zupt.py. Default OFF.
        //
        // NHC: a ground vehicle's body-frame lateral (left) and vertical (up)
        // velocity is ~0. Binary factor on (Pose3(i), Vel(i)) with residual
        // [v_body.y, v_body.z] = [ (R^T v_nav).y, (R^T v_nav).z ]. Gated to
        // moving, non-turning epochs so it never fights legitimate lateral
        // motion.
        bool use_nhc = false;
        double nhc_min_speed_mps = 2.0;          ///< only apply NHC above this speed
        double nhc_max_yaw_rate_radps = 0.20;    ///< skip NHC when |yaw rate| exceeds this (turn)
        double nhc_sigma_lateral_mps = 0.3;      ///< body-lateral velocity sigma
        double nhc_sigma_vertical_mps = 0.2;     ///< body-vertical velocity sigma
        // ZUPT: when the epoch's IMU window is stationary, pin Vel(i) ~ 0 with a
        // PriorFactor. Stationary detection mirrors the Stage-1 ESKF detector
        // (accel_std / gyro_std / gyro_median of bias-referenced IMU samples).
        bool use_zupt = false;
        double zupt_sigma_mps = 0.05;            ///< zero-velocity prior sigma
        // Velocity gate (mirrors inuex35): only fire ZUPT when the current
        // (IMU-predicted) speed is already below this. Without it, a quiet
        // accelerometer during CONSTANT-VELOCITY cruising is misread as
        // stationary and ZUPT freezes the vehicle (observed: FLOAT max 94 m).
        // 0 disables the gate.
        double zupt_max_speed_mps = 0.5;
        double zupt_max_accel_std = 0.55;        ///< stationary gate: accel deviation std [m/s^2]
        double zupt_max_gyro_std = 0.030;        ///< stationary gate: gyro deviation std [rad/s]
        double zupt_max_gyro_median = 0.020;     ///< stationary gate: gyro deviation median [rad/s]
        int zupt_min_samples = 5;                ///< minimum IMU samples in window to test

        // --- Phase 2 milestone 2e: fix-and-hold ambiguity resolution ---
        // In the fixed-lag path, once an arc's DD ambiguity is validated-fixed
        // (LAMBDA ratio above ambiguity_hold_ratio_threshold), pin it in the
        // graph with a tight prior at the integer for the remainder of the arc.
        // Held ambiguities have ~zero variance, so subsequent per-epoch LAMBDA
        // over the remaining floats passes far more often (fix-rate lever) and
        // the held integers stabilize the float. Reset is automatic: a cycle
        // slip / gap makes the problem builder assign a NEW ambiguity index
        // (fgo.cpp segments arcs on loss_of_lock), which is not in the held set.
        // Gated; default OFF.
        bool use_ambiguity_hold = false;
        double ambiguity_hold_ratio_threshold = 3.0;  ///< min ratio to hold (stricter than fix)
        double ambiguity_hold_sigma_cycles = 1e-3;    ///< tight prior sigma at the held integer
        int ambiguity_hold_min_fixed = 4;             ///< min ambiguities in a passing epoch to hold
    };

    struct EpochSeed {
        GNSSTime time;
        Vector3d position_ecef = Vector3d::Zero();
        double receiver_clock_bias_m = 0.0;
    };

    struct ObservationModelDebug {
        double raw_pseudorange_m = 0.0;
        double raw_carrier_m = 0.0;
        double satellite_clock_m = 0.0;
        double ionosphere_delay_m = 0.0;
        double troposphere_delay_m = 0.0;
        double group_delay_m = 0.0;
        double corrected_pseudorange_m = 0.0;
        double corrected_carrier_m = 0.0;
        double geometric_range_m = 0.0;
        double elevation_rad = 0.0;
        double azimuth_rad = 0.0;
    };

    struct PseudorangeFactor {
        std::size_t epoch_index = 0;
        SatelliteId satellite;
        GNSSSystem clock_group = GNSSSystem::GPS;
        Vector3d satellite_position_ecef = Vector3d::Zero();
        double corrected_pseudorange_m = 0.0;
        double sigma_m = 1.0;
        double elevation_rad = 0.0;
    };

    struct TimeDifferencedCarrierFactor {
        std::size_t previous_epoch_index = 0;
        std::size_t current_epoch_index = 0;
        SatelliteId satellite;
        SignalType signal = SignalType::GPS_L1CA;
        Vector3d previous_satellite_position_ecef = Vector3d::Zero();
        Vector3d current_satellite_position_ecef = Vector3d::Zero();
        double delta_carrier_m = 0.0;
        double sigma_m = 0.03;
        double dt_s = 0.0;
    };

    struct SingleDifferenceDopplerFactor {
        std::size_t epoch_index = 0;
        SatelliteId satellite;
        SatelliteId reference_satellite;
        SignalType signal = SignalType::GPS_L1CA;
        Vector3d los = Vector3d::Zero();
        double residual_mps = 0.0;
        double sigma_mps = 0.2;
        double elevation_rad = 0.0;
    };

    struct SingleDifferenceTdcpFactor {
        std::size_t previous_epoch_index = 0;
        std::size_t current_epoch_index = 0;
        SatelliteId satellite;
        SatelliteId reference_satellite;
        SignalType signal = SignalType::GPS_L1CA;
        Vector3d previous_los = Vector3d::Zero();
        Vector3d los = Vector3d::Zero();
        double delta_carrier_m = 0.0;
        double sigma_m = 0.003;
        double elevation_rad = 0.0;
    };

    struct AmbiguityState {
        SatelliteId satellite;
        SatelliteId reference_satellite;
        SignalType signal = SignalType::GPS_L1CA;
        std::size_t segment_index = 0;
        double wavelength_m = 0.0;
        double initial_ambiguity_m = 0.0;
        bool is_double_difference = false;
    };

    struct CarrierPhaseFactor {
        std::size_t epoch_index = 0;
        std::size_t ambiguity_index = 0;
        SatelliteId satellite;
        GNSSSystem clock_group = GNSSSystem::GPS;
        SignalType signal = SignalType::GPS_L1CA;
        Vector3d satellite_position_ecef = Vector3d::Zero();
        double corrected_pseudorange_m = 0.0;
        double corrected_carrier_m = 0.0;
        double wavelength_m = 0.0;
        double sigma_m = 0.01;
        double elevation_rad = 0.0;
        bool has_carrier_phase = true;
        bool loss_of_lock = false;
        bool has_doppler_residual = false;
        double doppler_residual_mps = 0.0;
        double doppler_sigma_mps = 0.2;
        Vector3d los = Vector3d::Zero();
        ObservationModelDebug model_debug;
    };

    struct DoubleDifferencePseudorangeFactor {
        std::size_t epoch_index = 0;
        SatelliteId satellite;
        SatelliteId reference_satellite;
        SignalType signal = SignalType::GPS_L1CA;
        Vector3d rover_satellite_position_ecef = Vector3d::Zero();
        Vector3d rover_reference_position_ecef = Vector3d::Zero();
        Vector3d base_satellite_position_ecef = Vector3d::Zero();
        Vector3d base_reference_position_ecef = Vector3d::Zero();
        Vector3d base_position_ecef = Vector3d::Zero();
        double observed_dd_pseudorange_m = 0.0;
        double sigma_m = 1.0;
        double elevation_rad = 0.0;
        ObservationModelDebug rover_satellite_model;
        ObservationModelDebug rover_reference_model;
        ObservationModelDebug base_satellite_model;
        ObservationModelDebug base_reference_model;
    };

    struct DoubleDifferenceCarrierFactor {
        std::size_t epoch_index = 0;
        std::size_t ambiguity_index = 0;
        std::size_t reference_ambiguity_index = 0;
        bool use_ambiguity_difference = true;
        SatelliteId satellite;
        SatelliteId reference_satellite;
        SignalType signal = SignalType::GPS_L1CA;
        Vector3d rover_satellite_position_ecef = Vector3d::Zero();
        Vector3d rover_reference_position_ecef = Vector3d::Zero();
        Vector3d base_satellite_position_ecef = Vector3d::Zero();
        Vector3d base_reference_position_ecef = Vector3d::Zero();
        Vector3d base_position_ecef = Vector3d::Zero();
        double observed_dd_carrier_m = 0.0;
        double sigma_m = 0.02;
        double elevation_rad = 0.0;
        ObservationModelDebug rover_satellite_model;
        ObservationModelDebug rover_reference_model;
        ObservationModelDebug base_satellite_model;
        ObservationModelDebug base_reference_model;
    };

    struct AmbiguityBetweenFactor {
        std::size_t previous_epoch_index = 0;
        std::size_t current_epoch_index = 0;
        std::size_t previous_ambiguity_index = 0;
        std::size_t current_ambiguity_index = 0;
        SatelliteId satellite;
        SignalType signal = SignalType::GPS_L1CA;
        double sigma_m = 0.001;
    };

    struct FGOProblemDiagnostics {
        std::size_t input_epochs = 0;
        std::size_t seeded_epochs = 0;
        std::size_t skipped_epochs_without_seed = 0;
        std::size_t double_difference_matched_base_epochs = 0;
        std::size_t double_difference_interpolated_base_epochs = 0;
        std::size_t double_difference_candidate_pairs = 0;
        std::size_t double_difference_rejected_no_base_epoch = 0;
        std::size_t double_difference_rejected_no_reference = 0;
        std::size_t tdcp_candidate_pairs = 0;
        std::size_t tdcp_rejected_gap = 0;
        std::size_t tdcp_rejected_missing_previous = 0;
        std::size_t tdcp_rejected_loss_of_lock = 0;
        std::size_t tdcp_rejected_code_phase_jump = 0;
    };

    // --- Phase 2 milestone 2b: IMU preintegration inputs ---
    //
    // Continuous-time IMU noise densities + bias random-walk for the GTSAM
    // PreintegrationCombinedParams. Sigmas, not covariances; the backend
    // squares them. Defaults are a reasonable consumer/industrial MEMS grade
    // (order of the tokyo low-cost preset) and can be overridden by the caller.
    // Defaults are the (deliberately conservative) values validated on tokyo1
    // in milestone 2b; final tuning is deferred to 2c/2e.
    struct ImuNoiseParams {
        double accel_noise_sigma = 1.0e-1;      ///< accel white noise [m/s^2/sqrt(Hz)]
        double gyro_noise_sigma = 1.0e-2;       ///< gyro white noise [rad/s/sqrt(Hz)]
        double accel_bias_rw_sigma = 1.0e-2;    ///< accel bias random walk [m/s^3/sqrt(Hz)]
        double gyro_bias_rw_sigma = 1.0e-3;     ///< gyro bias random walk [rad/s^2/sqrt(Hz)]
        double integration_sigma = 1.0e-2;      ///< integration uncertainty [m/s/sqrt(Hz)]
        double gravity_mps2 = 9.80665;          ///< local gravity magnitude
    };

    // Everything the GTSAM backend needs to add IMU factors, computed by the
    // caller (harness): the local nav (ENU) frame definition, the per-sample
    // IMU stream already remapped to body FLU with gyro in rad/s, the initial
    // navigation state (from Stage-1 alignment) and its prior sigmas, and the
    // preintegration noise. The nav frame is ENU (Z-up); gravity points to -Z.
    struct ImuInput {
        bool valid = false;
        // Local ENU nav-frame origin: pose translations are expressed as the
        // body/IMU origin in this ENU frame, and ecef_T_nav maps them to ECEF.
        Vector3d nav_origin_ecef = Vector3d::Zero();
        double nav_origin_lat_rad = 0.0;
        double nav_origin_lon_rad = 0.0;
        // Time-sorted IMU samples, already body-FLU (ImuAxisConvention applied)
        // with gyro converted to rad/s. The backend preintegrates the samples
        // falling in each [epoch[i].time, epoch[i+1].time) interval.
        std::vector<ImuSample> samples_body_flu;
        // Initial navigation state (first epoch), nav = ENU frame. Attitude is
        // the body->nav rotation from Stage-1 static leveling + heading align.
        Matrix3d init_attitude_body_to_nav = Matrix3d::Identity();
        Vector3d init_velocity_nav = Vector3d::Zero();
        Vector3d init_accel_bias = Vector3d::Zero();
        Vector3d init_gyro_bias = Vector3d::Zero();
        // First-state prior sigmas (gauge/anchor for the IMU chain).
        double init_attitude_sigma_roll_pitch_rad = 0.02;
        double init_attitude_sigma_yaw_rad = 0.5;
        double init_velocity_sigma_mps = 0.5;
        double init_accel_bias_sigma = 0.05;
        double init_gyro_bias_sigma = 0.01;
        ImuNoiseParams noise;
    };

    struct FGOProblem {
        std::vector<EpochSeed> epochs;
        ImuInput imu;  ///< Milestone 2b IMU inputs (valid only when populated).
        std::vector<bool> clock_jumps;
        std::vector<PseudorangeFactor> pseudorange_factors;
        std::vector<TimeDifferencedCarrierFactor> tdcp_factors;
        std::vector<SingleDifferenceDopplerFactor> single_difference_doppler_factors;
        std::vector<SingleDifferenceTdcpFactor> single_difference_tdcp_factors;
        std::vector<AmbiguityState> ambiguity_states;
        std::vector<CarrierPhaseFactor> carrier_observations;
        std::vector<CarrierPhaseFactor> double_difference_pseudorange_observations;
        std::vector<CarrierPhaseFactor> double_difference_reference_observations;
        std::vector<CarrierPhaseFactor> carrier_phase_factors;
        std::vector<DoubleDifferencePseudorangeFactor> double_difference_pseudorange_factors;
        std::vector<DoubleDifferenceCarrierFactor> double_difference_carrier_factors;
        std::vector<AmbiguityBetweenFactor> ambiguity_between_factors;
        FGOProblemDiagnostics diagnostics;
    };

    struct FGODiagnostics {
        int iterations = 0;
        bool converged = false;
        std::size_t epochs = 0;
        std::size_t pseudorange_factors = 0;
        std::size_t tdcp_factors = 0;
        std::size_t single_difference_doppler_factors = 0;
        std::size_t single_difference_tdcp_factors = 0;
        std::size_t carrier_phase_factors = 0;
        std::size_t double_difference_pseudorange_factors = 0;
        std::size_t double_difference_carrier_factors = 0;
        std::size_t ambiguity_states = 0;
        std::size_t ambiguity_fix_candidates = 0;
        std::size_t lambda_ambiguity_candidates = 0;
        std::size_t lambda_ambiguity_used_candidates = 0;
        std::size_t lambda_ambiguity_attempts = 0;
        std::size_t fixed_ambiguities = 0;
        std::size_t tdcp_candidate_pairs = 0;
        std::size_t tdcp_rejected_gap = 0;
        std::size_t tdcp_rejected_missing_previous = 0;
        std::size_t tdcp_rejected_loss_of_lock = 0;
        std::size_t tdcp_rejected_code_phase_jump = 0;
        std::size_t double_difference_matched_base_epochs = 0;
        std::size_t double_difference_interpolated_base_epochs = 0;
        std::size_t double_difference_candidate_pairs = 0;
        std::size_t double_difference_rejected_no_base_epoch = 0;
        std::size_t double_difference_rejected_no_reference = 0;
        std::size_t motion_factors = 0;
        std::size_t ambiguity_between_factors = 0;
        std::size_t robust_pseudorange_factors = 0;
        std::size_t robust_carrier_phase_factors = 0;
        std::size_t robust_double_difference_pseudorange_factors = 0;
        std::size_t robust_double_difference_carrier_factors = 0;
        std::size_t robust_tdcp_factors = 0;
        std::size_t graph_factors = 0;
        std::size_t graph_values = 0;
        std::size_t imu_intervals = 0;  ///< 2b: CombinedImuFactors added between epochs
        std::size_t smoother_max_window_vars = 0;  ///< 2c: peak in-window variable count
        std::size_t smoother_updates = 0;          ///< 2c: number of smoother.update() calls
        std::size_t smoother_recovery_epochs = 0;  ///< 2e: epochs re-anchored after an indeterminate update
        std::size_t nhc_epochs = 0;   ///< 2d: epochs an NHC factor was applied
        std::size_t zupt_epochs = 0;  ///< 2d: epochs a ZUPT prior was applied
        std::size_t ambiguity_hold_epochs = 0;  ///< 2e: epochs FIXED via held (not fresh) integers
        std::size_t ambiguity_hold_arcs = 0;    ///< 2e: distinct arcs pinned at their integer
        std::size_t float_rejected_seed_position_divergence = 0;
        std::size_t float_rejected_position_jump = 0;
        bool fixed_solution = false;
        bool lambda_ambiguity_fix_solved = false;
        bool lambda_ambiguity_fix_used = false;
        bool partial_lambda_ambiguity_fix_used = false;
        double initial_cost = 0.0;
        double final_cost = 0.0;
        double processing_time_ms = 0.0;
        double epoch_lambda_processing_time_ms = 0.0;
        double epoch_lambda_setup_time_ms = 0.0;
        double epoch_lambda_factorization_time_ms = 0.0;
        double epoch_lambda_covariance_solve_time_ms = 0.0;
        double epoch_lambda_search_time_ms = 0.0;
        double epoch_lambda_fixed_output_time_ms = 0.0;
        double epoch_lambda_debug_record_time_ms = 0.0;
        double postprocessing_time_ms = 0.0;
        double total_processing_time_ms = 0.0;
        double last_update_norm_m = 0.0;
        double residual_rms_m = 0.0;
        double tdcp_residual_rms_m = 0.0;
        double single_difference_doppler_residual_rms_mps = 0.0;
        double single_difference_tdcp_residual_rms_m = 0.0;
        double carrier_phase_residual_rms_m = 0.0;
        double double_difference_pseudorange_residual_rms_m = 0.0;
        double double_difference_carrier_residual_rms_m = 0.0;
        double fixed_ambiguity_residual_rms_cycles = 0.0;
        double lambda_ambiguity_ratio = 0.0;
    };

    struct AmbiguityEstimate {
        SatelliteId satellite;
        SignalType signal = SignalType::GPS_L1CA;
        std::size_t segment_index = 0;
        double wavelength_m = 0.0;
        double ambiguity_m = 0.0;
        double ambiguity_cycles = 0.0;
        int fixed_cycles = 0;
        double fixed_ambiguity_m = 0.0;
        double fix_residual_cycles = 0.0;
        bool is_fixed = false;
        bool fixed_by_lambda = false;
    };

    struct LambdaDebugEntry {
        std::size_t epoch_index = 0;
        GNSSTime time;
        bool solved = false;
        bool fixed_epoch = false;
        double ratio = 0.0;
        int candidate_count = 0;
        int row = 0;
        int col = 0;
        int local_index = 0;
        int other_local_index = 0;
        SatelliteId satellite;
        SatelliteId other_satellite;
        double ambiguity_float = 0.0;
        double fixed_ambiguity = 0.0;
        double covariance = 0.0;
        double position_covariance_x = 0.0;
        double position_covariance_y = 0.0;
        double position_covariance_z = 0.0;
    };

    struct CostTraceEntry {
        std::string phase;
        int local_iteration = 0;
        int global_iteration = 0;
        double cost = 0.0;
        double absolute_decrease = 0.0;
        double relative_decrease = 0.0;
        double update_norm = 0.0;
        bool converged = false;
    };

    struct FGOResult {
        Solution solution;
        FGODiagnostics diagnostics;
        std::vector<AmbiguityEstimate> ambiguity_estimates;
        std::vector<std::set<SatelliteId>> ambiguity_candidate_satellites_by_epoch;
        std::vector<std::set<SatelliteId>> ambiguity_reference_satellites_by_epoch;
        std::vector<std::map<SatelliteId, double>> ambiguity_estimate_cycles_by_epoch;
        std::vector<Vector3d> epoch_velocities_ecef_mps;
        std::vector<LambdaDebugEntry> lambda_debug_entries;
        std::vector<CostTraceEntry> cost_trace_entries;
        // Milestone 2b (populated only by the GTSAM IMU-coupled path):
        // per-epoch estimated attitude as [roll, pitch, heading] in degrees
        // (body FLU -> nav ENU; heading is clockwise from North) and estimated
        // velocity in the ENU nav frame [m/s].
        std::vector<Vector3d> epoch_attitude_rpy_deg;
        std::vector<Vector3d> epoch_velocity_nav_mps;
    };

    FGOProcessor() = default;
    explicit FGOProcessor(const FGOConfig& config) : config_(config) {}

    const FGOConfig& getConfig() const { return config_; }
    void setConfig(const FGOConfig& config) { config_ = config; }

    FGOProblem buildPseudorangeProblem(const std::vector<ObservationData>& epochs,
                                       const NavigationData& nav) const;

    FGOProblem buildDoubleDifferenceProblem(
        const std::vector<ObservationData>& rover_epochs,
        const std::vector<ObservationData>& base_epochs,
        const NavigationData& nav,
        const Vector3d& base_position_ecef) const;

    FGOResult optimize(const std::vector<ObservationData>& epochs,
                       const NavigationData& nav) const;

    FGOResult optimize(const std::vector<ObservationData>& rover_epochs,
                       const std::vector<ObservationData>& base_epochs,
                       const NavigationData& nav,
                       const Vector3d& base_position_ecef) const;

    FGOResult optimizeProblem(const FGOProblem& problem) const;

private:
    FGOConfig config_;
};

}  // namespace libgnss
