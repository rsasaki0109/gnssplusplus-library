#pragma once

#include <libgnss++/algorithms/fgo_config.hpp>
#include <libgnss++/algorithms/doppler_velocity_wls.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/solution.hpp>
#include <libgnss++/core/types.hpp>
#include <libgnss++/io/imu.hpp>

#include <cstddef>
#include <cstdint>
#include <array>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace libgnss {

/**
 * @brief Batch pseudorange factor-graph optimizer.
 *
 * This is the native Eigen backend for the GNSS FGO pipeline. It keeps the
 * factor/problem representation explicit so a GTSAM backend can be added later
 * without changing callers that prepare GNSS factors from RINEX/navigation data.
 */
class FGOProcessor {
public:
    // FGOConfig lives in <libgnss++/algorithms/fgo_config.hpp> (fgo::Config);
    // the alias preserves the historical FGOProcessor::FGOConfig spelling.
    using FGOConfig = fgo::Config;

    struct EpochSeed {
        GNSSTime time;
        Vector3d position_ecef = Vector3d::Zero();
        double receiver_clock_bias_m = 0.0;
        // The historical SPP seed stores receiver_clock_bias_m in seconds
        // until an opt-in raw bridge normalizes it.  Keep the marker explicit
        // so a bridge never guesses units from magnitude.
        bool receiver_clock_bias_is_meters = false;
        // True only when position_ecef came from a valid SPP solve at this
        // epoch; false for last-valid/header fallbacks.
        bool fresh_spp_solution = false;
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
        bool has_doppler_residual = false;
        double doppler_residual_mps = 0.0;
        double doppler_measured_range_rate_mps = 0.0;
        double doppler_satellite_range_rate_mps = 0.0;
        double doppler_satellite_clock_drift_mps = 0.0;
        bool doppler_uses_rotated_satellite_state = false;
        // Raw rover-receiver SNR/CN0 [dB-Hz] for this observation (Observation::snr
        // at the point this model_debug was built). Added for the sat-badness
        // EWMA down-weighting port's elevation/SNR penalty terms (see
        // FGOConfig::use_sat_badness_downweight); unused elsewhere. 0.0 when the
        // source Observation carried no SNR.
        double snr_dbhz = 0.0;
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

    /**
     * @brief Receiver-only (undifferenced) Doppler factor.
     *
     * The measured range-rate residual is prepared from the rover
     * observation and broadcast satellite state.  Unlike the
     * SingleDifferenceDopplerFactor this row has no base/reference satellite
     * and therefore remains usable in a no-base phone graph.
     */
    struct UndifferencedDopplerFactor {
        std::size_t epoch_index = 0;
        std::size_t previous_epoch_index =
            std::numeric_limits<std::size_t>::max();
        SatelliteId satellite;
        SignalType signal = SignalType::GPS_L1CA;
        Vector3d los = Vector3d::Zero();
        double residual_mps = 0.0;
        double sigma_mps = 0.2;
        double elevation_rad = 0.0;
        double dt_s = 0.0;
        double measured_range_rate_mps = 0.0;
        double satellite_range_rate_mps = 0.0;
        double satellite_clock_drift_mps = 0.0;
        bool includes_receiver_clock_drift = false;
        bool uses_rotated_satellite_state = false;
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
        std::size_t target_ambiguity_index = 0;
        std::size_t reference_ambiguity_index = 0;
        int arc_length_epochs = 0;
        double dt_s = 0.0;
        bool has_doppler_witness = false;
        double previous_sd_doppler_mps = 0.0;
        double current_sd_doppler_mps = 0.0;
        double previous_sd_doppler_sigma_mps = 0.0;
        double current_sd_doppler_sigma_mps = 0.0;
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
        std::size_t sparse_epochs_retained = 0;
        std::size_t sparse_empty_epochs_retained = 0;
        std::size_t double_difference_matched_base_epochs = 0;
        std::size_t double_difference_interpolated_base_epochs = 0;
        std::size_t double_difference_candidate_pairs = 0;
        std::size_t double_difference_rejected_no_base_epoch = 0;
        std::size_t double_difference_rejected_no_reference = 0;
        std::size_t tdcp_candidate_pairs = 0;
        std::size_t tdcp_rejected_gap = 0;
        std::size_t tdcp_rejected_clock_discontinuity = 0;
        std::size_t tdcp_rejected_missing_previous = 0;
        std::size_t tdcp_rejected_loss_of_lock = 0;
        std::size_t tdcp_rejected_invalid_measurement = 0;
        std::size_t tdcp_rejected_code_phase_jump = 0;
        std::size_t code_minus_carrier_jump_resets = 0;       ///< CMC screening: arc breaks forced
        std::size_t geometry_free_cycle_slip_resets = 0;      ///< confirmed geometry-free band resets
        std::size_t code_minus_carrier_level_exclusions = 0;  ///< CMC screening: (sat,signal) epochs excluded
        std::size_t cmc_ref_avoided_count = 0;  ///< cmc_aware_reference_selection: references changed away from a CMC-excluded candidate
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

    /**
     * @brief A truth-free native PDC state used as an optional initializer.
     *
     * This is intentionally a value object, not a coordinate-file interface.
     * The smartphone bridge constructs it in memory from the same
     * PseudorangeFactor/UndifferencedDopplerFactor rows consumed by FGO.  A
     * backend may reject an individual component by its `has_*` flag. The
     * P/D measurements are not duplicated as priors; the production/default
     * graph remains unchanged when the bridge config flag is false.
     */
    // State-only handoff from the in-process native PDC solve. This is an
    // initializer/diagnostic record, not an additional graph factor: the same
    // raw P/D observations remain owned by the normal FGO graph.
    struct NativePdcStateSeed {
        std::size_t epoch_index = 0;
        Vector3d position_ecef = Vector3d::Zero();
        Vector3d velocity_ecef_mps = Vector3d::Zero();
        std::array<double, 5> clock_bias_m = {0.0, 0.0, 0.0, 0.0, 0.0};
        double clock_rate_mps = 0.0;
        double position_sigma_m = 0.0;
        double velocity_sigma_mps = 0.0;
        double clock_sigma_m = 0.0;
        double clock_rate_sigma_mps = 0.0;
        int pseudorange_rows = 0;
        int doppler_rows = 0;
        int rank = 0;
        double condition_number = std::numeric_limits<double>::infinity();
        double normalized_pseudorange_rms =
            std::numeric_limits<double>::infinity();
        bool has_position = false;
        bool has_velocity = false;
        bool has_clock = false;
        bool has_clock_rate = false;
    };

    struct FGOProblem {
        std::vector<EpochSeed> epochs;
        ImuInput imu;  ///< Milestone 2b IMU inputs (valid only when populated).
        std::vector<bool> clock_jumps;
        // Diagnostic copy of the signed common GPS pseudorange change used by
        // the legacy positive-only clock_jumps detector, plus its support.
        // Neither field is consumed by an optimizer.
        std::vector<double> gps_common_pseudorange_delta_m;
        std::vector<int> gps_common_pseudorange_delta_satellites;
        std::vector<PseudorangeFactor> pseudorange_factors;
        std::vector<TimeDifferencedCarrierFactor> tdcp_factors;
        std::vector<UndifferencedDopplerFactor> undifferenced_doppler_factors;
        std::vector<doppler_velocity_wls::Estimate>
            doppler_velocity_wls_estimates;
        std::vector<SingleDifferenceDopplerFactor> single_difference_doppler_factors;
        std::vector<SingleDifferenceTdcpFactor> single_difference_tdcp_factors;
        std::vector<AmbiguityState> ambiguity_states;
        std::vector<CarrierPhaseFactor> carrier_observations;
        std::vector<CarrierPhaseFactor> double_difference_pseudorange_observations;
        std::vector<CarrierPhaseFactor> double_difference_reference_observations;
        std::vector<CarrierPhaseFactor> carrier_phase_factors;
        std::vector<DoubleDifferencePseudorangeFactor> double_difference_pseudorange_factors;
        std::vector<DoubleDifferenceCarrierFactor> double_difference_carrier_factors;
        // DD carrier rows dropped BEFORE reaching double_difference_carrier_
        // factors above by a build-time exclusion (currently: CMC sustained-
        // multipath level exclusion, code_minus_carrier_level_threshold_m,
        // when NOT running in code_minus_carrier_level_pseudorange_only
        // mode -- that mode keeps the DD-CP factor and only excludes the
        // DD-PR factor, so nothing needs to be retained here for it).  NEVER
        // added to the solved graph; consumed only by the surplus-satellite
        // independent integrity validation (FGOConfig::use_surplus_
        // satellite_validation) as its "observations excluded from the fix"
        // pool. ambiguity_index on these entries is a sentinel
        // (std::numeric_limits<std::size_t>::max()) -- there is no
        // AmbiguityState for an arc that was never solved for; the surplus
        // validator looks up wavelength from `signal` instead.
        std::vector<DoubleDifferenceCarrierFactor> excluded_double_difference_carrier_factors;
        std::vector<AmbiguityBetweenFactor> ambiguity_between_factors;
        // Optional in-memory PDC state bridge.  Empty unless a caller has
        // explicitly enabled and populated the research-only bridge.
        std::vector<NativePdcStateSeed> native_pdc_state_seeds;
        FGOProblemDiagnostics diagnostics;
    };

    struct GeometryFreeSlipShadowEpoch {
        int event_pairs = 0;
        double max_jump_m = 0.0;
        int tainted_ambiguities = 0;
        int doppler_event_signals = 0;
        double doppler_max_innovation_m = 0.0;
        int doppler_isolated_event_pairs = 0;
        std::set<std::pair<SatelliteId, SignalType>> event_satellite_signals;
    };

    enum class TemporalCarrierShadowClassification {
        Clean = 0,
        WitnessedOutlier = 1,
        UnexplainedOutlier = 2,
    };

    /// Diagnostic-only classification of one receiver-clock-free temporal
    /// carrier difference.  None of these fields has estimator authority.
    struct TemporalCarrierShadowFactorDiagnostics {
        SingleDifferenceTdcpFactor factor;
        double residual_m = 0.0;
        double normalized_residual = 0.0;
        bool residual_outlier = false;
        bool doppler_evaluated = false;
        double doppler_innovation_signed_m = 0.0;
        double doppler_innovation_m = 0.0;
        double doppler_innovation_sigma_m = 0.0;
        double normalized_doppler_innovation = 0.0;
        bool doppler_outlier = false;
        bool doppler_calibration_evaluated = false;
        double doppler_bias_m = 0.0;
        double doppler_calibrated_scale_m = 0.0;
        double doppler_centered_innovation_m = 0.0;
        double doppler_calibrated_score = 0.0;
        bool doppler_calibrated_outlier = false;
        bool geometry_free_witness = false;
        bool carrier_hold_witness = false;
        bool carrier_fde_witness = false;
        TemporalCarrierShadowClassification classification =
            TemporalCarrierShadowClassification::Clean;
    };

    enum class PredictedDdprQualityAction {
        Unavailable = 0,
        Keep = 1,
        Downweight = 2,
    };

    /// Causal, diagnostic-only temporal quality check for one DD pseudorange
    /// row. The previous position is the already-solved prior epoch and the
    /// current position is the pre-solve IMU prediction. No field has
    /// estimator authority.
    struct PredictedDdprQualityFactorDiagnostics {
        std::size_t previous_epoch_index = 0;
        std::size_t current_epoch_index = 0;
        SatelliteId satellite;
        SatelliteId reference_satellite;
        SignalType signal = SignalType::GPS_L1CA;
        double dt_s = 0.0;
        int pair_age_epochs = 0;
        double measured_ddpr_change_m = 0.0;
        bool doppler_evaluated = false;
        double doppler_predicted_change_m = 0.0;
        double doppler_innovation_m = 0.0;
        double doppler_innovation_sigma_m = 0.0;
        double normalized_doppler_innovation = 0.0;
        bool imu_geometry_evaluated = false;
        double imu_predicted_change_m = 0.0;
        double previous_predicted_ddpr_residual_m = 0.0;
        double current_predicted_ddpr_residual_m = 0.0;
        double imu_innovation_m = 0.0;
        double imu_innovation_sigma_m = 0.0;
        double normalized_imu_innovation = 0.0;
        double elevation_rad = 0.0;
        double target_snr_dbhz = 0.0;
        double reference_snr_dbhz = 0.0;
        PredictedDdprQualityAction proposed_action =
            PredictedDdprQualityAction::Unavailable;
    };

    /// Causal, diagnostic-only scalar bias prediction for one DD pseudorange
    /// pair. `prior_bias_m` is formed solely from earlier rows. The current
    /// residual is assimilated only after this diagnostic has been formed.
    struct PredictedDdprBiasStateDiagnostics {
        std::size_t previous_epoch_index = 0;
        std::size_t current_epoch_index = 0;
        SatelliteId satellite;
        SatelliteId reference_satellite;
        SignalType signal = SignalType::GPS_L1CA;
        double dt_s = 0.0;
        int pair_age_epochs = 0;
        int prior_updates = 0;
        bool continuity_reset = false;
        bool prediction_usable = false;
        bool update_applied = false;
        bool update_clipped = false;
        double raw_residual_m = 0.0;
        double prior_bias_m = 0.0;
        double prior_sigma_m = 0.0;
        double corrected_residual_m = 0.0;
        double measurement_sigma_m = 0.0;
        double innovation_sigma_m = 0.0;
        double normalized_innovation = 0.0;
        double applied_innovation_m = 0.0;
        double posterior_bias_m = 0.0;
        double posterior_sigma_m = 0.0;
    };

    struct SatelliteQuarantineWitnessDiagnostics {
        std::size_t epoch_index = 0;
        SatelliteId satellite;
        double postfit_ddpr_residual_m = 0.0;
        double epoch_median_postfit_ddpr_residual_m = 0.0;
        bool postfit_gross = false;
        bool doppler_evaluated = false;
        bool doppler_outlier = false;
        bool imu_evaluated = false;
        bool imu_outlier = false;
        int temporal_support_pairs = 0;
        bool quarantine_candidate = false;
    };

    struct FGODiagnostics {
        int iterations = 0;
        bool converged = false;
        std::size_t epochs = 0;
        std::size_t sparse_epochs_retained = 0;
        std::size_t sparse_empty_epochs_retained = 0;
        std::size_t pseudorange_factors = 0;
        /// TDCP measurements present in the backend-independent problem.
        std::size_t tdcp_factors = 0;
        /// TDCP residual rows/factors actually inserted by the selected backend.
        std::size_t tdcp_factors_inserted = 0;
        std::size_t undifferenced_doppler_factors = 0;
        // Truth-free per-epoch Doppler WLS initialization diagnostics.
        std::size_t doppler_velocity_wls_valid_epochs = 0;
        std::size_t doppler_velocity_wls_propagated_epochs = 0;
        std::size_t doppler_velocity_wls_rejected_epochs = 0;
        double doppler_velocity_wls_max_condition_number = 0.0;
        double doppler_velocity_wls_max_normalized_rms = 0.0;
        double doppler_velocity_wls_max_velocity_norm_mps = 0.0;
        double doppler_velocity_wls_max_clock_rate_abs_mps = 0.0;
        std::size_t single_difference_doppler_factors = 0;
        /// Satellite-single-difference TDCP measurements present in the problem.
        std::size_t single_difference_tdcp_factors = 0;
        /// Satellite-single-difference TDCP rows/factors actually inserted by the backend.
        std::size_t single_difference_tdcp_factors_inserted = 0;
        std::size_t carrier_phase_factors = 0;
        std::size_t double_difference_pseudorange_factors = 0;
        std::size_t double_difference_carrier_factors = 0;
        std::size_t ambiguity_states = 0;
        std::size_t ambiguity_fix_candidates = 0;
        std::size_t lambda_ambiguity_candidates = 0;
        std::size_t lambda_ambiguity_used_candidates = 0;
        std::size_t lambda_ambiguity_attempts = 0;
        /// Ambiguity keys removed before LAMBDA because they left the active smoother.
        std::size_t lambda_stale_candidates_filtered = 0;
        /// Exceptions while requesting the active pose/ambiguity joint marginal.
        std::size_t lambda_joint_marginal_failures = 0;
        std::size_t integer_constrained_reoptimization_attempts = 0;
        std::size_t integer_constrained_reoptimization_accepts = 0;
        std::size_t integer_constrained_reoptimization_rejects = 0;
        std::size_t imu_aided_ratio_accepts = 0;
        std::size_t imu_aided_ratio_rejects = 0;
        std::size_t fixed_history_dr_accepts = 0;
        std::size_t fixed_history_dr_rejects = 0;
        std::size_t fixed_history_dr_surplus_overrides = 0;  ///< surplus_validation_overrides_history_dr flipped a DR-rejected candidate to FIXED
        std::size_t fixed_history_dr_surplus_override_capped = 0;  ///< surplus_validation_overrides_history_dr_max_consecutive blocked an otherwise-qualifying override
        std::size_t ddpr_anchor_validation_accepts = 0;
        std::size_t ddpr_anchor_validation_rejects = 0;
        std::size_t fixed_postfit_validation_accepts = 0;
        std::size_t fixed_postfit_validation_rejects = 0;
        std::size_t external_doppler_dr_accepts = 0;
        std::size_t external_doppler_dr_rejects = 0;
        std::size_t external_doppler_dr_unavailable = 0;
        // --- Surplus-satellite independent integrity validation (use_surplus_satellite_validation) ---
        std::size_t surplus_validation_attempts = 0;   ///< LAMBDA attempts where the test rendered a verdict
        std::size_t surplus_validation_passes = 0;
        std::size_t surplus_validation_fails = 0;
        std::size_t surplus_validation_insufficient_surplus = 0;  ///< too few surplus sats at every fallback level
        std::size_t surplus_validation_rescued_epochs = 0;  ///< epochs FIXED only because this test passed a relaxed-ratio candidate
        std::size_t surplus_validation_separation_rejects = 0;  ///< surplus-pass rescues rejected by the existing fixed-vs-float/IMU separation aperture
        std::size_t surplus_validation_quality_rejects = 0;  ///< surplus-pass rescues rejected by the independent geometry/code-quality floor
        std::size_t surplus_validation_vetoed_epochs = 0;   ///< established-ratio fixes demoted by surplus_validation_veto_high_ratio_fails
        std::size_t surplus_validation_fallback_level_histogram[6] = {0, 0, 0, 0, 0, 0};  ///< index = deciding fallback level (0=GQEBR .. 5=GQ)
        // --- Below-floor low-count AR rescue (use_low_count_ambiguity_resolution) ---
        std::size_t low_count_ambiguity_attempts = 0;  ///< LAMBDA attempts made only because this knob lowered the floor
        std::size_t low_count_ambiguity_fix_accepted = 0;  ///< of which accepted (surplus pass [+ ratio floor])
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
        std::size_t ddpr_gnc_evaluated_epochs = 0;
        std::size_t ddpr_gnc_factors = 0;
        std::size_t ddpr_gnc_downweighted_factors = 0;
        std::size_t ddpr_gnc_counterfactual_attempts = 0;
        std::size_t ddpr_gnc_counterfactual_successes = 0;
        std::size_t candidate_integrity_witness_evaluated = 0;
        std::size_t candidate_integrity_witness_passes = 0;
        std::size_t satellite_quarantine_witness_satellites = 0;
        std::size_t satellite_quarantine_candidates = 0;
        std::size_t selective_arc_restart_detection_epochs = 0;
        std::size_t selective_arc_restart_candidate_satellites = 0;
        std::size_t selective_arc_restart_candidate_pairs = 0;
        std::size_t selective_arc_restart_applied_arcs = 0;
        std::size_t selective_arc_restart_skipped_fixed = 0;
        std::size_t selective_arc_restart_skipped_thrash = 0;
        std::size_t selective_arc_restart_skipped_cap = 0;
        std::size_t selective_arc_restart_skipped_no_arc = 0;
        std::size_t graph_factors = 0;
        std::size_t graph_values = 0;
        std::size_t native_pdc_position_seeds = 0;
        std::size_t native_pdc_velocity_seeds = 0;
        std::size_t native_pdc_clock_seeds = 0;
        std::size_t native_pdc_clock_rate_seeds = 0;
        std::size_t imu_intervals = 0;  ///< 2b: CombinedImuFactors added between epochs
        std::size_t smoother_max_window_vars = 0;  ///< 2c: peak in-window variable count
        std::size_t smoother_updates = 0;          ///< 2c: number of smoother.update() calls
        std::size_t smoother_recovery_epochs = 0;  ///< 2e: epochs re-anchored after an indeterminate update
        std::size_t nhc_epochs = 0;   ///< 2d: epochs an NHC factor was applied
        std::size_t zupt_epochs = 0;  ///< 2d: epochs a ZUPT prior was applied
        std::size_t ambiguity_hold_epochs = 0;  ///< 2e: epochs FIXED via held (not fresh) integers
        std::size_t ambiguity_hold_arcs = 0;    ///< 2e: distinct arcs pinned at their integer
        std::size_t quality_gated_epochs = 0;   ///< epochs where the quality gates suppressed fixing
        std::size_t code_minus_carrier_jump_resets = 0;       ///< CMC screening: arc breaks forced
        std::size_t geometry_free_cycle_slip_resets = 0;      ///< confirmed geometry-free band resets
        std::size_t code_minus_carrier_level_exclusions = 0;  ///< CMC screening: (sat,signal) epochs excluded
        std::size_t cmc_ref_avoided_count = 0;  ///< cmc_aware_reference_selection: references changed away from a CMC-excluded candidate
        // --- CP-hold / sanity FSM diagnostics (use_cp_hold_recovery) ---
        std::size_t cp_hold_triggers = 0;         ///< times CP-hold was (re)engaged/extended
        std::size_t cp_hold_epochs_held = 0;      ///< cumulative epochs with carrier suppressed
        std::size_t cp_hold_anchor_releases = 0;
        std::size_t selective_cp_hold_downweighted_factors = 0;
        std::size_t sanity_mass_resets = 0;       ///< persist-path (3 consecutive bad) resets
        std::size_t sanity_fast_resets = 0;       ///< catastrophic fast-path resets
        std::size_t sanity_pose_replacements = 0; ///< epochs where the reported pose was IMU-predicted
        std::size_t sanity_multipath_skips = 0;   ///< bad epochs skipped as single-satellite multipath
        std::size_t sanity_gdop_skips = 0;        ///< persist-eligible resets skipped for weak geometry
        std::size_t ambiguity_generation_bumps = 0;  ///< total per-arc generation bumps (fresh symbols)
        std::size_t ambiguity_generation_bumps_hold = 0;
        std::size_t ambiguity_generation_bumps_fde = 0;
        std::size_t ambiguity_generation_bumps_reset = 0;
        std::size_t ambiguity_generation_bumps_warm_reset = 0;
        std::size_t ambiguity_continuous_unfix_resets = 0;
        std::size_t ambiguity_continuous_unfix_anchor_allows = 0;
        std::size_t ambiguity_continuous_unfix_anchor_skips = 0;
        std::size_t ambiguity_generation_bumps_stale_pin = 0;
        // --- Stale-pin invalidation diagnostics (use_stale_pin_invalidation) ---
        std::size_t stale_pin_invalidations = 0;  ///< pinned arcs released per-arc at a trigger epoch
        // --- Fix plausibility demotion diagnostics ---
        std::size_t fix_plausibility_demotions = 0;  ///< FIXED epochs demoted to FLOAT by general or intrinsic GF integrity guards
        std::size_t geometry_free_fix_guard_demotions = 0;  ///< low-redundancy GF-reset fixes rejected by gross SPP disagreement
        std::size_t fix_plausibility_anchor_demotions = 0;  ///< of which via the DDPR-LS anchor gap
        std::size_t fix_plausibility_anchor_gross_gated = 0;  ///< anchor-gap evaluations skipped by the gross-offender gate (fix_demote_anchor_gross)
        std::size_t fix_plausibility_hold_skips = 0;  ///< fix-and-hold pinnings skipped on implausible epochs
        std::size_t fix_plausibility_surplus_reprieves = 0;  ///< demotions skipped because fix_demote_surplus_crosscheck's verdict passed
        std::size_t fix_plausibility_spp_model_reprieves = 0;  ///< residual-only demotions skipped by fresh-SPP/model agreement
        // --- Exception recovery diagnostics (use_solve_exception_recovery) ---
        std::size_t solve_exception_recoveries = 0;   ///< loose-prior retries that succeeded
        std::size_t solve_exception_warm_resets = 0;  ///< full smoother re-creations
        // --- DDPR-LS anchor diagnostics (use_ddpr_anchor) ---
        std::size_t ddpr_anchor_solves = 0;         ///< mini DDPR-LS solve attempts (any of the 3 call sites)
        std::size_t ddpr_anchor_successes = 0;      ///< of which trusted (n>=min_factors, res_rms<=max)
        std::size_t ddpr_anchor_gated_resets_skipped = 0;  ///< diagnostic-only: gate would have rejected the reset (persist path; the reset still fires -- see use_ddpr_anchor comment)
        std::size_t ddpr_anchor_gated_resets_allowed = 0;  ///< diagnostic-only: gate would have accepted the reset
        std::size_t ddpr_anchored_warm_resets = 0;  ///< exception recoveries that used the DDPR anchor (vs the IMU-seeded fallback)
        std::size_t ddpr_anchor_bootstrap_prior_epochs = 0;  ///< epochs an anchor bootstrap translation prior was actually added
        // --- FDE diagnostics (use_fde) ---
        std::size_t fde_pseudorange_rejections = 0;  ///< total DD PR factors removed
        std::size_t fde_carrier_rejections = 0;      ///< total DD CP factors removed
        std::size_t fde_carrier_quarantines = 0;     ///< gross DD CP ambiguities excluded from AR only
        std::size_t fde_safeguard_skips = 0;         ///< epochs where the reject-fraction safeguard aborted FDE
        std::size_t fde_epochs = 0;                  ///< epochs where >=1 factor was actually removed
        // --- Sat-badness EWMA down-weighting diagnostics (use_sat_badness_downweight) ---
        std::size_t sat_badness_downweighted_factors = 0;  ///< DD PR/CP factors whose sigma was inflated (bad_pair>0 and its scale>0)
        double sat_badness_max_score_seen = 0.0;            ///< max bad_pair score observed across the run
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
        double undifferenced_doppler_residual_rms_mps = 0.0;
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

    /// Terminal ambiguity-resolution state for one epoch.  This is kept
    /// separate from the reported FIX/FLOAT status so a FLOAT epoch says
    /// exactly which integrity gate stopped it.
    enum class AmbiguityResolutionOutcome : int {
        NotAttempted = 0,
        Disabled = 1,
        SmootherFailure = 2,
        QualityGateRejected = 3,
        NoCandidates = 4,
        InsufficientCandidates = 5,
        MarginalFailure = 6,
        LambdaSearchFailed = 7,
        RatioRejected = 8,
        ImuApertureRejected = 9,
        FixedHistoryRejected = 10,
        PostfitRejected = 11,
        Fixed = 12,
        SurplusValidationRejected = 13,
        LowCountRejected = 14,  ///< below-floor attempt (use_low_count_ambiguity_resolution) failed its surplus/ratio gate
        IntegerConstrainedReoptimizationRejected = 15,
    };

    /// Terminal reason why one satellite/signal DD carrier row did or did not
    /// reach the per-epoch LAMBDA candidate set.  This is diagnostic-only:
    /// the backend records decisions already made by the existing pipeline.
    enum class AmbiguityCandidateDisposition : int {
        LambdaEligible = 0,
        BuildTimeExcluded = 1,
        CarrierHoldSuppressed = 2,
        CarrierHoldQuarantined = 3,
        OneBandPerSatelliteExcluded = 4,
        ConstellationExcluded = 5,
        PreviousResidualGateExcluded = 6,
        FdeExcluded = 7,
        StaleSmootherKeyExcluded = 8,
        EpochQualityGateExcluded = 9,
        AmbiguityResolutionDisabled = 10,
    };

    /// Satellite/signal-level trace for one DD carrier row in one epoch.
    struct AmbiguityCandidateTrace {
        SatelliteId satellite;
        SatelliteId reference_satellite;
        SignalType signal = SignalType::SIGNAL_TYPE_COUNT;
        std::size_t ambiguity_index = std::numeric_limits<std::size_t>::max();
        AmbiguityCandidateDisposition disposition =
            AmbiguityCandidateDisposition::LambdaEligible;
    };

    /// One counterfactual leave-one-target-satellite-out LAMBDA trial.
    /// Populated only by monitor_ratio_impact_partial_ar and never consumed
    /// by the estimator.
    struct RatioImpactTrialTrace {
        SatelliteId excluded_satellite;
        int excluded_ambiguities = 0;
        double excluded_max_variance_cycles2 = 0.0;
        double excluded_max_fractional_cycles = 0.0;
        double excluded_ddpr_residual_m = 0.0;
        bool candidate_available = false;
        double ratio = 0.0;
        int fixed_ambiguities = 0;
        Vector3d candidate_position_ecef = Vector3d::Zero();
        double float_separation_m = 0.0;
        double imu_separation_m = 0.0;
    };

    /// Two constellation-disjoint reduced LAMBDA candidates. Populated only
    /// by monitor_disjoint_constellation_ar and never consumed by the
    /// estimator.
    struct DisjointConstellationArShadow {
        bool evaluated = false;
        std::uint64_t partition_a_system_mask = 0;
        std::uint64_t partition_b_system_mask = 0;
        int partition_a_ambiguities = 0;
        int partition_b_ambiguities = 0;
        double partition_a_ratio = 0.0;
        double partition_b_ratio = 0.0;
        double partition_a_bootstrapped_success_rate = 0.0;
        double partition_b_bootstrapped_success_rate = 0.0;
        bool partition_a_ratio_passed = false;
        bool partition_b_ratio_passed = false;
        bool partition_a_candidate_available = false;
        bool partition_b_candidate_available = false;
        Vector3d partition_a_position_ecef = Vector3d::Zero();
        Vector3d partition_b_position_ecef = Vector3d::Zero();
        double partition_separation_m = 0.0;
        double partition_a_primary_separation_m = 0.0;
        double partition_b_primary_separation_m = 0.0;
    };

    /// Counterfactual two-stage multi-frequency AR result. Populated only by
    /// monitor_conditional_multiband_ar and never consumed by the estimator.
    struct ConditionalMultibandArShadow {
        bool evaluated = false;
        int primary_ambiguities = 0;
        int secondary_ambiguities = 0;
        double primary_ratio = 0.0;
        double secondary_ratio = 0.0;
        double primary_bootstrapped_success_rate = 0.0;
        double secondary_bootstrapped_success_rate = 0.0;
        bool primary_ratio_passed = false;
        bool secondary_ratio_passed = false;
        bool candidate_available = false;
        Vector3d candidate_position_ecef = Vector3d::Zero();
        double float_separation_m = 0.0;
        double imu_separation_m = 0.0;
    };

    /// Counterfactual LAMBDA result using only ambiguity arcs whose integer
    /// candidate has persisted across multiple consecutive epochs.
    struct MultiEpochArShadow {
        bool evaluated = false;
        int persistent_ambiguities = 0;
        int minimum_support_epochs = 0;
        double ratio = 0.0;
        double bootstrapped_success_rate = 0.0;
        bool history_integers_agree = false;
        bool ratio_passed = false;
        bool candidate_available = false;
        Vector3d candidate_position_ecef = Vector3d::Zero();
        double float_separation_m = 0.0;
        double imu_separation_m = 0.0;
        // Held-out carrier witness: rows whose ambiguity is not part of the
        // persistent LAMBDA subset are checked at the candidate position.
        // A missing independent pool leaves evaluated=false (fail closed).
        bool surplus_validation_evaluated = false;
        bool surplus_validation_pass = false;
        int surplus_validation_fallback_level = -1;
        int surplus_validation_surplus_used = 0;
        // GICI-style constrained graph-cost witness. The active fixed-lag
        // graph is independently batch-refined with this candidate imposed
        // as tight ambiguity priors, then scored using the original graph
        // without those priors. Diagnostic-only; never updates the smoother.
        bool graph_cost_evaluated = false;
        bool graph_cost_pass = false;
        int graph_cost_factor_count = 0;
        double graph_cost_before = 0.0;
        double graph_cost_after = 0.0;
    };

    /// One diagnostic-only DDPR GNC weight at the current epoch's optimized
    /// pose. These rows are never consumed by the estimator.
    struct DdprGncFactorTrace {
        SatelliteId satellite;
        SatelliteId reference_satellite;
        SignalType signal = SignalType::GPS_L1CA;
        double residual_m = 0.0;
        double sigma_m = 0.0;
        double normalized_residual = 0.0;
        double weight = 1.0;
    };

    /// One selective arc-restart candidate/action row. Populated only when
    /// use_selective_arc_restart (active) or
    /// monitor_selective_arc_restart_candidates (monitor-only) is enabled.
    struct SelectiveArcRestartTrace {
        std::size_t detection_epoch = 0;
        SatelliteId satellite;
        bool is_reference = false;
        std::size_t ambiguity_index = 0;
        bool detected = false;
        bool applied = false;
        double postfit_residual_m = 0.0;
        double epoch_median_postfit_residual_m = 0.0;
        double normalized_doppler_innovation = 0.0;
        double normalized_imu_innovation = 0.0;
    };

    /// Per-epoch fixed-lag integrity state.  These values expose why an epoch
    /// did or did not fix, rather than only reporting the final FIX/FLOAT label.
    struct FGOEpochDiagnostics {
        GNSSTime time;
        AmbiguityResolutionOutcome ar_outcome =
            AmbiguityResolutionOutcome::NotAttempted;
        double ddpr_rms_m = 0.0;
        bool ddpr_gnc_evaluated = false;
        int ddpr_gnc_factor_count = 0;
        int ddpr_gnc_stages = 0;
        double ddpr_gnc_initial_mu = 0.0;
        double ddpr_gnc_final_mu = 0.0;
        double ddpr_gnc_min_weight = 1.0;
        double ddpr_gnc_mean_weight = 1.0;
        double ddpr_gnc_effective_factor_count = 0.0;
        int ddpr_gnc_downweighted_factors = 0;
        double ddpr_gnc_weighted_rms_m = 0.0;
        std::vector<DdprGncFactorTrace> ddpr_gnc_factor_trace;
        bool ddpr_gnc_counterfactual_evaluated = false;
        bool ddpr_gnc_counterfactual_succeeded = false;
        int ddpr_gnc_counterfactual_factor_count = 0;
        int ddpr_gnc_counterfactual_stages = 0;
        double ddpr_gnc_counterfactual_cost_before = 0.0;
        double ddpr_gnc_counterfactual_cost_after = 0.0;
        double ddpr_gnc_counterfactual_ddpr_rms_before_m = 0.0;
        double ddpr_gnc_counterfactual_ddpr_rms_after_m = 0.0;
        Vector3d ddpr_gnc_counterfactual_position_ecef = Vector3d::Zero();
        double ddpr_gnc_counterfactual_float_separation_m = 0.0;
        bool ddpr_gnc_counterfactual_lambda_evaluated = false;
        int ddpr_gnc_counterfactual_lambda_ambiguities = 0;
        double ddpr_gnc_counterfactual_lambda_ratio = 0.0;
        bool ddpr_gnc_counterfactual_lambda_ratio_pass = false;
        bool candidate_integrity_witness_evaluated = false;
        bool candidate_integrity_anchor_available = false;
        int candidate_integrity_anchor_factors = 0;
        double candidate_integrity_anchor_rms_m = 0.0;
        double candidate_integrity_anchor_separation_m = 0.0;
        bool candidate_integrity_anchor_pass = false;
        bool candidate_integrity_imu_pass = false;
        bool candidate_integrity_doppler_available = false;
        bool candidate_integrity_doppler_pass = false;
        bool candidate_integrity_carrier_pass = false;
        bool candidate_integrity_composite_pass = false;
        double sd_doppler_rms_mps = 0.0;
        int clock_resilient_tdcp_factors = 0;
        double clock_resilient_tdcp_rms_m = 0.0;
        double clock_resilient_tdcp_max_abs_m = 0.0;
        int clock_resilient_tdcp_clean = 0;
        int clock_resilient_tdcp_witnessed_outliers = 0;
        int clock_resilient_tdcp_unexplained_outliers = 0;
        double gdop = 0.0;
        int num_satellites = 0;
        int sd_doppler_factors = 0;
        int ambiguity_candidates = 0;
        int lambda_attempts = 0;
        int lambda_selected_stage = -1;  ///< PPC cascade: 0=GQEBR ... 5=GQ
        double ambiguity_variance_median_cycles2 = 0.0;
        double ambiguity_variance_max_cycles2 = 0.0;
        double imu_pose_correction_m = 0.0;
        // Read-only copy of the builder's independent current-epoch SPP
        // seed. This exposes an absolute-code witness for AR analysis while
        // keeping all estimator and FIX/FLOAT decisions unchanged.
        bool fresh_spp_solution = false;
        Vector3d spp_seed_position_ecef = Vector3d::Zero();
        // Last position candidate produced by a successful LAMBDA search in
        // this epoch, even when a later integrity/ratio decision leaves the
        // epoch FLOAT. Diagnostic-only; never feeds the estimator.
        bool lambda_candidate_available = false;
        Vector3d lambda_candidate_position_ecef = Vector3d::Zero();
        int lambda_candidate_fixed_ambiguities = 0;
        double lambda_candidate_ratio = 0.0;
        // Covariance-only quality diagnostics from the same Top-K LAMBDA
        // search that produced lambda_candidate_position_ecef. These fields
        // are monitor-only and never participate in FIX/FLOAT decisions.
        double lambda_candidate_bsr = 0.0;
        double lambda_candidate_bsr_qscale2 = 0.0;
        double lambda_candidate_bsr_qscale4 = 0.0;
        double lambda_candidate_bsr_qscale8 = 0.0;
        double lambda_candidate_bsr_qscale16 = 0.0;
        bool lambda_candidate_ffrt_table_supported = false;
        bool lambda_candidate_ffrt_accepts_any = false;
        double lambda_candidate_ffrt_min_ratio = 0.0;
        bool lambda_candidate_ffrt_pass = false;
        // Temporal-consensus shadow for the last LAMBDA candidate in this
        // epoch. Ambiguity indices are stable only within an unchanged arc,
        // so overlap automatically excludes restarted/referenced arcs. The
        // shadow never changes acceptance, hold, or graph state.
        int lambda_candidate_integer_overlap = 0;
        int lambda_candidate_integer_agreements = 0;
        double lambda_candidate_integer_agreement_fraction = 0.0;
        int lambda_candidate_integer_consensus_streak = 0;
        // Diagnostic-only geometry-free cycle-slip shadow. It analyzes the
        // rover/base single-difference carrier phases already present in the
        // DD factors, but never changes ambiguity arcs or graph factors.
        int gf_slip_shadow_event_pairs = 0;
        double gf_slip_shadow_max_jump_m = 0.0;
        int gf_slip_shadow_tainted_ambiguities = 0;
        int doppler_slip_shadow_event_signals = 0;
        double doppler_slip_shadow_max_innovation_m = 0.0;
        int gf_doppler_shadow_isolated_pairs = 0;
        ConditionalMultibandArShadow conditional_multiband_ar_shadow;
        MultiEpochArShadow multiepoch_ar_shadow;
        bool ratio_impact_evaluated = false;
        int ratio_impact_trials = 0;
        double ratio_impact_best_ratio = 0.0;
        int ratio_impact_best_fixed_ambiguities = 0;
        Vector3d ratio_impact_best_position_ecef = Vector3d::Zero();
        double ratio_impact_best_float_separation_m = 0.0;
        double ratio_impact_best_imu_separation_m = 0.0;
        std::vector<RatioImpactTrialTrace> ratio_impact_trial_trace;
        DisjointConstellationArShadow disjoint_constellation_ar_shadow;
        bool ddpr_anchor_evaluated = false;
        bool ddpr_anchor_bootstrap_prior_applied = false;
        int ddpr_anchor_active_factors = 0;
        double ddpr_anchor_residual_rms_m = 0.0;
        Vector3d ddpr_anchor_position_ecef = Vector3d::Zero();
        double fixed_float_separation_m = 0.0;
        double fixed_imu_prediction_separation_m = 0.0;
        // --- "c2" DR-gate bypass counterfactual (see FGOConfig::
        // surplus_validation_overrides_history_dr). Monitor fields below are
        // populated whenever a surplus-passed candidate's RAW fixed-history-
        // DR verdict (before any reprieve/override) would reject it -- cheap
        // and unconditional, independent of whether the override knob is
        // enabled -- so callers can score the would-be fix against
        // reference.csv even when the epoch is left FLOAT. ---
        bool dr_bypass_candidate_evaluated = false;  ///< true = this epoch had a surplus-passed candidate blocked by the raw DR verdict
        Vector3d dr_bypass_candidate_position_ecef = Vector3d::Zero();  ///< that candidate's fixed antenna ECEF position
        bool dr_bypass_applied = false;  ///< surplus_validation_overrides_history_dr actually accepted this candidate
        double fixed_postfit_ddcp_rms_m = 0.0;
        double fixed_postfit_ddcp_max_normalized = 0.0;
        double fixed_postfit_ddcp_chi2_per_dof = 0.0;
        int fixed_postfit_ddcp_factors = 0;
        double effective_ratio_threshold = 0.0;
        bool integer_constrained_reoptimization_evaluated = false;
        bool integer_constrained_reoptimization_pass = false;
        double integer_constrained_base_cost_before = 0.0;
        double integer_constrained_base_cost_after = 0.0;
        double external_dr_separation_m = 0.0;
        double external_dr_mahalanobis2 = 0.0;
        int external_dr_age_epochs = -1;
        bool external_doppler_velocity_valid = false;
        Vector3d external_doppler_velocity_ecef_mps = Vector3d::Zero();
        bool external_dr_evaluated = false;
        bool external_dr_accepted = false;
        bool external_dr_rejected = false;
        // Monitor-only NHC/ZUPT gate inputs and decisions. These values are
        // computed before the current epoch is optimized and never feed AR.
        int motion_constraint_imu_samples = 0;
        double motion_constraint_accel_std_mps2 = 0.0;
        double motion_constraint_gyro_std_radps = 0.0;
        double motion_constraint_gyro_median_radps = 0.0;
        double motion_constraint_yaw_rate_radps = 0.0;
        double motion_constraint_seed_speed_mps = 0.0;
        bool zupt_candidate = false;
        bool zupt_applied = false;
        bool nhc_candidate = false;
        bool nhc_applied = false;
        bool carrier_hold_active = false;
        bool imu_aperture_accepted = false;
        bool imu_aperture_rejected = false;
        // --- Surplus-satellite independent integrity validation ---
        bool surplus_validation_evaluated = false;  ///< false = insufficient surplus sats at every fallback level
        bool surplus_validation_pass = false;
        bool surplus_validation_used_for_rescue = false;  ///< this test flipped a relaxed-ratio candidate to FIXED
        bool surplus_validation_used_for_veto = false;    ///< this test demoted an established-ratio fix
        int surplus_validation_fallback_level = -1;  ///< 0=GQEBR .. 5=GQ, -1 = not evaluated
        int surplus_validation_surplus_used = 0;     ///< surplus satellites in the deciding pool
        // --- Below-floor low-count AR rescue (use_low_count_ambiguity_resolution) ---
        bool low_count_ar_attempted = false;  ///< this epoch's LAMBDA attempt only happened because the floor was lowered
        bool low_count_ar_used = false;       ///< this epoch's FIXED label came from the low-count rescue path
        int carrier_factors_available = 0;
        int carrier_factors_added = 0;
        int carrier_factors_suppressed_hold = 0;
        // Candidate attrition funnel.  ambiguity_candidates retains its
        // historical meaning (after one-band/constellation filtering, before
        // runtime residual/FDE/stale-key filtering).
        int ambiguity_candidates_after_hold = 0;
        int ambiguity_candidates_final = 0;
        int ambiguity_candidates_excluded_build_time = 0;
        int ambiguity_candidates_excluded_hold = 0;
        int ambiguity_candidates_excluded_one_band = 0;
        int ambiguity_candidates_excluded_constellation = 0;
        int ambiguity_candidates_excluded_previous_residual = 0;
        int ambiguity_candidates_excluded_fde = 0;
        int ambiguity_candidates_excluded_stale = 0;
        std::vector<AmbiguityCandidateTrace> ambiguity_candidate_trace;
        int ambiguity_generation_bumps_hold = 0;
        int ambiguity_generation_bumps_fde = 0;
        int ambiguity_generation_bumps_reset = 0;
        int ambiguity_generation_bumps_warm_reset = 0;
        int ambiguity_generation_bumps_stale_pin = 0;
        // --- Active selective arc restart (use_selective_arc_restart) ---
        bool selective_arc_restart_armed = false;
        int selective_arc_restart_candidate_satellites = 0;
        int selective_arc_restart_candidate_pairs = 0;
        int selective_arc_restart_applied_arcs = 0;
        int selective_arc_restart_skipped_fixed = 0;
        int selective_arc_restart_skipped_thrash = 0;
        int selective_arc_restart_skipped_cap = 0;
        bool selective_arc_restart_monitor_only = false;
        std::vector<SelectiveArcRestartTrace> selective_arc_restart_trace;
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
        std::vector<FGOEpochDiagnostics> epoch_diagnostics;
        std::vector<TemporalCarrierShadowFactorDiagnostics>
            temporal_carrier_shadow_factors;
        std::vector<PredictedDdprQualityFactorDiagnostics>
            predicted_ddpr_quality_factors;
        std::vector<PredictedDdprBiasStateDiagnostics>
            predicted_ddpr_bias_state_factors;
        std::vector<SatelliteQuarantineWitnessDiagnostics>
            satellite_quarantine_witnesses;
        std::vector<SelectiveArcRestartTrace> selective_arc_restart_trace;
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

    static std::vector<GeometryFreeSlipShadowEpoch>
    analyzeGeometryFreeSlipShadow(const FGOProblem& problem,
                                  double threshold_m = 0.05,
                                  double max_gap_s = 1.5);

    static std::vector<SingleDifferenceTdcpFactor>
    buildClockResilientTemporalCarrierShadow(const FGOProblem& problem,
                                             double sigma_m = 0.003,
                                             double max_gap_s = 1.5);

    /// Evaluate and classify a pre-built temporal-carrier shadow against a
    /// supplied per-epoch ECEF trajectory. This routine is solver-independent
    /// so a frozen solution CSV can replay diagnostics without rerunning GTSAM.
    static std::vector<TemporalCarrierShadowFactorDiagnostics>
    classifyClockResilientTemporalCarrierShadow(
        const FGOProblem& problem,
        const std::vector<SingleDifferenceTdcpFactor>& factors,
        const std::vector<Vector3d>& epoch_positions_ecef,
        std::vector<FGOEpochDiagnostics>& epoch_diagnostics,
        const std::vector<GeometryFreeSlipShadowEpoch>* geometry_free_shadow =
            nullptr,
        const std::vector<std::set<std::size_t>>*
            fde_rejected_ambiguities_by_epoch = nullptr);

    /// Compare temporal DD pseudorange changes with receiver-clock-free DD
    /// Doppler and a one-step predicted antenna trajectory. `previous_*`
    /// supplies the causal solved pose at k-1; `predicted_*` supplies the
    /// pre-solve pose prediction at k. The result is monitor-only.
    static std::vector<PredictedDdprQualityFactorDiagnostics>
    analyzePredictedDdprQualityShadow(
        const FGOProblem& problem,
        const std::vector<Vector3d>& previous_solution_positions_ecef,
        const std::vector<Vector3d>& predicted_positions_ecef,
        double doppler_sigma_mps = 0.2,
        double normalized_outlier_threshold = 5.0,
        double max_gap_s = 1.5);

    /// Predict persistent pair-specific DD pseudorange bias from prior causal
    /// predicted-DDPR residual rows. The current row never predicts itself.
    static std::vector<PredictedDdprBiasStateDiagnostics>
    analyzePredictedDdprBiasStateShadow(
        const std::vector<PredictedDdprQualityFactorDiagnostics>& quality_rows,
        double process_noise_m_sqrt_s = 0.25,
        double initial_sigma_m = 5.0,
        double min_measurement_sigma_m = 0.5,
        double robust_update_sigma = 3.0,
        int min_prior_updates = 2);

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
