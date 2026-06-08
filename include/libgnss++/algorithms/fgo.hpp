#pragma once

#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/solution.hpp>
#include <libgnss++/core/types.hpp>

#include <cstddef>
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
    struct FGOConfig {
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

    struct FGOProblem {
        std::vector<EpochSeed> epochs;
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
