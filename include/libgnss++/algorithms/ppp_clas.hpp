#pragma once

#include <libgnss++/algorithms/ppp_osr_types.hpp>
#include <libgnss++/algorithms/ppp_atmosphere.hpp>
#include <libgnss++/algorithms/ppp_shared.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/solution.hpp>

#include <functional>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace libgnss::ppp_clas {

struct FixValidationStats {
    bool accepted = false;
    int phase_rows = 0;
    int code_rows = 0;
    int phase_outlier_rows = 0;
    double phase_rms = 0.0;
    double code_rms = 0.0;
    double phase_chisq = 100.0;
    double max_phase_sigma = 0.0;
};

// MRTKLIB residual_test() semantics (mrtk_ppp_rtk.c:1040-1085): exclude
// individual DD phase residuals beyond outlier_sigma_gate sigmas from the
// chi-square sum, and when too few rows remain fall back to
// chisq = (accepted/total < 0.5) ? 100 : 0 instead of a hard 100.
struct FixValidationOptions {
    double outlier_sigma_gate = 0.0;  ///< 0 disables outlier exclusion
    bool mrtklib_chisq_fallback = false;
    /// MRTKLIB filter2_/residual_test innovation-variance basis
    /// (mrtk_ppp_rtk.c:1125, Q = H'*P*H + R): when non-null, each DD phase
    /// residual's gate/chi-square variance adds the state-projected term
    /// h_dd' P h_dd (position, troposphere, ionosphere and ambiguity
    /// partials) computed from this covariance. MRTKLIB evaluates post-fix
    /// residuals at xa but forms Q from the FLOAT posterior Pp, so callers
    /// should pass the float filter covariance here. Null keeps the
    /// historical measurement-only (R) basis.
    const MatrixXd* innovation_covariance = nullptr;
};

using TropMappingFunction =
    std::function<double(const Vector3d&, double, const GNSSTime&)>;
using AmbiguityIndexFunction = std::function<int(const SatelliteId&)>;
using AmbiguityResetFunction = std::function<void(const SatelliteId&, SignalType)>;
using ResolveAmbiguitiesFunction = std::function<bool()>;
using ValidateFixedSolutionFunction = std::function<FixValidationStats()>;

struct MeasurementRow {
    Eigen::RowVectorXd H;
    double residual = 0.0;
    double variance = 0.0;
    // MRTKLIB ddres()/ddcov() metadata. For a DD row, variance is the
    // diagonal Ri+Rj term, reference_variance is Ri, and rows sharing a
    // non-negative covariance block receive Ri on their off-diagonals.
    // Defaults preserve the historical diagonal-ZD/SD measurement model.
    double reference_variance = 0.0;
    int dd_covariance_block = -1;
    SatelliteId reference_satellite;
    SatelliteId satellite;
    bool is_phase = false;
    bool ambiguity_fresh = false;
    int freq_index = 0;
    int ambiguity_index = -1;
};

struct AmbiguityObservation {
    SatelliteId ambiguity_satellite;
    SignalType signal = SignalType::SIGNAL_TYPE_COUNT;
    double wavelength_m = 0.0;
    double carrier_phase_cycles = 0.0;
    double snr = 0.0;
};

struct MeasurementBuildResult {
    std::vector<MeasurementRow> measurements;
    std::vector<AmbiguityObservation> observed_ambiguities;
    std::set<SatelliteId> observed_iono_states;
};

struct AppliedOsrCorrections {
    double pseudorange_correction_m = 0.0;
    double carrier_phase_correction_m = 0.0;
};

struct KalmanUpdateStats {
    bool updated = false;
    int nobs = 0;
    VectorXd dx;
    VectorXd residuals;
    VectorXd variances;
    MatrixXd pre_anchor_covariance;
    std::set<SatelliteId> rejected_phase_ambiguities;
};

struct EpochUpdateResult {
    bool updated = false;
    bool insufficient_valid_satellites = false;
    KalmanUpdateStats update_stats;
};

struct AmbiguityResolutionResult {
    bool attempted = false;
    bool accepted = false;
    bool rejected_after_fix = false;
    FixValidationStats validation_stats;
};

struct EpochPreparationResult {
    bool ready = false;
    bool initialized_this_epoch = false;
};

struct ClasSlipDetectionStats {
    int lli_count = 0;
    int gf_count = 0;
    int mw_count = 0;
    int code_change_count = 0;
    int outage_resets = 0;
    int per_sat_outage_resets = 0;
    int total_resets = 0;
    std::set<SatelliteId> reset_satellites;
};

AppliedOsrCorrections selectAppliedOsrCorrections(
    const OSRCorrection& osr,
    int freq_index,
    ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy policy);

bool usesClasTropospherePrior(
    ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy policy);

std::vector<SatelliteId> collectResidualIonoSatellites(
    const ObservationData& obs,
    const SSRProducts& ssr_products);

EpochPreparationResult prepareEpochState(
    const ObservationData& obs,
    const PositionSolution& seed_solution,
    const SSRProducts& ssr_products,
    ppp_shared::PPPState& filter_state,
    bool& filter_initialized,
    GNSSTime& convergence_start_time,
    Vector3d& static_anchor_position,
    bool& has_static_anchor_position,
    const ppp_shared::PPPConfig& config,
    double modeled_zenith_troposphere_delay_m,
    bool has_last_processed_time,
    const GNSSTime& last_processed_time,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    std::map<SatelliteId, CLASDispersionCompensationInfo>& dispersion_compensation,
    std::map<SatelliteId, CLASPhaseBiasRepairInfo>& phase_bias_repair,
    double ambiguity_reset_variance);

void initializeFilterState(
    ppp_shared::PPPState& filter_state,
    const PositionSolution& seed_solution,
    const GNSSTime& time,
    const std::vector<SatelliteId>& iono_satellites,
    const ppp_shared::PPPConfig& config,
    double modeled_zenith_troposphere_delay_m,
    bool qzss_visible);

void syncSlipState(
    const ObservationData& obs,
    ppp_shared::PPPState& filter_state,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    std::map<SatelliteId, CLASDispersionCompensationInfo>& dispersion_compensation,
    std::map<SatelliteId, CLASPhaseBiasRepairInfo>& phase_bias_repair,
    double ambiguity_reset_variance,
    bool mark_dispersion_slip);

ClasSlipDetectionStats detectClasCycleSlips(
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    const ppp_shared::PPPConfig& config,
    double dt_seconds,
    ppp_shared::PPPState& filter_state,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    std::map<SatelliteId, CLASDispersionCompensationInfo>& dispersion_compensation,
    std::map<SatelliteId, CLASPhaseBiasRepairInfo>& phase_bias_repair,
    const AmbiguityResetFunction& ambiguity_reset_function,
    double ambiguity_reset_variance,
    bool debug_enabled = false);

void predictFilterState(
    ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    double dt,
    const Vector3d& seed_position_ecef,
    double seed_receiver_clock_bias_m,
    bool seed_valid,
    const std::set<SatelliteId>* observed_satellites = nullptr);

void markSlipCompensationFromAmbiguities(
    const ObservationData& obs,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    std::map<SatelliteId, CLASDispersionCompensationInfo>& dispersion_compensation);

void ensureAmbiguityStates(
    ppp_shared::PPPState& filter_state,
    const std::vector<OSRCorrection>& osr_corrections,
    double initial_variance = 3600.0);

void applyPendingPhaseBiasStateShifts(
    ppp_shared::PPPState& filter_state,
    const std::vector<OSRCorrection>& osr_corrections,
    std::map<SatelliteId, CLASPhaseBiasRepairInfo>& phase_bias_repair,
    bool debug_enabled = false);

MeasurementBuildResult buildEpochMeasurements(
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    const Vector3d& receiver_position,
    double receiver_clock_m,
    double trop_zenith,
    const std::map<std::string, std::string>& epoch_atmos,
    const TropMappingFunction& trop_mapping_function,
    const AmbiguityResetFunction& ambiguity_reset_function = {},
    bool debug_enabled = false,
    int reference_rank = 0);

KalmanUpdateStats applyMeasurementUpdate(
    ppp_shared::PPPState& filter_state,
    const std::vector<MeasurementRow>& measurements,
    const ppp_shared::PPPConfig& config,
    const PositionSolution* seed_solution = nullptr);

EpochUpdateResult runEpochMeasurementUpdate(
    const ObservationData& obs,
    const CLASEpochContext& epoch_context,
    ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    const PositionSolution& seed_solution,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const TropMappingFunction& trop_mapping_function,
    const AmbiguityResetFunction& ambiguity_reset_function,
    const AmbiguityIndexFunction& ambiguity_index_function,
    bool debug_enabled = false);

void updateObservedAmbiguities(
    const GNSSTime& time,
    const std::vector<AmbiguityObservation>& observed_ambiguities,
    const ppp_shared::PPPState& filter_state,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const AmbiguityIndexFunction& ambiguity_index_function);

FixValidationStats validateFixedSolution(
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    const ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    const TropMappingFunction& trop_mapping_function,
    const AmbiguityIndexFunction& ambiguity_index_function,
    bool debug_enabled = false,
    const FixValidationOptions& options = FixValidationOptions{});

AmbiguityResolutionResult resolveAndValidateAmbiguities(
    ppp_shared::PPPState& filter_state,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const ResolveAmbiguitiesFunction& resolve_ambiguities,
    const ValidateFixedSolutionFunction& validate_fixed_solution,
    bool debug_enabled = false);

void logUpdateSummary(
    const KalmanUpdateStats& update_stats,
    size_t satellite_count);

PositionSolution finalizeEpochSolution(
    const ppp_shared::PPPState& filter_state,
    const GNSSTime& time,
    bool fixed,
    double ar_ratio,
    int fixed_ambiguities,
    int num_satellites);

}  // namespace libgnss::ppp_clas
