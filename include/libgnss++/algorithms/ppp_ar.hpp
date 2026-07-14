#pragma once

#include <libgnss++/algorithms/ppp_shared.hpp>

#include <cmath>
#include <functional>
#include <map>
#include <set>
#include <vector>

namespace libgnss::ppp_ar {

SatelliteId clasRealSatellite(const SatelliteId& satellite);
std::pair<GNSSSystem, int> ambiguityDdGroup(const SatelliteId& satellite);
double claslibRatioThresholdForNb(int nb);

// MRTKLIB PAR candidate gate: with the three configured frequency slots,
// satellites missing more than one accepted ambiguity are not trial-excluded.
std::vector<SatelliteId> selectMrtklibParCandidates(
    const std::vector<SatelliteId>& eligible_frequency_states,
    const std::map<SatelliteId, double>& satellite_elevations_rad,
    int min_active_frequency_states = 2);

// MRTKLIB clas.toml / mrtk_ppp_rtk.c holdamb() semantics (ARMODE_FIXHOLD).
constexpr double kMrtklibVarHoldAmbCycles2 = 0.001;
constexpr double kMrtklibHoldElevationMaskRad = 30.0 * M_PI / 180.0;
constexpr double kMrtklibArElevationMaskRad = 20.0 * M_PI / 180.0;
constexpr int kMrtklibMinFixCount = 0;

struct WlnlHoldConstraint {
    int ref_state = -1;
    int sat_state = -1;
    double fixed_dd_m = 0.0;
    double ambiguity_scale_m = 0.0;
    double ref_elevation_rad = 0.0;
    double sat_elevation_rad = 0.0;
    SatelliteId ref_satellite;
    SatelliteId sat_satellite;
};

// MRTKLIB rtk->nfix bookkeeping: counts consecutive validated fixes and keeps
// the constraints applied by the most recent holdamb() for slip housekeeping.
struct WlnlHoldState {
    bool active = false;
    int consecutive_fix_count = 0;
    std::vector<WlnlHoldConstraint> constraints;
};

// MRTKLIB holdamb() parity: constraints pull the float filter ambiguity DDs
// toward the values of the constrained fixed solution xa (fixed_state), the
// same source MRTKLIB uses (v = (xa[ref]-xa[i]) - (x[ref]-x[i])).
bool buildWlnlHoldConstraints(
    const ppp_shared::PPPState& fixed_state,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const std::map<SatelliteId, double>& satellite_elevations_rad,
    std::vector<WlnlHoldConstraint>& constraints,
    bool allow_l2_pseudo_states = false);

bool applyWlnlHoldAmbiguity(
    ppp_shared::PPPState& filter_state,
    const std::vector<WlnlHoldConstraint>& constraints,
    double hold_variance_cycles2 = kMrtklibVarHoldAmbCycles2,
    double hold_elevation_mask_rad = kMrtklibHoldElevationMaskRad);

bool wlnlHoldStillValid(
    const WlnlHoldState& hold,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states);

void clearWlnlHoldState(WlnlHoldState& hold);

// MRTKLIB testsnr() elevation-dependent mask (mrtk_obs.c:228-242, clas.toml L1/L2/L5).
bool clasKinematicSnrMasked(int freq_index, double elevation_rad, double snr_dbhz);

struct DdFixAttempt {
    bool fixed = false;
    double ratio = 0.0;
    double required_ratio = 0.0;
    int nb = 0;
    ppp_shared::PPPState state;
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo> ambiguities;
};

struct EligibleAmbiguities {
    std::vector<SatelliteId> satellites;
    std::vector<int> state_indices;
    std::vector<double> scales;
    int total_ambiguities = 0;
    int skipped_reinitialization = 0;
    int skipped_lock = 0;
    int skipped_slip_window = 0;
    int skipped_scale = 0;
    int skipped_index = 0;
};

EligibleAmbiguities collectEligibleAmbiguities(
    const ppp_shared::PPPState& filter_state,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    int min_lock_count,
    const GNSSTime& time = GNSSTime{},
    double slip_ar_exclusion_seconds = 10.0);

DdFixAttempt tryDirectDdFix(
    const ppp_shared::PPPConfig& config,
    const ppp_shared::PPPState& filter_state,
    const MatrixXd& pre_anchor_covariance,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const std::vector<SatelliteId>& satellites,
    const std::vector<int>& state_indices,
    const std::vector<double>& scales,
    const std::set<SatelliteId>& excluded_real_satellites,
    bool debug_enabled);

DdFixAttempt tryDirectDdFixWithPar(
    const ppp_shared::PPPConfig& config,
    const ppp_shared::PPPState& filter_state,
    const MatrixXd& pre_anchor_covariance,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const EligibleAmbiguities& eligible_ambiguities,
    const std::map<SatelliteId, double>& real_satellite_elevations,
    bool debug_enabled);

using WlnlGroupKey = std::pair<GNSSSystem, std::pair<int, int>>;

struct WlnlNlInfo {
    double nl_ambiguity_cycles = 0.0;
    double lambda_nl_m = 0.0;
    double lambda_wl_m = 0.0;
    double beta = 0.0;
    double alpha1 = 0.0;
    double alpha2 = 0.0;
    double iono_state_scale = 0.0;
    WlnlGroupKey group{};
    bool valid = false;
};

struct WlnlFixAttempt {
    bool fixed = false;
    bool has_constrained_state = false;
    bool state_lambda_solved = false;
    bool state_lambda_used = false;
    double ratio = 0.0;
    double state_lambda_ratio = 0.0;
    double state_required_ratio = 0.0;
    double state_dd_residual_norm = 0.0;
    double state_position_shift_m = 0.0;
    double state_wlnl_max_abs_delta_cycles = 0.0;
    double state_wlnl_max_fractional_cycles = 0.0;
    int state_dd_count = 0;
    int state_wlnl_mismatch_count = 0;
    int state_wlnl_noninteger_count = 0;
    int nb = 0;
    ppp_shared::PPPState constrained_state;
};

struct WlnlWideLaneFixSummary {
    int fixed_count = 0;
    int max_mw_count = 0;
};

struct WlnlPreparation {
    EligibleAmbiguities eligible_ambiguities;
    WlnlWideLaneFixSummary wl_summary;
    int min_lock_count = 0;
};

WlnlPreparation prepareWlnlCandidates(
    const ppp_shared::PPPConfig& config,
    const ppp_shared::PPPState& filter_state,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    bool use_ssr_products,
    const GNSSTime& time,
    bool debug_enabled);

WlnlWideLaneFixSummary applyWideLaneFixes(
    const ppp_shared::PPPConfig& config,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const std::vector<SatelliteId>& satellites,
    bool debug_enabled);

using WlnlNlInfoProvider = std::function<bool(const SatelliteId&, WlnlNlInfo&)>;

std::map<SatelliteId, WlnlNlInfo> buildWlnlNlInfoMap(
    const std::vector<SatelliteId>& satellites,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const WlnlNlInfoProvider& provider);

WlnlFixAttempt resolveWlnlFix(
    const ppp_shared::PPPConfig& config,
    ppp_shared::PPPState& filter_state,
    const MatrixXd& constraint_covariance,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const EligibleAmbiguities& eligible_ambiguities,
    const WlnlNlInfoProvider& provider,
    bool debug_enabled,
    const std::map<SatelliteId, double>* satellite_elevations_rad = nullptr);

WlnlFixAttempt resolveWlnlFix(
    const ppp_shared::PPPConfig& config,
    ppp_shared::PPPState& filter_state,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const EligibleAmbiguities& eligible_ambiguities,
    const WlnlNlInfoProvider& provider,
    bool debug_enabled,
    const std::map<SatelliteId, double>* satellite_elevations_rad = nullptr);

WlnlFixAttempt tryWlnlFix(
    const ppp_shared::PPPConfig& config,
    ppp_shared::PPPState& filter_state,
    const MatrixXd& constraint_covariance,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const std::vector<SatelliteId>& satellites,
    const std::vector<int>& state_indices,
    const std::map<SatelliteId, WlnlNlInfo>& nl_info,
    bool debug_enabled,
    const std::map<SatelliteId, double>* satellite_elevations_rad = nullptr,
    const std::set<SatelliteId>& excluded_real_satellites = {});

WlnlFixAttempt tryWlnlFixWithPar(
    const ppp_shared::PPPConfig& config,
    ppp_shared::PPPState& filter_state,
    const MatrixXd& constraint_covariance,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const std::vector<SatelliteId>& satellites,
    const std::vector<int>& state_indices,
    const std::map<SatelliteId, WlnlNlInfo>& nl_info,
    bool debug_enabled,
    const std::map<SatelliteId, double>* satellite_elevations_rad = nullptr);

struct FixedNlObservation {
    SatelliteId satellite;
    SignalType signal1 = SignalType::SIGNAL_TYPE_COUNT;
    SignalType signal2 = SignalType::SIGNAL_TYPE_COUNT;
    double frequency1_hz = 0.0;
    double frequency2_hz = 0.0;
    double raw_phase1_m = 0.0;
    double raw_phase2_m = 0.0;
    double corrected_phase1_m = 0.0;
    double corrected_phase2_m = 0.0;
    double raw_code1_m = 0.0;
    double raw_code2_m = 0.0;
    double corrected_code1_m = 0.0;
    double corrected_code2_m = 0.0;
    double cpc1_m = 0.0;
    double cpc2_m = 0.0;
    double prc1_m = 0.0;
    double prc2_m = 0.0;
    double receiver_ant1_m = 0.0;
    double receiver_ant2_m = 0.0;
    double nl_phase_m = 0.0;
    double fixed_nl_cycles = 0.0;
    double lambda_nl_m = 0.0;
    Vector3d sat_pos = Vector3d::Zero();
    double sat_clk = 0.0;
    bool use_trop_model = true;
    double extra_prediction_m = 0.0;
    double cpc_nl_m = 0.0;
    double osr_trop_correction_m = 0.0;
    double osr_nl_iono_m = 0.0;
    double receiver_ant_nl_m = 0.0;
};

using FixedNlObservationProvider = std::function<bool(
    const SatelliteId&,
    const ppp_shared::PPPAmbiguityInfo&,
    FixedNlObservation&)>;

std::vector<FixedNlObservation> buildFixedNlObservations(
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const FixedNlObservationProvider& provider);

using TropMappingFunction = std::function<double(const Vector3d&, double, const GNSSTime&)>;

bool solveFixedNlPosition(
    const std::vector<FixedNlObservation>& fixed_observations,
    const Vector3d& initial_position,
    double initial_clock_m,
    double trop_zenith,
    const GNSSTime& time,
    const TropMappingFunction& trop_mapping_function,
    Vector3d& fixed_position,
    double* position_shift_norm_m = nullptr,
    double* final_clock_m = nullptr);

struct FixedCarrierObservation {
    Vector3d satellite_position = Vector3d::Zero();
    double satellite_clock_bias_s = 0.0;
    double trop_mapping = 0.0;
    double modeled_trop_delay_m = 0.0;
    double carrier_phase_if = 0.0;
    double variance_cp = 0.0;
    double ambiguity_m = 0.0;
    double system_clock_offset_m = 0.0;
    double ionosphere_m = 0.0;
};

using FixedCarrierObservationProvider = std::function<bool(size_t, FixedCarrierObservation&)>;

std::vector<FixedCarrierObservation> buildFixedCarrierObservations(
    size_t candidate_count,
    const FixedCarrierObservationProvider& provider);

bool solveFixedCarrierPosition(
    const std::vector<FixedCarrierObservation>& fixed_observations,
    const Vector3d& initial_position,
    double initial_clock_m,
    double trop_zenith,
    bool estimate_troposphere,
    Vector3d& fixed_position);

}  // namespace libgnss::ppp_ar
