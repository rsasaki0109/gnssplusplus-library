#pragma once

#include <libgnss++/algorithms/ppp_shared.hpp>

#include <functional>
#include <set>
#include <vector>

namespace libgnss::ppp_ar {

SatelliteId clasRealSatellite(const SatelliteId& satellite);
std::pair<GNSSSystem, int> ambiguityDdGroup(const SatelliteId& satellite);
double claslibRatioThresholdForNb(int nb);

struct MadocaArDesignTerm {
    SatelliteId satellite;
    int frequency_index = 0;
    int state_index = -1;
    double coefficient = 0.0;
};

struct MadocaArDesignRow {
    std::vector<MadocaArDesignTerm> terms;
    bool valid = false;
};

using AmbiguityStateIndexProvider = std::function<int(const SatelliteId&, int frequency_index)>;
using AmbiguityWavelengthProvider = std::function<double(const SatelliteId&, int frequency_index)>;

int madocaFrequencyAmbiguityStateIndex(
    const ppp_shared::PPPState& filter_state,
    const SatelliteId& satellite,
    int frequency_index);

AmbiguityStateIndexProvider makeMadocaFrequencyAmbiguityStateIndexProvider(
    const ppp_shared::PPPState& filter_state);

MadocaArDesignRow buildMadocaArDesignRow(
    const SatelliteId& satellite,
    const SatelliteId& reference_satellite,
    int first_frequency_index,
    int second_frequency_index,
    const AmbiguityStateIndexProvider& state_index_provider,
    const AmbiguityWavelengthProvider& wavelength_provider);

double evaluateMadocaArDesignRow(
    const MadocaArDesignRow& row,
    const VectorXd& state);

double madocaArDesignRowVariance(
    const MadocaArDesignRow& row,
    const MatrixXd& covariance);

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
    int skipped_scale = 0;
    int skipped_index = 0;
};

EligibleAmbiguities collectEligibleAmbiguities(
    const ppp_shared::PPPState& filter_state,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    int min_lock_count);

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
    WlnlGroupKey group{};
    bool valid = false;
};

struct WlnlFixAttempt {
    bool fixed = false;
    double ratio = 0.0;
    int nb = 0;
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
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const EligibleAmbiguities& eligible_ambiguities,
    const WlnlNlInfoProvider& provider,
    bool debug_enabled);

WlnlFixAttempt tryWlnlFix(
    const ppp_shared::PPPConfig& config,
    ppp_shared::PPPState& filter_state,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const std::vector<SatelliteId>& satellites,
    const std::vector<int>& state_indices,
    const std::map<SatelliteId, WlnlNlInfo>& nl_info,
    bool debug_enabled);

// MADOCALIB-style cascaded-AR wide-lane state constraint.
// For each pair of WL-fixed satellites (picked in SD form per system group),
// injects a single-SD pseudo-observation
//   y_SD_WL_cycles = sat.wl_fixed_integer - ref.wl_fixed_integer
// against the per-frequency ambiguity state via a sparse design row with
// 1/lambda coefficients.  Uses a scalar Kalman update with variance R
// (in cycles^2) to tighten the per-frequency ambiguity covariance before
// the N1 DD LAMBDA search.  Requires that per-frequency ambiguity states
// are active (state_index_provider returns >=0 for freq 0 AND 1).
struct MadocaWlConstraintSummary {
    int attempted_pairs = 0;
    int applied_constraints = 0;
    int skipped_invalid_row = 0;
    int skipped_no_covariance = 0;
    int skipped_large_innovation = 0;
};

MadocaWlConstraintSummary applyMadocaWideLaneStateConstraint(
    ppp_shared::PPPState& filter_state,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const AmbiguityStateIndexProvider& state_index_provider,
    const AmbiguityWavelengthProvider& wavelength_provider,
    double observation_variance_cycles_sq,
    bool debug_enabled);

// MADOCALIB-style cascaded-AR extra-wide-lane (EWL) state constraint.
// Operates on the per-frequency ambiguity state pairs (freq_index=1, 2) — the
// second and third frequencies, typically GPS L2+L5 or Galileo E5b+E5a/E6 —
// deriving the SD EWL float directly from the current state, rounding if the
// fractional part and variance satisfy the gates, and injecting the integer
// as a Kalman pseudo-observation.  Because EWL wavelengths are long
// (~5.86 m for GPS L2-L5), the rounding is robust and the resulting
// covariance tightening propagates into WL and N1 via the state correlation
// matrix.  This is the "STEP_EWL" stage of the MADOCALIB pipeline and is
// the primary driver of the native-vs-bridge AR parity gap.
struct MadocaEwlConstraintSummary {
    int candidates = 0;
    int applied_constraints = 0;
    int skipped_invalid_row = 0;
    int skipped_no_covariance = 0;
    int skipped_large_frac = 0;
    int skipped_large_sigma = 0;
};

MadocaEwlConstraintSummary applyMadocaExtraWideLaneStateConstraint(
    ppp_shared::PPPState& filter_state,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const AmbiguityStateIndexProvider& state_index_provider,
    const AmbiguityWavelengthProvider& wavelength_provider,
    double observation_variance_cycles_sq,
    double frac_gate_cycles,
    double sigma_gate_cycles,
    bool debug_enabled);

struct FixedNlObservation {
    double nl_phase_m = 0.0;
    double fixed_nl_cycles = 0.0;
    double lambda_nl_m = 0.0;
    Vector3d sat_pos = Vector3d::Zero();
    double sat_clk = 0.0;
    bool use_trop_model = true;
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
    double* position_shift_norm_m = nullptr);

struct FixedCarrierObservation {
    Vector3d satellite_position = Vector3d::Zero();
    double satellite_clock_bias_s = 0.0;
    double trop_mapping = 0.0;
    double modeled_trop_delay_m = 0.0;
    double modeled_zenith_trop_delay_m = 0.0;
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
