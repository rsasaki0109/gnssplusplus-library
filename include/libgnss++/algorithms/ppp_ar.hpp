#pragma once

#include <libgnss++/algorithms/ppp_shared.hpp>

#include <functional>
#include <set>
#include <vector>

namespace libgnss::ppp_ar {

SatelliteId clasRealSatellite(const SatelliteId& satellite);
std::pair<GNSSSystem, int> ambiguityDdGroup(const SatelliteId& satellite);
double claslibRatioThresholdForNb(int nb);

struct DdFixAttempt {
    bool fixed = false;
    double ratio = 0.0;
    double required_ratio = 0.0;
    int nb = 0;
    ppp_shared::PPPState state;
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo> ambiguities;
    bool has_hold_state = false;
    ppp_shared::PPPState hold_state;
};

struct EligibleAmbiguities {
    std::vector<SatelliteId> satellites;
    std::vector<int> state_indices;
    std::vector<double> scales;
    int total_ambiguities = 0;
    int skipped_reinitialization = 0;
    int skipped_lock = 0;
    int skipped_stale = 0;
    int skipped_scale = 0;
    int skipped_index = 0;
};

EligibleAmbiguities collectEligibleAmbiguities(
    const ppp_shared::PPPState& filter_state,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    int min_lock_count,
    const GNSSTime* observation_time = nullptr);

DdFixAttempt tryDirectDdFix(
    const ppp_shared::PPPConfig& config,
    const ppp_shared::PPPState& filter_state,
    const MatrixXd& pre_anchor_covariance,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const std::vector<SatelliteId>& satellites,
    const std::vector<int>& state_indices,
    const std::vector<double>& scales,
    const std::set<SatelliteId>& excluded_real_satellites,
    bool debug_enabled,
    const GNSSTime* observation_time = nullptr);

DdFixAttempt tryDirectDdFixWithPar(
    const ppp_shared::PPPConfig& config,
    const ppp_shared::PPPState& filter_state,
    const MatrixXd& pre_anchor_covariance,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const EligibleAmbiguities& eligible_ambiguities,
    const std::map<SatelliteId, double>& real_satellite_elevations,
    bool debug_enabled,
    const GNSSTime* observation_time = nullptr);

using WlnlGroupKey = std::pair<GNSSSystem, std::pair<int, int>>;

struct WlnlNlInfo {
    double nl_ambiguity_cycles = 0.0;
    double lambda_nl_m = 0.0;
    double lambda_wl_m = 0.0;
    double beta = 0.0;
    double alpha1 = 0.0;
    double alpha2 = 0.0;
    double l1_phase_corr_m = 0.0;
    double l2_phase_corr_m = 0.0;
    double nl_phase_m = 0.0;
    double predicted_m = 0.0;
    WlnlGroupKey group{};
    bool valid = false;
};

struct WlnlFixAttempt {
    bool fixed = false;
    double ratio = 0.0;
    int nb = 0;
    bool has_fixed_state = false;
    double fixed_state_dd_gap_cycles = 1e9;
    ppp_shared::PPPState fixed_state;
    struct DdConstraint {
        SatelliteId ref_satellite;
        SatelliteId sat_satellite;
        double l1_dd_ambiguity_m = 0.0;
        double l2_dd_ambiguity_m = 0.0;
    };
    std::vector<DdConstraint> dd_constraints;
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

struct FixedNlObservation {
    double nl_phase_m = 0.0;
    double fixed_nl_cycles = 0.0;
    double lambda_nl_m = 0.0;
    Vector3d sat_pos = Vector3d::Zero();
    double sat_clk = 0.0;
    double system_clock_offset_m = 0.0;
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
