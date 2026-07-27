#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/algorithms/rtk_ar_evaluation.hpp>
#include <libgnss++/algorithms/rtk_ar_selection.hpp>
#include <libgnss++/algorithms/lambda.hpp>
#include <libgnss++/algorithms/rtk_cp_pr_gate.hpp>
#include <libgnss++/algorithms/rtk_ddpr_anchor.hpp>
#include <libgnss++/algorithms/rtk_measurement.hpp>
#include <libgnss++/algorithms/rtk_selection.hpp>
#include <libgnss++/algorithms/rtk_ins_time_update.hpp>
#include <libgnss++/algorithms/rtk_tdcp_diagnostics.hpp>
#include <libgnss++/algorithms/rtk_update.hpp>
#include <libgnss++/algorithms/spp_velocity.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/signal_policy.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/models/troposphere.hpp>
#include <iostream>
#include <iterator>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>

namespace libgnss {

namespace {

constexpr GNSSSystem kRTKSupportedSystems[] = {
    GNSSSystem::GPS,
    GNSSSystem::GLONASS,
    GNSSSystem::Galileo,
    GNSSSystem::BeiDou,
    GNSSSystem::QZSS,
};

constexpr double kGlonassHWBiasInitialVariance = 1.0;  // (m/MHz)^2
constexpr double kGlonassHWBiasProcessNoise = 1e-12;   // (m/MHz)^2 / s

bool isAmbiguityResolutionSystem(const RTKProcessor::RTKConfig& config, GNSSSystem system) {
    return system == GNSSSystem::GPS ||
           (system == GNSSSystem::GLONASS &&
            config.glonass_ar_mode != RTKProcessor::RTKConfig::GlonassARMode::OFF) ||
           system == GNSSSystem::Galileo ||
           system == GNSSSystem::BeiDou ||
           system == GNSSSystem::QZSS;
}

bool usesHoldAmbiguitySystem(const RTKProcessor::RTKConfig& config, GNSSSystem system) {
    return isAmbiguityResolutionSystem(config, system);
}

bool requiresMatchedCarrierWavelength(const RTKProcessor::RTKConfig& config, GNSSSystem system) {
    return !(system == GNSSSystem::GLONASS &&
             config.glonass_ar_mode != RTKProcessor::RTKConfig::GlonassARMode::OFF);
}

bool usesGlonassAutocal(const RTKProcessor::RTKConfig& config) {
    return config.enable_glonass &&
           config.glonass_ar_mode == RTKProcessor::RTKConfig::GlonassARMode::AUTOCAL;
}

bool usesEstimatedIono(const RTKProcessor::RTKConfig& config) {
    return config.ionoopt == RTKProcessor::RTKConfig::IonoOpt::EST;
}

bool isMovingBasePositionMode(const RTKProcessor::RTKConfig& config) {
    return config.position_mode == RTKProcessor::RTKConfig::PositionMode::MOVING_BASE;
}

bool isDynamicPositionMode(const RTKProcessor::RTKConfig& config) {
    return config.position_mode != RTKProcessor::RTKConfig::PositionMode::STATIC;
}

bool isBeiDouGeoSatellite(const SatelliteId& sat) {
    return signal_policy::isBeiDouGeoSatellite(sat);
}

bool isUsableRTKSatellite(const SatelliteId& sat) {
    // Start with MEO/IGSO BeiDou support. GEO still needs tighter handling.
    return !isBeiDouGeoSatellite(sat);
}

bool isEnabledRTKSystem(const RTKProcessor::RTKConfig& config, GNSSSystem system) {
    if (system == GNSSSystem::GLONASS) {
        return config.enable_glonass;
    }
    if (system == GNSSSystem::BeiDou) {
        return config.enable_beidou;
    }
    return true;
}

double combinedSnrDbHz(double rover_snr, double base_snr) {
    const bool rover_valid = std::isfinite(rover_snr) && rover_snr > 0.0;
    const bool base_valid = std::isfinite(base_snr) && base_snr > 0.0;
    if (rover_valid && base_valid) {
        return std::min(rover_snr, base_snr);
    }
    if (rover_valid) {
        return rover_snr;
    }
    if (base_valid) {
        return base_snr;
    }
    return 0.0;
}

bool isPrimaryRTKSignal(GNSSSystem system, SignalType signal) {
    return signal_policy::isPrimarySignal(system, signal);
}

bool isSecondaryRTKSignal(GNSSSystem system, SignalType signal) {
    return signal_policy::isSecondarySignal(system, signal);
}

// Phase 18 Step 3: L5-class signal slot, distinct from L2 secondary.
bool isL5RTKSignal(GNSSSystem system, SignalType signal) {
    return signal_policy::isL5Signal(system, signal);
}

int signalSelectionPriority(GNSSSystem system, SignalType signal, bool primary) {
    return signal_policy::signalPriority(system, signal, primary);
}

const Observation* selectPreferredObservation(
    GNSSSystem system,
    const std::vector<const Observation*>& candidates,
    bool primary) {
    const Observation* best = nullptr;
    int best_priority = 1000;
    for (const auto* obs : candidates) {
        if (obs == nullptr) continue;
        const int priority = signalSelectionPriority(system, obs->signal, primary);
        if (priority < best_priority) {
            best = obs;
            best_priority = priority;
        }
    }
    return best;
}

bool selectMatchedObservationPair(
    GNSSSystem system,
    const std::vector<const Observation*>& rover_candidates,
    const std::vector<const Observation*>& base_candidates,
    bool primary,
    const Observation*& rover_selected,
    const Observation*& base_selected) {
    rover_selected = nullptr;
    base_selected = nullptr;
    int best_priority = 1000;

    for (const auto* rover_obs : rover_candidates) {
        if (rover_obs == nullptr) continue;
        for (const auto* base_obs : base_candidates) {
            if (base_obs == nullptr) continue;
            if (rover_obs->signal != base_obs->signal) continue;
            const int priority = signalSelectionPriority(system, rover_obs->signal, primary);
            if (priority < best_priority) {
                rover_selected = rover_obs;
                base_selected = base_obs;
                best_priority = priority;
            }
        }
    }
    return rover_selected != nullptr && base_selected != nullptr;
}

double ionoFreeCoeff1(double f1, double f2) {
    const double denom = f1 * f1 - f2 * f2;
    return std::abs(denom) > 0.0 ? (f1 * f1) / denom : 0.0;
}

double ionoFreeCoeff2(double f1, double f2) {
    const double denom = f1 * f1 - f2 * f2;
    return std::abs(denom) > 0.0 ? -(f2 * f2) / denom : 0.0;
}

double wideLaneWavelength(double f1, double f2) {
    const double denom = f1 - f2;
    return std::abs(denom) > 0.0 ? constants::SPEED_OF_LIGHT / denom : 0.0;
}

double narrowLaneWavelength(double f1, double f2) {
    const double c1 = ionoFreeCoeff1(f1, f2);
    const double c2 = ionoFreeCoeff2(f1, f2);
    if (c1 == 0.0 && c2 == 0.0) {
        return 0.0;
    }
    return c1 * (constants::SPEED_OF_LIGHT / f1) + c2 * (constants::SPEED_OF_LIGHT / f2);
}

double ionoFrequencyScale(int freq, double l1_frequency_hz, double current_frequency_hz) {
    if (freq == 0 || l1_frequency_hz <= 0.0 || current_frequency_hz <= 0.0) {
        return 1.0;
    }
    const double ratio = l1_frequency_hz / current_frequency_hz;
    return ratio * ratio;
}

inline double distanceToNearestInteger(double value) {
    return std::abs(value - std::round(value));
}

bool applyAmbiguityConstraintUpdate(VectorXd& head_state,
                                    VectorXd& dd_float,
                                    MatrixXd& Qb,
                                    MatrixXd& Qab,
                                    int lhs_index,
                                    int rhs_index,
                                    double fixed_difference,
                                    double measurement_variance) {
    if (lhs_index < 0 || rhs_index < 0 ||
        lhs_index >= dd_float.size() || rhs_index >= dd_float.size() ||
        lhs_index >= Qb.rows() || rhs_index >= Qb.rows() ||
        lhs_index >= Qb.cols() || rhs_index >= Qb.cols() ||
        lhs_index >= Qab.cols() || rhs_index >= Qab.cols()) {
        return false;
    }

    VectorXd h = VectorXd::Zero(dd_float.size());
    h(lhs_index) = 1.0;
    h(rhs_index) = -1.0;
    const VectorXd Qb_h = Qb * h;
    const auto h_Qb = h.transpose() * Qb;
    const VectorXd Qab_h = Qab * h;
    const double innovation_variance =
        h.dot(Qb_h) + std::max(measurement_variance, 1e-6);
    if (!std::isfinite(innovation_variance) || innovation_variance <= 0.0) {
        return false;
    }

    const double innovation = fixed_difference - h.dot(dd_float);
    dd_float += (Qb_h / innovation_variance) * innovation;
    head_state += (Qab_h / innovation_variance) * innovation;
    Qb -= (Qb_h * h_Qb) / innovation_variance;
    Qab -= (Qab_h * h_Qb) / innovation_variance;
    Qb = (Qb + Qb.transpose()) * 0.5;
    for (int i = 0; i < Qb.rows(); ++i) {
        if (Qb(i, i) < 1e-6) {
            Qb(i, i) = 1e-6;
        }
    }
    return dd_float.allFinite() && head_state.allFinite() &&
           Qb.allFinite() && Qab.allFinite();
}

double glonassInterChannelBiasMeters(const RTKProcessor::RTKConfig& config,
                                     GNSSSystem ref_system,
                                     GNSSSystem sat_system,
                                     double ref_frequency_hz,
                                     double sat_frequency_hz,
                                     int freq) {
    if (ref_system != GNSSSystem::GLONASS || sat_system != GNSSSystem::GLONASS) {
        return 0.0;
    }
    if (ref_frequency_hz <= 0.0 || sat_frequency_hz <= 0.0) {
        return 0.0;
    }
    const double df_mhz = (ref_frequency_hz - sat_frequency_hz) / 1e6;
    const double slope = (freq == 0) ? config.glonass_icb_l1_m_per_mhz
                                     : config.glonass_icb_l2_m_per_mhz;
    return slope * df_mhz;
}

SPPProcessor::SPPConfig makeRTKSppConfig(const RTKProcessor::RTKConfig& rtk_config) {
    SPPProcessor::SPPConfig config;
    config.use_multi_constellation = true;
    config.enable_glonass = rtk_config.enable_glonass;
    config.enable_beidou = rtk_config.enable_beidou;
    return config;
}

SPPProcessor makeRTKSppProcessor(const RTKProcessor::RTKConfig& rtk_config) {
    return SPPProcessor(makeRTKSppConfig(rtk_config));
}

}  // namespace

// Delegate to extracted modules
static inline double tropModel(const Vector3d& pos_ecef, double elevation) {
    return models::tropDelaySaastamoinen(pos_ecef, elevation);
}

// Geometric range with Sagnac correction (delegated to coordinates.hpp)
static inline double geodist_range(const Vector3d& rs, const Vector3d& rr) {
    return geodist(rs, rr);
}

RTKProcessor::RTKProcessor() : spp_processor_(makeRTKSppProcessor(rtk_config_)) { filter_initialized_ = false; }
RTKProcessor::RTKProcessor(const RTKConfig& rtk_config)
    : rtk_config_(rtk_config), spp_processor_(makeRTKSppProcessor(rtk_config_)) { filter_initialized_ = false; }

void RTKProcessor::setRTKConfig(const RTKConfig& config) {
    rtk_config_ = config;
    syncSPPConfig();
    // Phase 2a: cmc_suspect_tracker_ is lazily constructed with the
    // cmc_ref_level_m threshold captured at construction time; drop it so a
    // config change (e.g. --cmc-ref-level) takes effect on the next epoch
    // instead of being silently ignored.
    cmc_suspect_tracker_.reset();
}

void RTKProcessor::syncSPPConfig() {
    spp_processor_.setSPPConfig(makeRTKSppConfig(rtk_config_));
}

bool RTKProcessor::initialize(const ProcessorConfig& config) {
    config_ = config;
    syncSPPConfig();
    spp_processor_.initialize(config);
    reset();
    return true;
}

PositionSolution RTKProcessor::processEpoch(const ObservationData& rover_obs, const NavigationData& nav) {
    return spp_processor_.processEpoch(rover_obs, nav);
}

void RTKProcessor::reset() {
    filter_initialized_ = false;
    adaptive_noise_tracker_.clear();
    debug_telemetry_ = EpochDebugTelemetry{};
    debug_telemetry_.prior_held_integer_count = static_cast<int>(last_dd_fixed_.size());
    debug_telemetry_.prior_held_pair_count = static_cast<int>(last_best_subset_.size());
    debug_telemetry_.prior_consecutive_fix_count = consecutive_fix_count_;
    debug_telemetry_.prior_tracked_ambiguity_count =
        static_cast<int>(lock_count_l1_.size() +
                         lock_count_l2_.size() +
                         lock_count_l5_.size());
    filter_state_ = RTKState{};
    filter_state_.next_state_idx = REAL_STATES + IONO_STATES;
    ambiguity_states_.clear();
    lock_count_l1_.clear();
    lock_count_l2_.clear();
    lock_count_l5_.clear();  // Phase 18 Step 2: clear L5 lock counts on reset
    has_fixed_solution_ = false;
    has_last_fixed_position_ = false;
    has_last_fixed_time_ = false;
    has_last_solution_position_ = false;
    has_last_trusted_position_ = false;
    has_fixed_update_gate_previous_solution_ = false;
    has_ref_satellite_ = false;
    has_last_epoch_ = false;
    has_last_trusted_time_ = false;
    has_prev_trusted_position_ = false;
    has_last_doppler_velocity_ = false;
    has_doppler_continuity_position_ = false;
    current_epoch_nlos_fraction_ = std::numeric_limits<double>::quiet_NaN();
    current_sat_data_.clear();
    gf_l1l2_history_.clear();
    doppler_phase_history_l1_m_.clear();
    doppler_phase_history_l2_m_.clear();
    code_phase_history_l1_m_.clear();
    code_phase_history_l2_m_.clear();
    tdcp_history_l1_.clear();
    tdcp_history_l2_.clear();
    tdcp_history_l5_.clear();
    cmc_suspect_tracker_.reset();
    cmc_ref_hysteresis_by_system_.clear();
    cmc_aware_ref_by_system_.clear();
    cmc_ref_suspect_epoch_count_ = 0;
    cmc_ref_switch_count_ = 0;
    has_external_position_time_update_ = false;
    has_external_velocity_time_update_ = false;
    ins_time_update_applied_count_ = 0;
    ins_time_update_rejected_count_ = 0;
    ins_time_update_applied_last_epoch_ = false;
    position_correction_count_ = 0;
    position_correction_sum_sq_m2_ = 0.0;
    consecutive_cp_pr_gate_rejections_ = 0;
    has_last_ddpr_anchor_ = false;
    consecutive_fix_count_ = 0;
    consecutive_float_count_ = 0;
    consecutive_nonfix_count_ = 0;
    consecutive_high_float_residual_count_ = 0;
    consecutive_high_fixed_residual_count_ = 0;
    adaptive_dynamic_slip_hold_count_ = 0;
    last_ar_ratio_ = 0.0;
    last_num_fixed_ambiguities_ = 0;
    current_update_diagnostics_ = RTKUpdateDiagnostics{};
    std::lock_guard<std::mutex> lock(stats_mutex_);
    total_epochs_processed_ = 0;
    fixed_solutions_ = 0;
    float_solutions_ = 0;
}

ProcessorStats RTKProcessor::getStats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ProcessorStats stats;
    stats.total_epochs = total_epochs_processed_;
    stats.valid_solutions = fixed_solutions_ + float_solutions_;
    stats.fixed_solutions = fixed_solutions_;
    return stats;
}

bool RTKProcessor::getFloatPosteriorPosition(
    Vector3d& position_ecef, Matrix3d& position_covariance_ecef) const {
    if (!filter_initialized_ || !base_position_known_ ||
        filter_state_.state.size() < BASE_STATES ||
        filter_state_.covariance.rows() < BASE_STATES ||
        filter_state_.covariance.cols() < BASE_STATES) {
        return false;
    }
    position_ecef = base_position_ + filter_state_.state.head<BASE_STATES>();
    position_covariance_ecef =
        filter_state_.covariance.topLeftCorner<BASE_STATES, BASE_STATES>();
    return position_ecef.allFinite() && position_covariance_ecef.allFinite();
}

bool RTKProcessor::getLastDdPrAnchor(
    Vector3d& position_ecef, Matrix3d& position_covariance_ecef, GNSSTime& time) const {
    if (!has_last_ddpr_anchor_) {
        return false;
    }
    position_ecef = last_ddpr_anchor_position_ecef_;
    position_covariance_ecef = last_ddpr_anchor_covariance_ecef_;
    time = last_ddpr_anchor_time_;
    return true;
}

// RTKLIB varerr: SD measurement error variance
double RTKProcessor::varerr(double elevation, bool is_phase, double snr_dbhz) const {
    double sin_el = std::sin(elevation);
    if (sin_el < 0.1) sin_el = 0.1;
    double a = is_phase ? rtk_config_.carrier_phase_sigma : rtk_config_.pseudorange_sigma;
    double b = is_phase ? rtk_config_.carrier_phase_sigma : rtk_config_.pseudorange_sigma;
    double variance = 2.0 * (a * a + b * b / (sin_el * sin_el));
    const double snr_min_baseline = rtk_config_.snr_min_baseline_m;
    const bool baseline_passes_snr_floor =
        !std::isfinite(snr_min_baseline) ||
        snr_min_baseline <= 0.0 ||
        (filter_state_.state.size() >= BASE_STATES &&
         std::isfinite(filter_state_.state.head<3>().norm()) &&
         filter_state_.state.head<3>().norm() >= snr_min_baseline);
    if (rtk_config_.enable_snr_weighting &&
        baseline_passes_snr_floor &&
        std::isfinite(snr_dbhz) &&
        snr_dbhz > 0.0 &&
        rtk_config_.snr_reference_dbhz > 0.0 &&
        rtk_config_.snr_max_variance_scale >= 1.0) {
        const double snr_deficit_db = std::max(0.0, rtk_config_.snr_reference_dbhz - snr_dbhz);
        const double variance_scale =
            std::clamp(std::pow(10.0, snr_deficit_db / 10.0),
                       1.0,
                       rtk_config_.snr_max_variance_scale);
        variance *= variance_scale;
    }
    return variance;
}

double RTKProcessor::elevationWeight(double elevation) const {
    double sin_el = std::sin(elevation);
    if (sin_el < 0.1) sin_el = 0.1;
    return 1.0 / (sin_el * sin_el);
}

// ============================================================
// State vector management (SD ambiguities)
// ============================================================
void RTKProcessor::expandState(int new_size) {
    int old_size = filter_state_.state.size();
    if (new_size <= old_size) return;
    VectorXd new_state = VectorXd::Zero(new_size);
    MatrixXd new_cov = MatrixXd::Zero(new_size, new_size);
    if (old_size > 0) {
        new_state.head(old_size) = filter_state_.state;
        new_cov.topLeftCorner(old_size, old_size) = filter_state_.covariance;
    }
    filter_state_.state = new_state;
    filter_state_.covariance = new_cov;
}

int RTKProcessor::getOrCreateN1Index(const SatelliteId& sat, double initial_value) {
    int idx = IB(sat, 0);
    if (filter_state_.state(idx) != 0.0) {
        filter_state_.n1_indices[sat] = idx;
        return idx;
    }
    filter_state_.n1_indices[sat] = idx;
    filter_state_.state(idx) = initial_value;
    for (int j = 0; j < filter_state_.state.size(); ++j) {
        filter_state_.covariance(idx, j) = 0.0;
        filter_state_.covariance(j, idx) = 0.0;
    }
    filter_state_.covariance(idx, idx) = 900.0;
    return idx;
}

int RTKProcessor::getOrCreateN2Index(const SatelliteId& sat, double initial_value) {
    int idx = IB(sat, 1);
    if (filter_state_.state(idx) != 0.0) {
        filter_state_.n2_indices[sat] = idx;
        return idx;
    }
    filter_state_.n2_indices[sat] = idx;
    filter_state_.state(idx) = initial_value;
    for (int j = 0; j < filter_state_.state.size(); ++j) {
        filter_state_.covariance(idx, j) = 0.0;
        filter_state_.covariance(j, idx) = 0.0;
    }
    filter_state_.covariance(idx, idx) = 900.0;
    return idx;
}

// Phase 18 Step 4: parallel to N1/N2 index creators.
// IB(sat, 2) maps into the L5 slot of the state vector (FREQ_SLOTS=3 reserved by Step 2).
int RTKProcessor::getOrCreateN5Index(const SatelliteId& sat, double initial_value) {
    int idx = IB(sat, 2);
    if (filter_state_.state(idx) != 0.0) {
        filter_state_.n5_indices[sat] = idx;
        return idx;
    }
    filter_state_.n5_indices[sat] = idx;
    filter_state_.state(idx) = initial_value;
    for (int j = 0; j < filter_state_.state.size(); ++j) {
        filter_state_.covariance(idx, j) = 0.0;
        filter_state_.covariance(j, idx) = 0.0;
    }
    filter_state_.covariance(idx, idx) = 900.0;
    return idx;
}

int RTKProcessor::getOrCreateIonoIndex(const SatelliteId& sat, double initial_value) {
    int idx = II(sat);
    if (filter_state_.covariance(idx, idx) > 0.0) {
        filter_state_.iono_indices[sat] = idx;
        return idx;
    }
    filter_state_.iono_indices[sat] = idx;
    // Keep the state active in the sparse Kalman path even if the initial iono estimate is near zero.
    filter_state_.state(idx) = std::abs(initial_value) > 1e-6 ? initial_value : 1e-3;
    for (int j = 0; j < filter_state_.state.size(); ++j) {
        filter_state_.covariance(idx, j) = 0.0;
        filter_state_.covariance(j, idx) = 0.0;
    }
    filter_state_.covariance(idx, idx) = 100.0;
    return idx;
}

void RTKProcessor::removeSatelliteFromState(const SatelliteId& sat) {
    auto it0 = filter_state_.iono_indices.find(sat);
    if (it0 != filter_state_.iono_indices.end()) {
        int idx = it0->second;
        filter_state_.state(idx) = 0.0;
        for (int j = 0; j < filter_state_.state.size(); ++j) {
            filter_state_.covariance(idx, j) = 0.0;
            filter_state_.covariance(j, idx) = 0.0;
        }
        filter_state_.iono_indices.erase(it0);
    }
    auto it1 = filter_state_.n1_indices.find(sat);
    if (it1 != filter_state_.n1_indices.end()) {
        int idx = it1->second;
        filter_state_.state(idx) = 0.0;
        for (int j = 0; j < filter_state_.state.size(); ++j) {
            filter_state_.covariance(idx, j) = 0.0;
            filter_state_.covariance(j, idx) = 0.0;
        }
        filter_state_.n1_indices.erase(it1);
    }
    auto it2 = filter_state_.n2_indices.find(sat);
    if (it2 != filter_state_.n2_indices.end()) {
        int idx = it2->second;
        filter_state_.state(idx) = 0.0;
        for (int j = 0; j < filter_state_.state.size(); ++j) {
            filter_state_.covariance(idx, j) = 0.0;
            filter_state_.covariance(j, idx) = 0.0;
        }
        filter_state_.n2_indices.erase(it2);
    }
    // Phase 18 Step 2: erase L5 ambiguity slot if present (no-op until Step 3+ populates n5_indices).
    auto it5 = filter_state_.n5_indices.find(sat);
    if (it5 != filter_state_.n5_indices.end()) {
        int idx = it5->second;
        filter_state_.state(idx) = 0.0;
        for (int j = 0; j < filter_state_.state.size(); ++j) {
            filter_state_.covariance(idx, j) = 0.0;
            filter_state_.covariance(j, idx) = 0.0;
        }
        filter_state_.n5_indices.erase(it5);
    }
}

// ============================================================
// Satellite data collection
// ============================================================
std::map<SatelliteId, RTKProcessor::SatelliteData> RTKProcessor::collectSatelliteData(
    const ObservationData& rover_obs, const ObservationData& base_obs, const NavigationData& nav) {
    std::map<SatelliteId, SatelliteData> result;
    std::map<SatelliteId, std::vector<const Observation*>> rover_l1, rover_l2, base_l1, base_l2;
    // Phase 18 Step 3: L5 collection (only populated when enable_l5=true).
    std::map<SatelliteId, std::vector<const Observation*>> rover_l5, base_l5;
    const bool l5_enabled = rtk_config_.enable_l5;
    for (const auto& obs : rover_obs.observations) {
        if (!isEnabledRTKSystem(rtk_config_, obs.satellite.system)) continue;
        if (!isUsableRTKSatellite(obs.satellite)) continue;
        if (isPrimaryRTKSignal(obs.satellite.system, obs.signal) &&
            obs.has_carrier_phase && obs.has_pseudorange) {
            rover_l1[obs.satellite].push_back(&obs);
        }
        if (isSecondaryRTKSignal(obs.satellite.system, obs.signal) &&
            obs.has_carrier_phase && obs.has_pseudorange) {
            // When L5 enabled, exclude L5-class obs from L2 slot so they don't
            // displace true L2C signals or pollute the L2 wavelength.
            if (l5_enabled && isL5RTKSignal(obs.satellite.system, obs.signal)) {
                rover_l5[obs.satellite].push_back(&obs);
            } else {
                rover_l2[obs.satellite].push_back(&obs);
            }
        }
    }
    for (const auto& obs : base_obs.observations) {
        if (!isEnabledRTKSystem(rtk_config_, obs.satellite.system)) continue;
        if (!isUsableRTKSatellite(obs.satellite)) continue;
        if (isPrimaryRTKSignal(obs.satellite.system, obs.signal) &&
            obs.has_carrier_phase && obs.has_pseudorange) {
            base_l1[obs.satellite].push_back(&obs);
        }
        if (isSecondaryRTKSignal(obs.satellite.system, obs.signal) &&
            obs.has_carrier_phase && obs.has_pseudorange) {
            if (l5_enabled && isL5RTKSignal(obs.satellite.system, obs.signal)) {
                base_l5[obs.satellite].push_back(&obs);
            } else {
                base_l2[obs.satellite].push_back(&obs);
            }
        }
    }
    Vector3d rover_pos_for_clk = rover_obs.receiver_position;
    if (filter_initialized_ && filter_state_.state.size() >= 3) {
        Vector3d predicted_rover = base_position_ + filter_state_.state.head<3>();
        if (predicted_rover.norm() > 1e6) {
            rover_pos_for_clk = predicted_rover;
        }
    }

    double rover_clk_bias = 0.0;
    {
        double sum_residual = 0.0; int count = 0;
        for (const auto& [sat, candidates] : rover_l1) {
            const Observation* r_obs = selectPreferredObservation(sat.system, candidates, true);
            if (r_obs == nullptr) continue;
            Vector3d sat_pos, sat_vel; double clk, clk_drift;
            double approx_travel = r_obs->pseudorange / constants::SPEED_OF_LIGHT;
            GNSSTime approx_tx = rover_obs.time - approx_travel;
            if (!nav.calculateSatelliteState(sat, approx_tx, sat_pos, sat_vel, clk, clk_drift)) continue;
            double geometric_range = (sat_pos - rover_pos_for_clk).norm();
            sum_residual += r_obs->pseudorange - geometric_range + constants::SPEED_OF_LIGHT * clk;
            count++;
        }
        if (count > 0) rover_clk_bias = sum_residual / count;
    }
    for (const auto& [sat, rover_l1_candidates] : rover_l1) {
        auto b_it = base_l1.find(sat);
        if (b_it == base_l1.end()) continue;

        const Observation* r_obs = nullptr;
        const Observation* b_obs = nullptr;
        if (!selectMatchedObservationPair(
                sat.system, rover_l1_candidates, b_it->second, true, r_obs, b_obs)) {
            continue;
        }

        double pr_travel = r_obs->pseudorange / constants::SPEED_OF_LIGHT;
        GNSSTime t_approx = rover_obs.time - pr_travel;
        Vector3d sat_pos, sat_vel; double clk, clk_drift;
        if (!nav.calculateSatelliteState(sat, t_approx, sat_pos, sat_vel, clk, clk_drift)) continue;
        GNSSTime t_refined = t_approx - clk;
        if (!nav.calculateSatelliteState(sat, t_refined, sat_pos, sat_vel, clk, clk_drift)) continue;
        // Use unrotated sat_pos; Sagnac is handled analytically by geodist_range
        Vector3d corrected_sat_pos = sat_pos;
        Vector3d base_sat_pos = sat_pos;
        {
            double base_pr = b_obs->pseudorange;
            double base_travel = base_pr / constants::SPEED_OF_LIGHT;
            GNSSTime base_tx = base_obs.time - base_travel;
            Vector3d bsp, bsv; double bclk, bclkd;
            if (nav.calculateSatelliteState(sat, base_tx, bsp, bsv, bclk, bclkd)) {
                GNSSTime base_refined = base_tx - bclk;
                if (nav.calculateSatelliteState(sat, base_refined, bsp, bsv, bclk, bclkd)) {
                    base_sat_pos = bsp;
                }
            }
        }
        SatelliteData sd; sd.satellite = sat; sd.sat_pos = corrected_sat_pos;
        sd.sat_pos_base = base_sat_pos; sd.has_ephemeris = true;
        sd.sat_vel = sat_vel; sd.sat_clock_drift = clk_drift; sd.has_sat_velocity = true;
        auto geom = nav.calculateGeometry(rover_pos_for_clk, corrected_sat_pos);
        sd.elevation = geom.elevation;
        auto base_geom = nav.calculateGeometry(base_position_, base_sat_pos);
        sd.base_elevation = base_geom.elevation;
        if (sd.elevation < rtk_config_.elevation_mask) continue;
        const Ephemeris* eph = nav.getEphemeris(sat, t_refined);
        sd.l1_signal = r_obs->signal;
        sd.l1_frequency_hz = signalFrequencyHz(sd.l1_signal, eph);
        sd.l1_wavelength = signalWavelengthMeters(sd.l1_signal, eph);
        if (sd.l1_wavelength <= 0.0) continue;
        sd.rover_l1_phase = r_obs->carrier_phase; sd.rover_l1_code = r_obs->pseudorange;
        sd.base_l1_phase = b_obs->carrier_phase; sd.base_l1_code = b_obs->pseudorange;
        sd.rover_l1_doppler = r_obs->doppler;
        sd.base_l1_doppler = b_obs->doppler;
        sd.rover_l1_snr = r_obs->snr;
        sd.base_l1_snr = b_obs->snr;
        sd.has_l1 = true; sd.l1_lli = r_obs->lli | b_obs->lli;
        sd.has_l1_doppler = r_obs->has_doppler && b_obs->has_doppler;
        auto r_l2 = rover_l2.find(sat); auto b_l2 = base_l2.find(sat);
        if (r_l2 != rover_l2.end() && b_l2 != base_l2.end()) {
            const Observation* r_l2_obs = nullptr;
            const Observation* b_l2_obs = nullptr;
            if (selectMatchedObservationPair(
                    sat.system, r_l2->second, b_l2->second, false, r_l2_obs, b_l2_obs)) {
                sd.l2_signal = r_l2_obs->signal;
                sd.l2_frequency_hz = signalFrequencyHz(sd.l2_signal, eph);
                sd.l2_wavelength = signalWavelengthMeters(sd.l2_signal, eph);
                sd.rover_l2_phase = r_l2_obs->carrier_phase; sd.rover_l2_code = r_l2_obs->pseudorange;
                sd.base_l2_phase = b_l2_obs->carrier_phase; sd.base_l2_code = b_l2_obs->pseudorange;
                sd.rover_l2_doppler = r_l2_obs->doppler;
                sd.base_l2_doppler = b_l2_obs->doppler;
                sd.rover_l2_snr = r_l2_obs->snr;
                sd.base_l2_snr = b_l2_obs->snr;
                sd.has_l2 = sd.l2_wavelength > 0.0;
                sd.l2_lli = r_l2_obs->lli | b_l2_obs->lli;
                sd.has_l2_doppler = r_l2_obs->has_doppler && b_l2_obs->has_doppler;
            }
        }
        // Phase 18 Step 3: L5 pairing (no-op until rtk_config_.enable_l5 is set).
        if (l5_enabled) {
            auto r_l5 = rover_l5.find(sat); auto b_l5 = base_l5.find(sat);
            if (r_l5 != rover_l5.end() && b_l5 != base_l5.end()) {
                const Observation* r_l5_obs = nullptr;
                const Observation* b_l5_obs = nullptr;
                // Both rover/base L5 candidate lists contain only L5-class signals,
                // so selectMatchedObservationPair with primary=false picks the highest-
                // priority L5 variant common to both (e.g. matched GPS_L5 / GAL_E5A).
                if (selectMatchedObservationPair(
                        sat.system, r_l5->second, b_l5->second, false, r_l5_obs, b_l5_obs)) {
                    sd.l5_signal = r_l5_obs->signal;
                    sd.l5_frequency_hz = signalFrequencyHz(sd.l5_signal, eph);
                    sd.l5_wavelength = signalWavelengthMeters(sd.l5_signal, eph);
                    if (sd.l5_wavelength > 0.0) {
                        sd.rover_l5_phase = r_l5_obs->carrier_phase;
                        sd.rover_l5_code = r_l5_obs->pseudorange;
                        sd.base_l5_phase = b_l5_obs->carrier_phase;
                        sd.base_l5_code = b_l5_obs->pseudorange;
                        sd.rover_l5_doppler = r_l5_obs->doppler;
                        sd.base_l5_doppler = b_l5_obs->doppler;
                        sd.rover_l5_snr = r_l5_obs->snr;
                        sd.base_l5_snr = b_l5_obs->snr;
                        sd.has_l5 = true;
                        sd.l5_lli = r_l5_obs->lli | b_l5_obs->lli;
                        sd.has_l5_doppler = r_l5_obs->has_doppler && b_l5_obs->has_doppler;
                    }
                }
            }
        }
        result[sat] = sd;
    }
    return result;
}

SatelliteId RTKProcessor::selectReferenceSatellite(const std::map<SatelliteId, SatelliteData>& sat_data) {
    SatelliteId best_dual, best_l1;
    double max_dual_el = -1.0;
    double max_l1_el = -1.0;
    for (const auto& [sat, sd] : sat_data) {
        if (sd.has_l1 && sd.has_l2 && sd.elevation > max_dual_el) {
            max_dual_el = sd.elevation;
            best_dual = sat;
        }
        if (sd.has_l1 && sd.elevation > max_l1_el) {
            max_l1_el = sd.elevation;
            best_l1 = sat;
        }
    }
    return max_dual_el >= 0.0 ? best_dual : best_l1;
}

// ============================================================
// Reference satellite change: no-op for SD parameterization
// ============================================================
void RTKProcessor::handleReferenceSatelliteChange(const SatelliteId& new_ref,
    const std::map<SatelliteId, SatelliteData>& sat_data) {
    (void)sat_data;
    current_ref_satellite_ = new_ref;
    has_ref_satellite_ = true;
}

bool RTKProcessor::selectSystemReferenceSatellite(
    const std::map<SatelliteId, SatelliteData>& sat_data,
    GNSSSystem system,
    int min_lock_count,
    SatelliteId& ref_sat) const {
    return rtk_selection::selectSystemReferenceSatellite(
        buildSelectionSnapshot(sat_data), system, min_lock_count, ref_sat);
}

std::vector<rtk_selection::SatelliteSelectionData> RTKProcessor::buildSelectionSnapshot(
    const std::map<SatelliteId, SatelliteData>& sat_data) const {
    std::vector<rtk_selection::SatelliteSelectionData> snapshot;
    snapshot.reserve(sat_data.size());
    for (const auto& [sat, sd] : sat_data) {
        rtk_selection::SatelliteSelectionData item;
        item.satellite = sat;
        item.has_l1 = sd.has_l1;
        item.has_l2 = sd.has_l2;
        item.has_l5 = sd.has_l5;  // Phase 18 Step 4
        item.l1_wavelength = sd.l1_wavelength;
        item.l2_wavelength = sd.l2_wavelength;
        item.l5_wavelength = sd.l5_wavelength;  // Phase 18 Step 4
        item.elevation = sd.elevation;
        auto n1_it = filter_state_.n1_indices.find(sat);
        item.n1_active = n1_it != filter_state_.n1_indices.end() &&
                         filter_state_.state(n1_it->second) != 0.0;
        auto n2_it = filter_state_.n2_indices.find(sat);
        item.n2_active = n2_it != filter_state_.n2_indices.end() &&
                         filter_state_.state(n2_it->second) != 0.0;
        auto n5_it = filter_state_.n5_indices.find(sat);
        item.n5_active = n5_it != filter_state_.n5_indices.end() &&
                         filter_state_.state(n5_it->second) != 0.0;
        auto l1_it = lock_count_l1_.find(sat);
        item.lock_count_l1 = l1_it != lock_count_l1_.end() ? l1_it->second : 0;
        auto l2_it = lock_count_l2_.find(sat);
        item.lock_count_l2 = l2_it != lock_count_l2_.end() ? l2_it->second : 0;
        auto l5_it = lock_count_l5_.find(sat);
        item.lock_count_l5 = l5_it != lock_count_l5_.end() ? l5_it->second : 0;
        snapshot.push_back(item);
    }

    // WP8: hard NLOS exclusion. Both buildMeasurementBlocks() (float KF)
    // and buildDoubleDifferencePairs() (AR/LAMBDA candidate set) call this
    // one shared function to get their satellite candidate list, so
    // filtering it here applies identically to both consumers. No-op
    // (bit-identical) unless nlos_weight_mode == EXCLUDE and a weight
    // table is loaded, exactly mirroring the WP7 sigma-inflation hook's own
    // absent-flag guard.
    if (rtk_config_.nlos_weight_mode == nlos_weights::NlosWeightMode::EXCLUDE &&
        nlos_weight_table_ && !nlos_weight_table_->empty()) {
        std::set<SatelliteId> excluded;
        for (const auto& item : snapshot) {
            const double los_prob = nlos_weights::lookupLosProb(
                *nlos_weight_table_, current_epoch_time_.tow, item.satellite.toString(),
                rtk_config_.nlos_tow_tolerance_s);
            if (nlos_weights::nlosShouldExclude(
                    los_prob, rtk_config_.nlos_weight_mode, rtk_config_.nlos_exclude_threshold)) {
                excluded.insert(item.satellite);
            }
        }
        const bool guard_allows = nlos_weights::nlosExclusionGuardAllows(
            static_cast<int>(snapshot.size()), static_cast<int>(excluded.size()),
            rtk_config_.nlos_min_sats);
        if (guard_allows) {
            // Second guard: never exclude a system's *last* remaining
            // reference-satellite candidate -- that would zero out the
            // whole system's DD set rather than just shrinking it, even if
            // the epoch-wide min-sats floor above was satisfied.
            for (GNSSSystem system : kRTKSupportedSystems) {
                SatelliteId full_ref;
                if (!rtk_selection::selectSystemReferenceSatellite(snapshot, system, 0, full_ref)) {
                    continue;  // no reference candidate at all pre-exclusion; nothing to protect
                }
                if (excluded.count(full_ref) == 0) continue;
                std::vector<rtk_selection::SatelliteSelectionData> trial;
                trial.reserve(snapshot.size());
                for (const auto& item : snapshot) {
                    if (excluded.count(item.satellite) == 0) trial.push_back(item);
                }
                SatelliteId trial_ref;
                if (!rtk_selection::selectSystemReferenceSatellite(trial, system, 0, trial_ref)) {
                    excluded.erase(full_ref);
                }
            }
            if (!excluded.empty()) {
                std::vector<rtk_selection::SatelliteSelectionData> filtered;
                filtered.reserve(snapshot.size());
                for (auto& item : snapshot) {
                    if (excluded.count(item.satellite) == 0) filtered.push_back(std::move(item));
                }
                snapshot = std::move(filtered);
            }
        }
    }

    return snapshot;
}

// ============================================================
// Phase 2a: CMC-aware DD reference-satellite selection (opt-in)
// ============================================================
void RTKProcessor::updateCmcAwareReferenceSelection(
    const std::map<SatelliteId, SatelliteData>& sat_data,
    const std::set<SatelliteId>& gf_slips,
    const std::set<SatelliteId>& gf_slips_l1l5,
    const std::set<SatelliteId>& code_slips_l1,
    const std::set<SatelliteId>& doppler_slips_l1) {
    if (!cmc_suspect_tracker_) {
        cmc_suspect_tracker_ =
            std::make_unique<rtk_cmc_reference::CmcSuspectTracker>(rtk_config_.cmc_ref_level_m);
    }

    // 1) Per-satellite CMC suspect classification for this epoch, driven by
    // the SD L1 code-minus-phase deviation from each satellite's own
    // running baseline. Reuses this epoch's already-computed slip sets as
    // the "arc restarted" signal (same reasoning as FGOProcessor's
    // rover_arc_restarted: a fresh ambiguity invalidates the old baseline).
    std::set<SatelliteId> seen;
    std::set<SatelliteId> suspects;
    for (const auto& [sat, sd] : sat_data) {
        if (!sd.has_l1 || sd.l1_wavelength <= 0.0) continue;
        seen.insert(sat);
        const double cmc_m = rtk_slip_detection::singleDifferenceCodeMinusPhaseM(
            sd.rover_l1_code, sd.base_l1_code, sd.rover_l1_phase, sd.base_l1_phase, sd.l1_wavelength);
        const bool arc_restarted = gf_slips.count(sat) > 0 || gf_slips_l1l5.count(sat) > 0 ||
                                   code_slips_l1.count(sat) > 0 || doppler_slips_l1.count(sat) > 0 ||
                                   (sd.l1_lli & 0x01) != 0;
        if (cmc_suspect_tracker_->classify(sat, cmc_m, arc_restarted)) {
            suspects.insert(sat);
            ++cmc_ref_suspect_epoch_count_;
        }
    }
    cmc_suspect_tracker_->pruneMissing(seen);

    // 2) Per-system hysteresis reference selection, fed by this epoch's
    // suspect classification above.
    cmc_aware_ref_by_system_.clear();
    const auto snapshot = buildSelectionSnapshot(sat_data);
    const double return_min_elev_rad = rtk_config_.cmc_ref_return_min_elev_deg * M_PI / 180.0;
    const double switch_away_max_elev_drop_rad =
        rtk_config_.cmc_ref_switch_max_elev_drop_deg * M_PI / 180.0;
    const double switch_away_min_elev_rad = rtk_config_.cmc_ref_switch_min_elev_deg * M_PI / 180.0;

    for (GNSSSystem system : kRTKSupportedSystems) {
        if (!isEnabledRTKSystem(rtk_config_, system)) continue;
        SatelliteId natural_ref;
        if (!rtk_selection::selectSystemReferenceSatellite(snapshot, system, 0, natural_ref)) {
            continue;  // no candidate at all this epoch -- nothing to select or track
        }

        std::vector<rtk_cmc_reference::ReferenceHysteresis::Candidate> candidates;
        double natural_ref_elevation_rad = 0.0;
        for (const auto& item : snapshot) {
            if (item.satellite.system != system || !item.has_l1 || !item.n1_active) continue;
            rtk_cmc_reference::ReferenceHysteresis::Candidate candidate;
            candidate.satellite = item.satellite;
            candidate.elevation_rad = item.elevation;
            candidate.dual_frequency = item.has_l2 && item.n2_active;
            candidate.suspect = suspects.count(item.satellite) > 0;
            if (item.satellite == natural_ref) natural_ref_elevation_rad = item.elevation;
            candidates.push_back(candidate);
        }

        auto& hysteresis = cmc_ref_hysteresis_by_system_[system];
        SatelliteId chosen_ref;
        bool switched = false;
        if (hysteresis.update(candidates, natural_ref, natural_ref_elevation_rad,
                              rtk_config_.cmc_ref_switch_epochs, return_min_elev_rad, chosen_ref,
                              switched, switch_away_max_elev_drop_rad,
                              switch_away_min_elev_rad)) {
            cmc_aware_ref_by_system_[system] = chosen_ref;
            if (switched) ++cmc_ref_switch_count_;
        }
    }
}

std::vector<RTKProcessor::DDPair> RTKProcessor::buildDoubleDifferencePairs(
    const std::map<SatelliteId, SatelliteData>& sat_data,
    int min_lock_count) const {
    std::vector<DDPair> dd_pairs;
    const auto snapshot = buildSelectionSnapshot(sat_data);

    for (GNSSSystem system : kRTKSupportedSystems) {
        if (!isEnabledRTKSystem(rtk_config_, system)) continue;
        const SatelliteId* forced_ref = nullptr;
        if (rtk_config_.cmc_aware_reference_selection) {
            const auto forced_it = cmc_aware_ref_by_system_.find(system);
            if (forced_it != cmc_aware_ref_by_system_.end()) forced_ref = &forced_it->second;
        }
        const auto system_pairs = rtk_selection::buildDoubleDifferencePairsForSystem(
            snapshot,
            system,
            min_lock_count,
            requiresMatchedCarrierWavelength(rtk_config_, system),
            forced_ref);
        for (const auto& pair : system_pairs) {
            const auto& indices = (pair.freq == 0) ? filter_state_.n1_indices :
                                  (pair.freq == 1) ? filter_state_.n2_indices :
                                                     filter_state_.n5_indices;  // Phase 18 Step 4
            auto ref_idx = indices.find(pair.ref_sat);
            auto sat_idx = indices.find(pair.sat);
            if (ref_idx == indices.end() || sat_idx == indices.end()) {
                continue;
            }
            dd_pairs.push_back({pair.ref_sat, ref_idx->second, sat_idx->second, pair.sat, pair.freq});
        }
    }

    return dd_pairs;
}

// ============================================================
// Filter initialization
// ============================================================
bool RTKProcessor::initializeFilter(const ObservationData& rover_obs,
    const ObservationData& base_obs, const NavigationData& nav) {
    (void)base_obs;
    const int state_size = rtk_config_.enable_velocity_states ? NX : LEGACY_NX;
    filter_state_.state = VectorXd::Zero(state_size);
    filter_state_.covariance = MatrixXd::Zero(state_size, state_size);
    filter_state_.iono_indices.clear();
    filter_state_.n1_indices.clear();
    filter_state_.n2_indices.clear();
    filter_state_.n5_indices.clear();  // Phase 18 Step 2: clear L5 index map on filter init
    filter_state_.next_state_idx = REAL_STATES + IONO_STATES;

    auto spp = spp_processor_.processEpoch(rover_obs, nav);
    Vector3d rover_pos;
    if (rtk_config_.prefer_rover_position_seed && rover_obs.receiver_position.norm() > 1e6) {
        rover_pos = rover_obs.receiver_position;
    } else if (spp.isValid()) {
        rover_pos = spp.position_ecef;
    } else if (rover_obs.receiver_position.norm() > 1e6) {
        rover_pos = rover_obs.receiver_position;
    } else {
        rover_pos = base_position_;
    }
    filter_state_.state.head<3>() = rover_pos - base_position_;

    for (int i = 0; i < BASE_STATES; ++i)
        filter_state_.covariance(i, i) = 900.0;

    filter_initialized_ = true;
    return true;
}

void RTKProcessor::updateGlonassHardwareBias(double dt) {
    if (!usesGlonassAutocal(rtk_config_)) {
        return;
    }
    if (!std::isfinite(dt) || dt <= 0.0) {
        dt = 1.0;
    }

    const double initial_values[GLO_HWBIAS_STATES] = {
        rtk_config_.glonass_icb_l1_m_per_mhz,
        rtk_config_.glonass_icb_l2_m_per_mhz,
    };
    for (int freq = 0; freq < GLO_HWBIAS_STATES; ++freq) {
        const int idx = IL(freq);
        if (filter_state_.state(idx) == 0.0 || filter_state_.covariance(idx, idx) <= 0.0) {
            filter_state_.state(idx) = initial_values[freq];
            for (int j = 0; j < filter_state_.state.size(); ++j) {
                filter_state_.covariance(idx, j) = 0.0;
                filter_state_.covariance(j, idx) = 0.0;
            }
            filter_state_.covariance(idx, idx) = kGlonassHWBiasInitialVariance;
        } else {
            filter_state_.covariance(idx, idx) += kGlonassHWBiasProcessNoise * dt;
        }
    }
}

// ============================================================
// Update SD biases (RTKLIB udbias)
// ============================================================
void RTKProcessor::updateTdcpDiagnostics(
    const std::map<SatelliteId, SatelliteData>& sat_data, double dt_s) {
    if (!rtk_config_.enable_tdcp_diagnostics) return;

    const rtk_tdcp_diagnostics::Config config{
        rtk_config_.tdcp_diagnostics_max_gap_s};
    double residual_sum_squares = 0.0;
    double residual_max_abs = 0.0;

    auto process_frequency = [&](auto& history, auto get_measurement) {
        std::set<SatelliteId> seen;
        for (const auto& [sat, data] : sat_data) {
            double phase_m = 0.0;
            double range_rate_mps = 0.0;
            bool loss_of_lock = false;
            if (!get_measurement(data, phase_m, range_rate_mps, loss_of_lock)) continue;
            seen.insert(sat);
            ++debug_telemetry_.tdcp_candidate_count;
            const auto previous = history.find(sat);
            if (previous == history.end()) {
                ++debug_telemetry_.tdcp_rejected_missing_previous;
            } else {
                const auto result = rtk_tdcp_diagnostics::evaluate(
                    previous->second.phase_m, phase_m,
                    previous->second.range_rate_mps, range_rate_mps,
                    dt_s, loss_of_lock, config);
                switch (result.status) {
                    case rtk_tdcp_diagnostics::Status::VALID:
                        ++debug_telemetry_.tdcp_residual_count;
                        residual_sum_squares += result.residual_m * result.residual_m;
                        residual_max_abs = std::max(residual_max_abs, std::abs(result.residual_m));
                        break;
                    case rtk_tdcp_diagnostics::Status::INVALID_GAP:
                        ++debug_telemetry_.tdcp_rejected_gap;
                        break;
                    case rtk_tdcp_diagnostics::Status::LOSS_OF_LOCK:
                        ++debug_telemetry_.tdcp_rejected_loss_of_lock;
                        break;
                    case rtk_tdcp_diagnostics::Status::INVALID_INPUT:
                        ++debug_telemetry_.tdcp_rejected_invalid;
                        break;
                }
            }
            history[sat] = TdcpHistory{phase_m, range_rate_mps};
        }
        for (auto it = history.begin(); it != history.end();) {
            it = seen.contains(it->first) ? std::next(it) : history.erase(it);
        }
    };

    process_frequency(tdcp_history_l1_, [](const SatelliteData& data, double& phase_m,
                                           double& range_rate_mps, bool& loss_of_lock) {
        if (!data.has_l1 || !data.has_l1_doppler || data.l1_wavelength <= 0.0) return false;
        phase_m = (data.rover_l1_phase - data.base_l1_phase) * data.l1_wavelength;
        range_rate_mps = rtk_slip_detection::singleDifferenceRangeRateMps(
            data.rover_l1_doppler, data.base_l1_doppler, data.l1_wavelength);
        loss_of_lock = data.l1_lli != 0;
        return true;
    });
    process_frequency(tdcp_history_l2_, [](const SatelliteData& data, double& phase_m,
                                           double& range_rate_mps, bool& loss_of_lock) {
        if (!data.has_l2 || !data.has_l2_doppler || data.l2_wavelength <= 0.0) return false;
        phase_m = (data.rover_l2_phase - data.base_l2_phase) * data.l2_wavelength;
        range_rate_mps = rtk_slip_detection::singleDifferenceRangeRateMps(
            data.rover_l2_doppler, data.base_l2_doppler, data.l2_wavelength);
        loss_of_lock = data.l2_lli != 0;
        return true;
    });
    if (rtk_config_.enable_l5) {
        process_frequency(tdcp_history_l5_, [](const SatelliteData& data, double& phase_m,
                                               double& range_rate_mps, bool& loss_of_lock) {
            if (!data.has_l5 || !data.has_l5_doppler || data.l5_wavelength <= 0.0) return false;
            phase_m = (data.rover_l5_phase - data.base_l5_phase) * data.l5_wavelength;
            range_rate_mps = rtk_slip_detection::singleDifferenceRangeRateMps(
                data.rover_l5_doppler, data.base_l5_doppler, data.l5_wavelength);
            loss_of_lock = data.l5_lli != 0;
            return true;
        });
    }

    if (debug_telemetry_.tdcp_residual_count > 0) {
        debug_telemetry_.tdcp_residual_rms_m = std::sqrt(
            residual_sum_squares / debug_telemetry_.tdcp_residual_count);
        debug_telemetry_.tdcp_residual_max_abs_m = residual_max_abs;
    }
}

void RTKProcessor::updateBias(const std::map<SatelliteId, SatelliteData>& sat_data, double dt_s) {
    updateTdcpDiagnostics(sat_data, dt_s);
    std::vector<SatelliteId> sats_to_remove;
    for (const auto& [sat, idx] : filter_state_.n1_indices) {
        if (sat_data.find(sat) == sat_data.end()) sats_to_remove.push_back(sat);
    }
    for (const auto& sat : sats_to_remove) {
        removeSatelliteFromState(sat);
        lock_count_l1_.erase(sat);
        lock_count_l2_.erase(sat);
        lock_count_l5_.erase(sat);  // Phase 18 Step 2: erase L5 lock count when sat dropped
        gf_l1l2_history_.erase(sat);
        gf_l1l5_history_.erase(sat);  // Phase 18 Step 5
        doppler_phase_history_l1_m_.erase(sat);
        doppler_phase_history_l2_m_.erase(sat);
        doppler_phase_history_l5_m_.erase(sat);  // Phase 18 Step 5
        code_phase_history_l1_m_.erase(sat);
        code_phase_history_l2_m_.erase(sat);
        code_phase_history_l5_m_.erase(sat);  // Phase 18 Step 5
    }

    const bool dynamic_slip_floor_candidate =
        isDynamicPositionMode(rtk_config_) && rtk_config_.use_dynamic_slip_threshold_floor;
    const int adaptive_nonfix_count =
        std::max(1, rtk_config_.adaptive_dynamic_slip_nonfix_count);
    if (dynamic_slip_floor_candidate &&
        rtk_config_.enable_adaptive_dynamic_slip_thresholds &&
        consecutive_nonfix_count_ >= adaptive_nonfix_count) {
        adaptive_dynamic_slip_hold_count_ =
            std::max(adaptive_dynamic_slip_hold_count_,
                     std::max(0, rtk_config_.adaptive_dynamic_slip_hold_epochs));
    }
    const bool adaptive_dynamic_slip_active =
        dynamic_slip_floor_candidate &&
        rtk_config_.enable_adaptive_dynamic_slip_thresholds &&
        (consecutive_nonfix_count_ >= adaptive_nonfix_count ||
         adaptive_dynamic_slip_hold_count_ > 0);
    const bool apply_dynamic_slip_floor =
        dynamic_slip_floor_candidate && !adaptive_dynamic_slip_active;
    debug_telemetry_.adaptive_dynamic_slip_active = adaptive_dynamic_slip_active;
    debug_telemetry_.consecutive_nonfix_before_bias_update = consecutive_nonfix_count_;
    debug_telemetry_.adaptive_dynamic_slip_hold_remaining =
        adaptive_dynamic_slip_hold_count_;
    if (adaptive_dynamic_slip_active && adaptive_dynamic_slip_hold_count_ > 0) {
        adaptive_dynamic_slip_hold_count_--;
    }

    std::set<SatelliteId> gf_slips;
    std::set<SatelliteId> gf_slips_l1l5;  // Phase 18 Step 5
    if (rtk_config_.enable_cycle_slip_detection) {
        const double gf_slip_threshold =
            apply_dynamic_slip_floor
                ? std::max(rtk_config_.cycle_slip_threshold, 0.12)
                : rtk_config_.cycle_slip_threshold;
        for (const auto& [sat, sd] : sat_data) {
            if (!sd.has_l1 || !sd.has_l2 || sd.l1_wavelength <= 0.0 || sd.l2_wavelength <= 0.0) continue;
            double gf = (sd.rover_l1_phase - sd.base_l1_phase) * sd.l1_wavelength -
                        (sd.rover_l2_phase - sd.base_l2_phase) * sd.l2_wavelength;
            auto prev_it = gf_l1l2_history_.find(sat);
            if (prev_it != gf_l1l2_history_.end() &&
                std::abs(gf - prev_it->second) > gf_slip_threshold) {
                gf_slips.insert(sat);
            }
            gf_l1l2_history_[sat] = gf;
        }
        if (rtk_config_.enable_l5) {
            // Phase 18 Step 5: parallel GF L1-L5 detector. Same threshold as L1-L2 since both
            // are dual-carrier ionosphere-free residuals — slip on either L1 or L5 carrier
            // appears as a multi-cycle jump in the GF combination.
            for (const auto& [sat, sd] : sat_data) {
                if (!sd.has_l1 || !sd.has_l5 || sd.l1_wavelength <= 0.0 || sd.l5_wavelength <= 0.0) continue;
                double gf15 = (sd.rover_l1_phase - sd.base_l1_phase) * sd.l1_wavelength -
                              (sd.rover_l5_phase - sd.base_l5_phase) * sd.l5_wavelength;
                auto prev_it = gf_l1l5_history_.find(sat);
                if (prev_it != gf_l1l5_history_.end() &&
                    std::abs(gf15 - prev_it->second) > gf_slip_threshold) {
                    gf_slips_l1l5.insert(sat);
                }
                gf_l1l5_history_[sat] = gf15;
            }
        }
    }
    debug_telemetry_.gf_slip_count = static_cast<int>(gf_slips.size());
    debug_telemetry_.gf_slip_l1l5_count = static_cast<int>(gf_slips_l1l5.size());

    std::set<SatelliteId> doppler_slips_l1;
    std::set<SatelliteId> doppler_slips_l2;
    std::set<SatelliteId> doppler_slips_l5;  // Phase 18 Step 5
    if (rtk_config_.enable_doppler_slip_detection &&
        std::isfinite(dt_s) &&
        dt_s > 0.0 &&
        dt_s <= 5.0) {
        const double doppler_slip_threshold =
            apply_dynamic_slip_floor
                ? std::max(rtk_config_.doppler_slip_threshold, 0.20)
                : std::max(rtk_config_.doppler_slip_threshold, 0.10);
        for (const auto& [sat, sd] : sat_data) {
            if (sd.has_l1 && sd.has_l1_doppler && sd.l1_wavelength > 0.0) {
                const double sd_phase_m =
                    (sd.rover_l1_phase - sd.base_l1_phase) * sd.l1_wavelength;
                const double sd_range_rate_mps =
                    rtk_slip_detection::singleDifferenceRangeRateMps(
                        sd.rover_l1_doppler, sd.base_l1_doppler, sd.l1_wavelength);
                auto previous = doppler_phase_history_l1_m_.find(sat);
                if (previous != doppler_phase_history_l1_m_.end() &&
                    rtk_slip_detection::detectDopplerSlip(
                        previous->second,
                        sd_phase_m,
                        sd_range_rate_mps,
                        dt_s,
                        doppler_slip_threshold)) {
                    doppler_slips_l1.insert(sat);
                }
                doppler_phase_history_l1_m_[sat] = sd_phase_m;
            }
            if (sd.has_l2 && sd.has_l2_doppler && sd.l2_wavelength > 0.0) {
                const double sd_phase_m =
                    (sd.rover_l2_phase - sd.base_l2_phase) * sd.l2_wavelength;
                const double sd_range_rate_mps =
                    rtk_slip_detection::singleDifferenceRangeRateMps(
                        sd.rover_l2_doppler, sd.base_l2_doppler, sd.l2_wavelength);
                auto previous = doppler_phase_history_l2_m_.find(sat);
                if (previous != doppler_phase_history_l2_m_.end() &&
                    rtk_slip_detection::detectDopplerSlip(
                        previous->second,
                        sd_phase_m,
                        sd_range_rate_mps,
                        dt_s,
                        doppler_slip_threshold)) {
                    doppler_slips_l2.insert(sat);
                }
                doppler_phase_history_l2_m_[sat] = sd_phase_m;
            }
            // Phase 18 Step 5: doppler-based slip detection on L5.
            if (rtk_config_.enable_l5 &&
                sd.has_l5 && sd.has_l5_doppler && sd.l5_wavelength > 0.0) {
                const double sd_phase_m =
                    (sd.rover_l5_phase - sd.base_l5_phase) * sd.l5_wavelength;
                const double sd_range_rate_mps =
                    rtk_slip_detection::singleDifferenceRangeRateMps(
                        sd.rover_l5_doppler, sd.base_l5_doppler, sd.l5_wavelength);
                auto previous = doppler_phase_history_l5_m_.find(sat);
                if (previous != doppler_phase_history_l5_m_.end() &&
                    rtk_slip_detection::detectDopplerSlip(
                        previous->second,
                        sd_phase_m,
                        sd_range_rate_mps,
                        dt_s,
                        doppler_slip_threshold)) {
                    doppler_slips_l5.insert(sat);
                }
                doppler_phase_history_l5_m_[sat] = sd_phase_m;
            }
        }
    }
    debug_telemetry_.doppler_slip_l1_count = static_cast<int>(doppler_slips_l1.size());
    debug_telemetry_.doppler_slip_l2_count = static_cast<int>(doppler_slips_l2.size());
    debug_telemetry_.doppler_slip_l5_count = static_cast<int>(doppler_slips_l5.size());

    std::set<SatelliteId> code_slips_l1;
    std::set<SatelliteId> code_slips_l2;
    std::set<SatelliteId> code_slips_l5;  // Phase 18 Step 5
    if (rtk_config_.enable_code_slip_detection) {
        const double code_slip_threshold =
            apply_dynamic_slip_floor
                ? std::max(rtk_config_.code_slip_threshold, 5.0)
                : std::max(rtk_config_.code_slip_threshold, 3.0);
        for (const auto& [sat, sd] : sat_data) {
            if (sd.has_l1 && sd.l1_wavelength > 0.0) {
                const double code_minus_phase_m =
                    rtk_slip_detection::singleDifferenceCodeMinusPhaseM(
                        sd.rover_l1_code,
                        sd.base_l1_code,
                        sd.rover_l1_phase,
                        sd.base_l1_phase,
                        sd.l1_wavelength);
                auto previous = code_phase_history_l1_m_.find(sat);
                if (previous != code_phase_history_l1_m_.end() &&
                    rtk_slip_detection::detectCodeSlip(
                        previous->second,
                        code_minus_phase_m,
                        code_slip_threshold)) {
                    code_slips_l1.insert(sat);
                }
                code_phase_history_l1_m_[sat] = code_minus_phase_m;
            }
            if (sd.has_l2 && sd.l2_wavelength > 0.0) {
                const double code_minus_phase_m =
                    rtk_slip_detection::singleDifferenceCodeMinusPhaseM(
                        sd.rover_l2_code,
                        sd.base_l2_code,
                        sd.rover_l2_phase,
                        sd.base_l2_phase,
                        sd.l2_wavelength);
                auto previous = code_phase_history_l2_m_.find(sat);
                if (previous != code_phase_history_l2_m_.end() &&
                    rtk_slip_detection::detectCodeSlip(
                        previous->second,
                        code_minus_phase_m,
                        code_slip_threshold)) {
                    code_slips_l2.insert(sat);
                }
                code_phase_history_l2_m_[sat] = code_minus_phase_m;
            }
            // Phase 18 Step 5: code-minus-phase slip detection on L5.
            if (rtk_config_.enable_l5 && sd.has_l5 && sd.l5_wavelength > 0.0) {
                const double code_minus_phase_m =
                    rtk_slip_detection::singleDifferenceCodeMinusPhaseM(
                        sd.rover_l5_code,
                        sd.base_l5_code,
                        sd.rover_l5_phase,
                        sd.base_l5_phase,
                        sd.l5_wavelength);
                auto previous = code_phase_history_l5_m_.find(sat);
                if (previous != code_phase_history_l5_m_.end() &&
                    rtk_slip_detection::detectCodeSlip(
                        previous->second,
                        code_minus_phase_m,
                        code_slip_threshold)) {
                    code_slips_l5.insert(sat);
                }
                code_phase_history_l5_m_[sat] = code_minus_phase_m;
            }
        }
    }
    debug_telemetry_.code_slip_l1_count = static_cast<int>(code_slips_l1.size());
    debug_telemetry_.code_slip_l2_count = static_cast<int>(code_slips_l2.size());
    debug_telemetry_.code_slip_l5_count = static_cast<int>(code_slips_l5.size());

    // Phase 2a: CMC-aware DD reference-satellite selection (opt-in). Reuses
    // this epoch's already-computed slip-detection sets (gf/doppler/code)
    // as the CMC baseline's arc-restart signal. No-op (cmc_aware_ref_by_
    // system_ left empty) unless the knob is on.
    if (rtk_config_.cmc_aware_reference_selection) {
        updateCmcAwareReferenceSelection(sat_data, gf_slips, gf_slips_l1l5, code_slips_l1,
                                         doppler_slips_l1);
    } else if (!cmc_aware_ref_by_system_.empty()) {
        cmc_aware_ref_by_system_.clear();
    }

    // Phase 18 Step 4: extend freq loop from {L1, L2} to {L1, L2, L5} when enable_l5.
    // Per-frequency accessors abstract over the SatelliteData layout differences.
    auto has_freq_signal = [](const SatelliteData& sd, int freq) -> bool {
        return freq == 0 ? sd.has_l1 : (freq == 1 ? sd.has_l2 : sd.has_l5);
    };
    auto freq_lli = [](const SatelliteData& sd, int freq) -> int {
        return freq == 0 ? sd.l1_lli : (freq == 1 ? sd.l2_lli : sd.l5_lli);
    };
    auto freq_wavelength = [](const SatelliteData& sd, int freq) -> double {
        return freq == 0 ? sd.l1_wavelength : (freq == 1 ? sd.l2_wavelength : sd.l5_wavelength);
    };
    auto freq_phase_diff = [](const SatelliteData& sd, int freq) -> double {
        if (freq == 0) return sd.rover_l1_phase - sd.base_l1_phase;
        if (freq == 1) return sd.rover_l2_phase - sd.base_l2_phase;
        return sd.rover_l5_phase - sd.base_l5_phase;
    };
    auto freq_code_diff = [](const SatelliteData& sd, int freq) -> double {
        if (freq == 0) return sd.rover_l1_code - sd.base_l1_code;
        if (freq == 1) return sd.rover_l2_code - sd.base_l2_code;
        return sd.rover_l5_code - sd.base_l5_code;
    };
    const int max_freq = rtk_config_.enable_l5 ? 3 : 2;
    for (int freq = 0; freq < max_freq; ++freq) {
        auto& indices = (freq == 0) ? filter_state_.n1_indices :
                        (freq == 1) ? filter_state_.n2_indices : filter_state_.n5_indices;
        auto& lock_counts = (freq == 0) ? lock_count_l1_ :
                            (freq == 1) ? lock_count_l2_ : lock_count_l5_;
        int lli_slip_count = 0;
        int ambiguity_reset_count = 0;

        // Detect cycle slips and reset
        for (const auto& [sat, sd] : sat_data) {
            if (!has_freq_signal(sd, freq)) continue;
            int lli = freq_lli(sd, freq);
            const bool lli_slip = (lli & 0x01) != 0;
            if (lli_slip) {
                lli_slip_count++;
            }
            // Phase 18 Step 5: L5 now participates in GF (L1-L5) / doppler-L5 / code-L5 slip checks.
            bool slip = lli_slip;
            if (freq == 0) {
                slip = slip ||
                       gf_slips.find(sat) != gf_slips.end() ||
                       gf_slips_l1l5.find(sat) != gf_slips_l1l5.end() ||
                       code_slips_l1.find(sat) != code_slips_l1.end() ||
                       doppler_slips_l1.find(sat) != doppler_slips_l1.end();
            } else if (freq == 1) {
                slip = slip ||
                       gf_slips.find(sat) != gf_slips.end() ||
                       code_slips_l2.find(sat) != code_slips_l2.end() ||
                       doppler_slips_l2.find(sat) != doppler_slips_l2.end();
            } else {  // freq == 2 (L5)
                slip = slip ||
                       gf_slips_l1l5.find(sat) != gf_slips_l1l5.end() ||
                       code_slips_l5.find(sat) != code_slips_l5.end() ||
                       doppler_slips_l5.find(sat) != doppler_slips_l5.end();
            }
            auto idx_it = indices.find(sat);
            if (idx_it != indices.end() && slip) {
                ambiguity_reset_count++;
                // navi.776 A2: a slip invalidates the learned phase variance
                // for this satellite/frequency; code memory survives.
                if (rtk_config_.enable_adaptive_measurement_noise) {
                    adaptive_noise_tracker_.resetKey(
                        freq * MAXSAT + satelliteSlot(sat),
                        rtk_measurement::MeasurementKind::PHASE);
                }
                int idx = idx_it->second;
                filter_state_.state(idx) = 0.0;
                filter_state_.covariance(idx, idx) = 0.0;
                int n = filter_state_.state.size();
                for (int j = 0; j < n; ++j) {
                    if (j != idx) { filter_state_.covariance(j, idx) = 0; filter_state_.covariance(idx, j) = 0; }
                }
                lock_counts[sat] = -rtk_config_.min_lock_count;
                if (usesEstimatedIono(rtk_config_)) {
                    auto iono_it = filter_state_.iono_indices.find(sat);
                    if (iono_it != filter_state_.iono_indices.end()) {
                        int iono_idx = iono_it->second;
                        filter_state_.state(iono_idx) = 0.0;
                        filter_state_.covariance(iono_idx, iono_idx) = 0.0;
                        for (int j = 0; j < n; ++j) {
                            if (j != iono_idx) {
                                filter_state_.covariance(j, iono_idx) = 0.0;
                                filter_state_.covariance(iono_idx, j) = 0.0;
                            }
                        }
                    }
                }
            }
        }
        if (freq == 0) {
            debug_telemetry_.lli_slip_l1_count = lli_slip_count;
            debug_telemetry_.ambiguity_reset_l1_count = ambiguity_reset_count;
        } else if (freq == 1) {
            debug_telemetry_.lli_slip_l2_count = lli_slip_count;
            debug_telemetry_.ambiguity_reset_l2_count = ambiguity_reset_count;
        } else {  // freq == 2 (Phase 18 Step 5)
            debug_telemetry_.lli_slip_l5_count = lli_slip_count;
            debug_telemetry_.ambiguity_reset_l5_count = ambiguity_reset_count;
        }

        for (GNSSSystem system : kRTKSupportedSystems) {
            if (!isEnabledRTKSystem(rtk_config_, system)) continue;
            std::map<SatelliteId, double> bias;
            double offset = 0.0;
            int offset_count = 0;

            for (const auto& [sat, sd] : sat_data) {
                if (sat.system != system) continue;
                if (!has_freq_signal(sd, freq)) continue;
                const double wavelength = freq_wavelength(sd, freq);
                if (wavelength <= 0.0) continue;
                const double cp = freq_phase_diff(sd, freq);
                const double pr = freq_code_diff(sd, freq);
                double b = cp - pr / wavelength;
                bias[sat] = b;
                auto idx_it = indices.find(sat);
                if (idx_it != indices.end() && filter_state_.state(idx_it->second) != 0.0) {
                    offset += b - filter_state_.state(idx_it->second);
                    offset_count++;
                }
            }

            if (offset_count > 0) {
                double avg_offset = offset / offset_count;
                for (auto& [sat, idx] : indices) {
                    if (sat.system == system && filter_state_.state(idx) != 0.0) {
                        filter_state_.state(idx) += avg_offset;
                    }
                }
            }

            for (const auto& [sat, b] : bias) {
                auto idx_it = indices.find(sat);
                if (idx_it != indices.end() && filter_state_.state(idx_it->second) != 0.0) continue;
                if (freq == 0) getOrCreateN1Index(sat, b);
                else if (freq == 1) getOrCreateN2Index(sat, b);
                else getOrCreateN5Index(sat, b);
                lock_counts[sat] = 0;
            }
        }

        // Add process noise
        if (rtk_config_.process_noise_ambiguity > 0) {
            for (const auto& [sat, idx] : indices) {
                if (filter_state_.state(idx) != 0.0) {
                    filter_state_.covariance(idx, idx) += rtk_config_.process_noise_ambiguity;
                }
            }
        }
    }

    if (usesEstimatedIono(rtk_config_)) {
        for (const auto& [sat, sd] : sat_data) {
            if (!sd.has_l1 || !sd.has_l2) continue;
            if (sat.system == GNSSSystem::GLONASS) continue;
            if (sd.l1_frequency_hz <= 0.0 || sd.l2_frequency_hz <= 0.0) continue;
            const double gamma =
                ionoFrequencyScale(1, sd.l1_frequency_hz, sd.l2_frequency_hz);
            const double denom = gamma - 1.0;
            if (!std::isfinite(denom) || std::abs(denom) < 1e-6) continue;
            const double sd_p1 = sd.rover_l1_code - sd.base_l1_code;
            const double sd_p2 = sd.rover_l2_code - sd.base_l2_code;
            const double iono_l1_m = (sd_p2 - sd_p1) / denom;
            const int idx = getOrCreateIonoIndex(sat, iono_l1_m);
            if (filter_state_.covariance(idx, idx) > 0.0 &&
                rtk_config_.process_noise_iono > 0.0) {
                filter_state_.covariance(idx, idx) += rtk_config_.process_noise_iono;
            }
        }
    }
}

// ============================================================
// Position update (RTKLIB udpos)
// ============================================================
void RTKProcessor::incrementLockCounts(const std::map<SatelliteId, SatelliteData>& sat_data) {
    for (const auto& [sat, sd] : sat_data) {
        if (sd.has_l1) lock_count_l1_[sat]++;
        if (sd.has_l2) lock_count_l2_[sat]++;
        if (sd.has_l5) lock_count_l5_[sat]++;  // Phase 18 Step 4: only set when enable_l5 populated has_l5
    }
}

void RTKProcessor::handleConsecutiveFloatReset(const ObservationData& rover_obs,
                                               const NavigationData& nav) {
    if (rtk_config_.max_consecutive_float_for_reset <= 0 ||
        consecutive_float_count_ < rtk_config_.max_consecutive_float_for_reset) {
        return;
    }

    // Skip reset under DEMO5_CONTINUOUS policy (simple AR maintains its own state).
    if (rtk_config_.ar_policy == RTKConfig::ARPolicy::DEMO5_CONTINUOUS) {
        consecutive_float_count_ = 0;
        return;
    }

    resetAmbiguityStatesForReacquisition(rover_obs, nav);
}

void RTKProcessor::resetAmbiguityStatesForReacquisition(const ObservationData& rover_obs,
                                                        const NavigationData& nav,
                                                        bool clear_hold_state) {
    // navi.776 A2: a full reacquisition invalidates all learned variances.
    adaptive_noise_tracker_.clear();
    if (!filter_initialized_) {
        consecutive_float_count_ = 0;
        consecutive_nonfix_count_ = 0;
        consecutive_high_float_residual_count_ = 0;
        consecutive_high_fixed_residual_count_ = 0;
        adaptive_dynamic_slip_hold_count_ = 0;
        return;
    }

    for (auto& [sat, idx] : filter_state_.n1_indices) {
        filter_state_.state(idx) = 0.0;
        filter_state_.covariance(idx, idx) = 900.0;
    }
    for (auto& [sat, idx] : filter_state_.n2_indices) {
        filter_state_.state(idx) = 0.0;
        filter_state_.covariance(idx, idx) = 900.0;
    }
    // Phase 18 Step 2: reset N5 ambiguities (no-op until n5_indices populated).
    for (auto& [sat, idx] : filter_state_.n5_indices) {
        filter_state_.state(idx) = 0.0;
        filter_state_.covariance(idx, idx) = 900.0;
    }
    // Also reset position to SPP to prevent baseline drift.
    resetPositionToSPP(rover_obs, nav);
    if (clear_hold_state) {
        restoreHoldState(HoldStateSnapshot{});
    }
    // Ordinary FLOAT resets retain held DD integers for hold fix. A
    // wrong-basin FIX reset explicitly clears them above.
    consecutive_float_count_ = 0;
    consecutive_nonfix_count_ = 0;
    consecutive_high_float_residual_count_ = 0;
    consecutive_high_fixed_residual_count_ = 0;
}

bool RTKProcessor::shouldResetAfterFixedResidualGate() {
    const double max_prefit_rms = rtk_config_.max_fixed_prefit_residual_rms_m;
    const int min_outliers = rtk_config_.min_fixed_prefit_outliers;
    const bool enabled = std::isfinite(max_prefit_rms) && max_prefit_rms > 0.0 &&
                         min_outliers > 0;
    const bool exceeded =
        enabled && std::isfinite(current_update_diagnostics_.prefit_residual_rms_m) &&
        current_update_diagnostics_.prefit_residual_rms_m > max_prefit_rms &&
        current_update_diagnostics_.suppressed_outliers >= min_outliers;
    const double max_covariance_trace =
        rtk_config_.max_fixed_overconfidence_covariance_trace_m2;
    const bool covariance_passes =
        !(std::isfinite(max_covariance_trace) && max_covariance_trace > 0.0) ||
        (std::isfinite(debug_telemetry_.float_position_covariance_trace_m2) &&
         debug_telemetry_.float_position_covariance_trace_m2 <= max_covariance_trace);
    if (!exceeded || !covariance_passes) {
        consecutive_high_fixed_residual_count_ = 0;
        return false;
    }
    ++consecutive_high_fixed_residual_count_;
    if (consecutive_high_fixed_residual_count_ <
        std::max(1, rtk_config_.fixed_prefit_reset_streak)) {
        return false;
    }
    if (!rtk_config_.fixed_prefit_quarantine_only) {
        consecutive_high_fixed_residual_count_ = 0;
    }
    return true;
}

bool RTKProcessor::floatResidualExceedsReacquisitionGate() const {
    if (rtk_config_.ar_policy == RTKConfig::ARPolicy::DEMO5_CONTINUOUS) {
        return false;
    }
    const double max_prefit_rms = rtk_config_.max_float_prefit_residual_rms_m;
    if (std::isfinite(max_prefit_rms) && max_prefit_rms > 0.0 &&
        std::isfinite(current_update_diagnostics_.prefit_residual_rms_m) &&
        current_update_diagnostics_.prefit_residual_rms_m > max_prefit_rms) {
        return true;
    }
    const double max_prefit_residual = rtk_config_.max_float_prefit_residual_max_m;
    if (std::isfinite(max_prefit_residual) && max_prefit_residual > 0.0 &&
        std::isfinite(current_update_diagnostics_.prefit_residual_max_m) &&
        current_update_diagnostics_.prefit_residual_max_m > max_prefit_residual) {
        return true;
    }
    return false;
}

bool RTKProcessor::floatResidualTrustedJumpPassesGate(
    const PositionSolution& float_solution,
    const Vector3d& saved_last_trusted_position,
    bool saved_has_last_trusted,
    const GNSSTime& saved_last_trusted_time,
    bool saved_has_last_trusted_time) const {
    const double min_trusted_jump =
        rtk_config_.min_float_prefit_residual_trusted_jump_m;
    if (!std::isfinite(min_trusted_jump) || min_trusted_jump <= 0.0) {
        return true;
    }
    if (!saved_has_last_trusted || !saved_has_last_trusted_time ||
        !float_solution.position_ecef.allFinite()) {
        return false;
    }
    const double dt = float_solution.time - saved_last_trusted_time;
    if (!std::isfinite(dt) || dt < 0.0) {
        return false;
    }
    const double trusted_jump =
        (float_solution.position_ecef - saved_last_trusted_position).norm();
    return std::isfinite(trusted_jump) && trusted_jump >= min_trusted_jump;
}

bool RTKProcessor::shouldResetAfterFloatResidualGate(
    const PositionSolution& float_solution,
    const Vector3d& saved_last_trusted_position,
    bool saved_has_last_trusted,
    const GNSSTime& saved_last_trusted_time,
    bool saved_has_last_trusted_time) {
    if (!floatResidualExceedsReacquisitionGate()) {
        consecutive_high_float_residual_count_ = 0;
        return false;
    }
    if (!floatResidualTrustedJumpPassesGate(float_solution,
                                            saved_last_trusted_position,
                                            saved_has_last_trusted,
                                            saved_last_trusted_time,
                                            saved_has_last_trusted_time)) {
        consecutive_high_float_residual_count_ = 0;
        return false;
    }
    consecutive_high_float_residual_count_++;
    const int reset_streak =
        std::max(1, rtk_config_.max_float_prefit_residual_reset_streak);
    if (consecutive_high_float_residual_count_ < reset_streak) {
        return false;
    }
    consecutive_high_float_residual_count_ = 0;
    return true;
}

void RTKProcessor::recordFixedEpoch() {
    consecutive_float_count_ = 0;
    consecutive_nonfix_count_ = 0;
    consecutive_high_float_residual_count_ = 0;
}

void RTKProcessor::recordFloatEpoch(const ObservationData& rover_obs, const NavigationData& nav) {
    consecutive_high_fixed_residual_count_ = 0;
    consecutive_float_count_++;
    consecutive_nonfix_count_++;
    if (rtk_config_.max_consecutive_nonfix_for_reset <= 0 ||
        consecutive_nonfix_count_ < rtk_config_.max_consecutive_nonfix_for_reset ||
        rtk_config_.ar_policy == RTKConfig::ARPolicy::DEMO5_CONTINUOUS) {
        return;
    }
    resetAmbiguityStatesForReacquisition(rover_obs, nav);
}

void RTKProcessor::recordFallbackEpoch(const ObservationData& rover_obs, const NavigationData& nav) {
    consecutive_fix_count_ = 0;
    consecutive_float_count_ = 0;
    consecutive_high_float_residual_count_ = 0;
    consecutive_high_fixed_residual_count_ = 0;
    if (!filter_initialized_) {
        consecutive_nonfix_count_ = 0;
        return;
    }
    consecutive_nonfix_count_++;
    if (rtk_config_.max_consecutive_nonfix_for_reset <= 0 ||
        consecutive_nonfix_count_ < rtk_config_.max_consecutive_nonfix_for_reset ||
        rtk_config_.ar_policy == RTKConfig::ARPolicy::DEMO5_CONTINUOUS) {
        return;
    }
    resetAmbiguityStatesForReacquisition(rover_obs, nav);
}

void RTKProcessor::resetPositionToSPP(const ObservationData& rover_obs, const NavigationData& nav) {
    // M1 time updates are strictly single-use. Consume before any mode return
    // so STATIC/MOVING_BASE or a caller mode change cannot retain a stale IMU
    // increment for a later kinematic epoch.
    const bool has_time_update_this_epoch = has_external_position_time_update_;
    const Vector3d position_delta_ecef = external_position_delta_ecef_;
    const Matrix3d position_process_noise_ecef = external_position_process_noise_ecef_;
    const bool has_velocity_update_this_epoch = has_external_velocity_time_update_;
    const Vector3d velocity_ecef = external_velocity_ecef_;
    const Eigen::Matrix<double, 6, 6> position_velocity_process_noise_ecef =
        external_position_velocity_process_noise_ecef_;
    const Matrix3d velocity_initial_covariance_ecef =
        external_velocity_initial_covariance_ecef_;
    has_external_position_time_update_ = false;
    has_external_velocity_time_update_ = false;
    ins_time_update_applied_last_epoch_ = false;

    if (rtk_config_.position_mode == RTKConfig::PositionMode::STATIC) {
        // Static: position accumulates with process noise
        double pos_pnoise = rtk_config_.process_noise_position;  // default 1e-4 m^2/s
        for (int i = 0; i < BASE_STATES; ++i)
            filter_state_.covariance(i, i) += pos_pnoise;
        return;
    }

    const bool moving_base_mode = isMovingBasePositionMode(rtk_config_);

    if (rtk_config_.use_external_position_time_update &&
        has_time_update_this_epoch && !moving_base_mode) {
        const bool applied = rtk_config_.enable_velocity_states &&
                has_velocity_update_this_epoch
            ? rtk_ins_time_update::applyPositionVelocity(
                  filter_state_.state, filter_state_.covariance,
                  position_delta_ecef, velocity_ecef,
                  position_velocity_process_noise_ecef,
                  velocity_initial_covariance_ecef,
                  rtk_config_.ins_time_update_position_q_floor_m2,
                  VELOCITY_STATE_INDEX)
            : rtk_ins_time_update::apply(
                  filter_state_.state, filter_state_.covariance,
                  position_delta_ecef, position_process_noise_ecef,
                  rtk_config_.ins_time_update_position_q_floor_m2);
        if (applied) {
            ++ins_time_update_applied_count_;
            ins_time_update_applied_last_epoch_ = true;
            // Both external mechanisms target the same epoch. Never let a
            // simultaneously queued absolute prior survive this successful
            // update and accidentally seed a later epoch.
            has_external_position_prior_ = false;
            return;
        }
        ++ins_time_update_rejected_count_;
    }

    // Phase 1 GNSS/IMU coupling (docs/design.md): opt-in external position
    // prior (e.g. INS-mechanization-predicted antenna position) in place of
    // the legacy SPP/trusted-position reseed below. The prior is always
    // consumed (has_external_position_prior_ cleared) here, whether or not
    // it is actually applied, so a caller that stops supplying one (e.g. an
    // IMU gap) transparently falls back to the legacy reseed on the very
    // next epoch rather than accidentally reusing a stale value. Guarded on
    // !moving_base_mode: MOVING_BASE tracks a relative baseline against a
    // time-varying base, which an absolute INS-predicted antenna position
    // cannot seed meaningfully.
    const bool has_prior_this_epoch = has_external_position_prior_;
    const Vector3d prior_ecef = external_position_prior_ecef_;
    const Matrix3d prior_cov = external_position_prior_covariance_;
    has_external_position_prior_ = false;
    if (rtk_config_.use_external_position_prior && has_prior_this_epoch && !moving_base_mode) {
        const Vector3d baseline = prior_ecef - base_position_;
        filter_state_.state.head<3>() = baseline;
        const int n = filter_state_.state.size();
        for (int i = 0; i < BASE_STATES; ++i) {
            for (int j = 0; j < n; ++j) {
                filter_state_.covariance(i, j) = 0.0;
                filter_state_.covariance(j, i) = 0.0;
            }
        }
        filter_state_.covariance.block<BASE_STATES, BASE_STATES>(0, 0) = prior_cov;
        return;
    }

    // Dynamic modes: refresh the baseline seed each epoch. Moving-base keeps the
    // relative baseline and only uses absolute rover hints when they exist.
    Vector3d rover_pos;
    double var_pos = moving_base_mode ? 25.0 : 900.0;
    auto spp = spp_processor_.processEpoch(rover_obs, nav);
    if (moving_base_mode) {
        if (rover_obs.receiver_position.norm() > 1e6) {
            rover_pos = rover_obs.receiver_position;
        } else if (has_fixed_solution_) {
            rover_pos = base_position_ + fixed_baseline_;
        } else if (filter_initialized_ && filter_state_.state.size() >= 3) {
            rover_pos = base_position_ + filter_state_.state.head<3>();
        } else if (spp.isValid()) {
            rover_pos = spp.position_ecef;
        } else {
            rover_pos = base_position_;
        }
    } else {
        bool seeded = false;
        if (rtk_config_.use_doppler_float_seed &&
            has_last_trusted_position_ && has_last_trusted_time_ &&
            has_last_doppler_velocity_) {
            const double anchor_age = rover_obs.time - last_trusted_time_;
            const double velocity_age = rover_obs.time - last_doppler_velocity_time_;
            if (std::isfinite(anchor_age) && anchor_age >= 0.0 &&
                anchor_age <= rtk_config_.doppler_float_seed_max_age_s &&
                std::isfinite(velocity_age) && velocity_age >= 0.0 &&
                velocity_age <= 1.0 && last_doppler_velocity_ecef_.allFinite()) {
                rover_pos = last_trusted_position_ +
                            last_doppler_velocity_ecef_ * anchor_age;
                var_pos = std::max(25.0,
                    std::pow(doppler_velocity_sigma_mps_ * std::max(anchor_age, 0.2), 2.0));
                seeded = true;
            }
        }
        if (!seeded && rtk_config_.prefer_trusted_position_seed &&
            has_last_trusted_position_ && has_last_trusted_time_) {
            const double dt = rover_obs.time - last_trusted_time_;
            if (std::isfinite(dt) && dt >= 0.0 && dt <= 1.0) {
                rover_pos = last_trusted_position_;
                var_pos = std::max(25.0, std::pow(3.0 * std::max(dt, 0.2), 2.0));
                seeded = true;
            }
        }

        // WP9: float-trust-policy graceful degradation. Only ever consulted
        // once trust has lapsed (the previous processed epoch did not
        // refresh trust) -- on every epoch of a healthy segment this block
        // is skipped entirely and the pre-WP9 legacy branch below runs
        // unchanged, matching WP8's finding that the wide reset should only
        // need softening during an actual trust drought. LEGACY (the
        // default) never enters this block at all.
        bool wp9_seeded = false;
        if (!seeded &&
            rtk_config_.float_trust_policy != float_trust_policy::FloatTrustPolicy::LEGACY) {
            const bool trust_refreshed_last_epoch =
                has_last_trusted_time_ && has_last_epoch_ &&
                (last_trusted_time_ == last_epoch_time_);
            const bool trust_lapsed = float_trust_policy::hasTrustLapsed(
                has_last_trusted_position_ && has_last_trusted_time_,
                trust_refreshed_last_epoch);
            if (trust_lapsed) {
                double dt_epoch = has_last_epoch_ ? (rover_obs.time - last_epoch_time_) : 0.2;
                if (!std::isfinite(dt_epoch) || dt_epoch <= 0.0) dt_epoch = 0.2;

                if (rtk_config_.float_trust_policy ==
                        float_trust_policy::FloatTrustPolicy::CV_PREDICT &&
                    has_last_solution_position_) {
                    Vector3d velocity = Vector3d::Zero();
                    if (has_prev_trusted_position_ && has_last_trusted_position_ &&
                        has_last_trusted_time_) {
                        velocity = float_trust_policy::estimateVelocityFromTrustedDeltas(
                            last_trusted_position_, prev_trusted_position_,
                            last_trusted_time_ - prev_trusted_time_, 10.0);
                    }
                    rover_pos = float_trust_policy::predictPositionConstantVelocity(
                        last_solution_position_, velocity, dt_epoch);
                    const double previous_var_pos = filter_state_.covariance(0, 0);
                    var_pos = float_trust_policy::growPositionVarianceCvPredict(
                        previous_var_pos, rtk_config_.trust_lapse_qpos_m2_per_s, dt_epoch, 900.0);
                    wp9_seeded = true;
                } else if (rtk_config_.float_trust_policy ==
                               float_trust_policy::FloatTrustPolicy::SCALED_RESET &&
                           spp.isValid()) {
                    const double dt_since_trust = has_last_trusted_time_
                        ? std::max(rover_obs.time - last_trusted_time_, 0.0)
                        : 1.0e6;  // never trusted yet -> effectively at the legacy cap
                    rover_pos = spp.position_ecef;
                    var_pos = float_trust_policy::scaledResetPositionVariance(
                        25.0, rtk_config_.trust_lapse_qpos_m2_per_s, dt_since_trust, 900.0);
                    wp9_seeded = true;
                } else if (rtk_config_.float_trust_policy ==
                               float_trust_policy::FloatTrustPolicy::LAPSE_GATED &&
                           spp.isValid()) {
                    // WP10: only switch off the LEGACY path once the
                    // *continuous* trust lapse exceeds the configured
                    // gate (or, optionally, on a sufficiently NLOS-heavy
                    // epoch regardless of lapse length). Below the gate
                    // (and with the optional NLOS trigger off/unmet),
                    // wp9_seeded is deliberately left false so this falls
                    // straight through to the unmodified legacy fallback
                    // branch below -- bit-identical to LEGACY for short
                    // lapses by construction, not just numerically close.
                    const double dt_since_trust = has_last_trusted_time_
                        ? std::max(rover_obs.time - last_trusted_time_, 0.0)
                        : 1.0e6;  // never trusted yet -> effectively at the legacy cap
                    // current_epoch_nlos_fraction_ still holds the *previous*
                    // epoch's value here (this epoch's own value isn't
                    // computed until collectSatelliteData() runs, later in
                    // processRTKEpoch()) -- a one-epoch-lagged proxy, cheap
                    // and adequate for the multi-second-to-minute NLOS-heavy
                    // dwells (e.g. the canyon) this trigger targets.
                    const bool nlos_frac_trigger =
                        rtk_config_.trust_lapse_gate_nlos_frac >= 0.0 &&
                        std::isfinite(current_epoch_nlos_fraction_) &&
                        current_epoch_nlos_fraction_ > rtk_config_.trust_lapse_gate_nlos_frac;
                    if (float_trust_policy::lapseGateExceeded(
                            dt_since_trust, rtk_config_.trust_lapse_gate_s) ||
                        nlos_frac_trigger) {
                        rover_pos = spp.position_ecef;
                        var_pos = float_trust_policy::scaledResetPositionVariance(
                            25.0, rtk_config_.trust_lapse_qpos_m2_per_s, dt_since_trust, 900.0);
                        wp9_seeded = true;
                    }
                }
            }
        }

        if (!seeded && !wp9_seeded) {
            if (rtk_config_.prefer_rover_position_seed &&
                rover_obs.receiver_position.norm() > 1e6) {
                rover_pos = rover_obs.receiver_position;
            } else if (spp.isValid()) {
                rover_pos = spp.position_ecef;
            } else if (has_last_fixed_position_) {
                rover_pos = last_fixed_position_;
            } else if (rover_obs.receiver_position.norm() > 1e6) {
                rover_pos = rover_obs.receiver_position;
            } else if (has_last_solution_position_) {
                rover_pos = last_solution_position_;
            } else {
                rover_pos = base_position_;
            }
        }
    }
    Vector3d baseline = rover_pos - base_position_;

    filter_state_.state.head<3>() = baseline;
    int n = filter_state_.state.size();
    for (int i = 0; i < BASE_STATES; ++i) {
        for (int j = 0; j < n; ++j) {
            filter_state_.covariance(i, j) = 0.0;
            filter_state_.covariance(j, i) = 0.0;
        }
        filter_state_.covariance(i, i) = var_pos;
    }
}

// ============================================================
// Main processing (RTKLIB relpos equivalent)
// ============================================================
PositionSolution RTKProcessor::processRTKEpoch(const ObservationData& rover_obs,
    const ObservationData& base_obs, const NavigationData& nav) {
    if (rtk_config_.max_fixed_doppler_consensus_m > 0.0 &&
        has_doppler_continuity_position_ && has_last_doppler_velocity_) {
        const double dt = rover_obs.time - doppler_continuity_time_;
        const double velocity_age = rover_obs.time - last_doppler_velocity_time_;
        if (std::isfinite(dt) && dt > 0.0 && dt <= 2.0 &&
            std::isfinite(velocity_age) && velocity_age >= 0.0 && velocity_age <= 2.0 &&
            last_doppler_velocity_ecef_.allFinite()) {
            doppler_continuity_position_ecef_ += last_doppler_velocity_ecef_ * dt;
            doppler_continuity_time_ = rover_obs.time;
        } else if (!std::isfinite(dt) || dt < 0.0 || dt > 2.0) {
            has_doppler_continuity_position_ = false;
        }
    }
    PositionSolution solution = processRTKEpochInternal(rover_obs, base_obs, nav);

    // Doppler-derived velocity: SPPProcessor now populates has_velocity on
    // its own solutions (spp.cpp), so the fallback-to-SPP paths inside
    // processRTKEpochInternal (fallback_spp()/the catch block) already carry
    // a velocity through untouched. The DD-RTK float/fixed paths
    // (generateSolution()) do not compute velocity at all, so run the same
    // SPP-style Doppler LS directly on the rover observations here -- no
    // dependency on RTK's DD/ambiguity state, just broadcast ephemeris +
    // Doppler, per docs/design.md.
    if (solution.isValid() &&
        (!solution.has_velocity || rtk_config_.max_fixed_doppler_consensus_m > 0.0)) {
        const Vector3d velocity_linearization_position =
            rtk_config_.max_fixed_doppler_consensus_m > 0.0 &&
                    has_doppler_continuity_position_
                ? doppler_continuity_position_ecef_
                : solution.position_ecef;
        const auto velocity_result = spp_velocity::solveVelocityFromObservations(
            rover_obs, nav, velocity_linearization_position, doppler_velocity_sigma_mps_);
        if (velocity_result.ok) {
            solution.velocity_ecef = velocity_result.velocity_ecef;
            solution.velocity_covariance = velocity_result.velocity_covariance;
            solution.has_velocity = true;
            solution.receiver_clock_drift = velocity_result.receiver_clock_drift;
        }
    }

    if (solution.isValid() && solution.has_velocity && solution.velocity_ecef.allFinite()) {
        last_doppler_velocity_ecef_ = solution.velocity_ecef;
        last_doppler_velocity_time_ = rover_obs.time;
        has_last_doppler_velocity_ = true;
    }

    if (rtk_config_.max_fixed_doppler_consensus_m > 0.0 &&
        solution.status == SolutionStatus::FIXED && solution.position_ecef.allFinite()) {
        // Only a candidate that already passed the independent consensus gate
        // may re-anchor the track. This bounds integration drift without letting
        // a rejected RTK basin contaminate the Doppler continuity state.
        doppler_continuity_position_ecef_ = solution.position_ecef;
        doppler_continuity_time_ = rover_obs.time;
        has_doppler_continuity_position_ = true;
    }

    return solution;
}

PositionSolution RTKProcessor::processRTKEpochInternal(const ObservationData& rover_obs,
    const ObservationData& base_obs, const NavigationData& nav) {
    debug_telemetry_ = EpochDebugTelemetry{};
    PositionSolution solution;
    solution.time = rover_obs.time;
    solution.status = SolutionStatus::NONE;
    current_update_diagnostics_ = RTKUpdateDiagnostics{};
    // WP7: record current tow for buildMeasurementBlocks()'s NLOS weight lookup.
    current_epoch_time_ = rover_obs.time;

    try {
        const bool moving_base_mode = isMovingBasePositionMode(rtk_config_);
        if (moving_base_mode && base_obs.receiver_position.norm() > 1e6) {
            setBasePosition(base_obs.receiver_position);
        }
        if (!base_position_known_) {
            auto spp = spp_processor_.processEpoch(rover_obs, nav);
            rememberSolution(spp);
            consecutive_fix_count_ = 0;
            consecutive_float_count_ = 0;
            consecutive_nonfix_count_ = 0;
            consecutive_high_float_residual_count_ = 0;
            consecutive_high_fixed_residual_count_ = 0;
            adaptive_dynamic_slip_hold_count_ = 0;
            return spp;
        }

        auto current_spp = spp_processor_.processEpoch(rover_obs, nav);
        auto fallback_spp = [&]() {
            last_ar_ratio_ = 0.0;
            last_num_fixed_ambiguities_ = 0;
            auto spp = current_spp;
            if (!moving_base_mode && spp.isValid() && has_last_trusted_position_ && has_last_trusted_time_) {
                const double trusted_jump =
                    (spp.position_ecef - last_trusted_position_).norm();
                if (spp.num_satellites <= 5 && trusted_jump > 25.0) {
                    spp = PositionSolution{};
                    spp.time = rover_obs.time;
                    spp.status = SolutionStatus::NONE;
                }
            }
            if (!moving_base_mode && spp.isValid() && has_last_solution_position_ && has_last_epoch_) {
                double dt = rover_obs.time - last_epoch_time_;
                if (!std::isfinite(dt) || dt < 0.5) dt = 1.0;
                const double jump =
                    (spp.position_ecef - last_solution_position_).norm();
                const double max_jump = std::max(100.0, 30.0 * dt);
                if (spp.num_satellites <= 4 && jump > max_jump) {
                    spp = PositionSolution{};
                    spp.time = rover_obs.time;
                    spp.status = SolutionStatus::NONE;
                }
            }
            if (spp.isValid()) spp.status = SolutionStatus::SPP;
            rememberSolution(spp);
            recordFallbackEpoch(rover_obs, nav);
            return spp;
        };

        auto finitePosition = [](const PositionSolution& sol) {
            return sol.position_ecef.allFinite();
        };

        auto deviatesTooFarFromSPP = [&](const PositionSolution& sol, double threshold_m) {
            if (!current_spp.isValid() || !finitePosition(sol) || !current_spp.position_ecef.allFinite()) {
                return false;
            }
            return (sol.position_ecef - current_spp.position_ecef).norm() > threshold_m;
        };

        if (!filter_initialized_) {
            auto init_sat = collectSatelliteData(rover_obs, base_obs, nav);
            if (init_sat.size() < 4) {
                return fallback_spp();
            }
            if (!initializeFilter(rover_obs, base_obs, nav)) {
                return fallback_spp();
            }
        }

        resetPositionToSPP(rover_obs, nav);

        handleConsecutiveFloatReset(rover_obs, nav);

        auto sat_data = collectSatelliteData(rover_obs, base_obs, nav);
        if (sat_data.size() < 4) {
            return fallback_spp();
        }

        // WP9/WP10: cache this epoch's NLOS fraction (sat_data is only
        // available locally here) for two downstream readers: WP9's
        // rememberSolution() jump-gate relax check (same epoch), and
        // WP10's LAPSE_GATED --trust-lapse-gate-nlos-frac trigger, which
        // reads it (necessarily one-epoch-lagged) from the *next*
        // epoch's resetPositionToSPP(), called before this recomputation.
        // NaN unless one of the two levers is on and a table is loaded --
        // std::isfinite() at every read site gates this to a strict no-op
        // otherwise.
        current_epoch_nlos_fraction_ = std::numeric_limits<double>::quiet_NaN();
        if ((rtk_config_.trust_gate_nlos_relax ||
             rtk_config_.trust_lapse_gate_nlos_frac >= 0.0) &&
            nlos_weight_table_ && !nlos_weight_table_->empty()) {
            int total = 0;
            int nlos = 0;
            for (const auto& kv : sat_data) {
                ++total;
                const double los_prob = nlos_weights::lookupLosProb(
                    *nlos_weight_table_, current_epoch_time_.tow, kv.first.toString(),
                    rtk_config_.nlos_tow_tolerance_s);
                if (los_prob < 0.5) ++nlos;
            }
            current_epoch_nlos_fraction_ = total > 0 ? static_cast<double>(nlos) / total : 0.0;
        }

        double state_dt = 1.0;
        if (has_last_epoch_) {
            state_dt = rover_obs.time - last_epoch_time_;
        }
        updateGlonassHardwareBias(state_dt);

        SatelliteId new_ref = selectReferenceSatellite(sat_data);
        handleReferenceSatelliteChange(new_ref, sat_data);
        current_sat_data_ = sat_data;
        updateBias(sat_data, state_dt);

        // Iterative KF update
        bool filter_ok = false;
        int max_kf_iterations = rtk_config_.kf_iterations;
        if (isDynamicPositionMode(rtk_config_) &&
            has_last_solution_position_ && max_kf_iterations > 2) {
            max_kf_iterations = 2;
        }
        for (int iter = 0; iter < max_kf_iterations; ++iter) {
            const Vector3d baseline_before_iter = filter_state_.state.head<3>();
            filter_ok = updateFilter(sat_data);
            if (!filter_ok) break;
            current_update_diagnostics_.iterations++;
            if (iter >= 1) {
                const double baseline_step =
                    (filter_state_.state.head<3>() - baseline_before_iter).norm();
                if (baseline_step < 1e-3) {
                    break;
                }
            }
        }

        if (filter_ok) {
            incrementLockCounts(sat_data);
        }

        if (filter_ok) {
            int n_sats = static_cast<int>(sat_data.size());
            const Vector3d saved_last_solution_position = last_solution_position_;
            const bool saved_has_last_solution = has_last_solution_position_;
            const GNSSTime saved_last_solution_time = last_epoch_time_;
            const bool saved_has_last_solution_time = has_last_epoch_;
            const Vector3d saved_last_trusted_position = last_trusted_position_;
            const bool saved_has_last_trusted = has_last_trusted_position_;
            const GNSSTime saved_last_trusted_time = last_trusted_time_;
            const bool saved_has_last_trusted_time = has_last_trusted_time_;
            fixed_update_gate_previous_position_ = saved_last_solution_position;
            fixed_update_gate_previous_time_ = saved_last_solution_time;
            has_fixed_update_gate_previous_solution_ =
                saved_has_last_solution && saved_has_last_solution_time;
            auto restoreRememberedState = [&]() {
                last_solution_position_ = saved_last_solution_position;
                has_last_solution_position_ = saved_has_last_solution;
                last_epoch_time_ = saved_last_solution_time;
                has_last_epoch_ = saved_has_last_solution_time;
                last_trusted_position_ = saved_last_trusted_position;
                has_last_trusted_position_ = saved_has_last_trusted;
                last_trusted_time_ = saved_last_trusted_time;
                has_last_trusted_time_ = saved_has_last_trusted_time;
                consecutive_high_float_residual_count_ = 0;
            };
            last_ar_ratio_ = 0.0;
            last_num_fixed_ambiguities_ = 0;
            solution = generateSolution(rover_obs.time, SolutionStatus::FLOAT, n_sats);
            const PositionSolution float_solution = solution;

            const double max_float_spp_divergence_m = rtk_config_.max_float_spp_divergence_m;
            const bool float_exceeds_spp_gate =
                std::isfinite(max_float_spp_divergence_m) &&
                max_float_spp_divergence_m > 0.0 &&
                deviatesTooFarFromSPP(solution, max_float_spp_divergence_m);
            if (!finitePosition(solution) ||
                deviatesTooFarFromSPP(solution, 150.0) ||
                float_exceeds_spp_gate) {
                restoreRememberedState();
                return fallback_spp();
            }

            if (!moving_base_mode &&
                n_sats <= 4 && has_last_trusted_position_ && has_last_trusted_time_) {
                const double trusted_jump =
                    (solution.position_ecef - saved_last_trusted_position).norm();
                if (trusted_jump > 25.0) {
                    restoreRememberedState();
                    return fallback_spp();
                }
            }

            if (!moving_base_mode && saved_has_last_trusted && saved_has_last_trusted_time) {
                double dt = rover_obs.time - saved_last_trusted_time;
                if (!std::isfinite(dt) || dt < 0.5) {
                    dt = 1.0;
                }
                if (dt <= 3.0) {
                    const double trusted_jump =
                        (solution.position_ecef - saved_last_trusted_position).norm();
                    const double max_trusted_jump = std::max(8.0, 12.0 * dt);
                    if (trusted_jump > max_trusted_jump) {
                        restoreRememberedState();
                        return fallback_spp();
                    }
                }
            }

            if (!moving_base_mode && saved_has_last_solution && saved_has_last_solution_time) {
                double dt = rover_obs.time - saved_last_solution_time;
                if (!std::isfinite(dt) || dt < 0.5) dt = 1.0;
                const double jump =
                    (solution.position_ecef - saved_last_solution_position).norm();
                const double max_jump = std::max(120.0, 35.0 * dt);
                if (n_sats <= 5 && jump > max_jump) {
                    restoreRememberedState();
                    return fallback_spp();
                }
            }

            const auto saved_hold_state = captureHoldState();
            bool forced_fixed_reacquisition_reset = false;
            bool fixed_prefit_quarantine = false;
            auto emitReacquisitionFloat = [&]() {
                debug_telemetry_.post_validation_rejected = true;
                debug_telemetry_.reject_reason = "fixed_prefit_wrong_basin";
                has_fixed_solution_ = false;
                restoreHoldState(saved_hold_state);
                restoreRememberedState();
                solution = float_solution;
                updateStatistics(SolutionStatus::FLOAT);
                consecutive_fix_count_ = 0;
                resetAmbiguityStatesForReacquisition(rover_obs, nav, true);
                has_last_fixed_position_ = false;
                has_last_fixed_time_ = false;
                has_last_trusted_position_ = false;
                has_last_trusted_time_ = false;
                forced_fixed_reacquisition_reset = true;
            };
            has_fixed_solution_ = false;
            struct ARCandidate {
                bool valid = false;
                Vector3d baseline = Vector3d::Zero();
                double ratio = 0.0;
                int num_fixed_ambiguities = 0;
                std::vector<DDPair> dd_pairs;
                std::vector<int> best_subset;
                VectorXd dd_fixed;
            };

            auto capture_candidate = [&]() {
                ARCandidate candidate;
                if (!has_fixed_solution_) {
                    return candidate;
                }
                candidate.valid = true;
                candidate.baseline = fixed_baseline_;
                candidate.ratio = last_ar_ratio_;
                candidate.num_fixed_ambiguities = last_num_fixed_ambiguities_;
                candidate.dd_pairs = last_dd_pairs_;
                candidate.best_subset = last_best_subset_;
                candidate.dd_fixed = last_dd_fixed_;
                return candidate;
            };

            auto restore_candidate = [&](const ARCandidate& candidate) {
                has_fixed_solution_ = candidate.valid;
                fixed_baseline_ = candidate.baseline;
                last_ar_ratio_ = candidate.ratio;
                last_num_fixed_ambiguities_ = candidate.num_fixed_ambiguities;
                last_dd_pairs_ = candidate.dd_pairs;
                last_best_subset_ = candidate.best_subset;
                last_dd_fixed_ = candidate.dd_fixed;
            };

            const int min_lock = std::max(1, rtk_config_.min_lock_count);
            auto build_pairs_for_mode = [&](RTKConfig::GlonassARMode mode) {
                const auto saved_mode = rtk_config_.glonass_ar_mode;
                rtk_config_.glonass_ar_mode = mode;
                auto dd_pairs = buildDoubleDifferencePairs(sat_data, min_lock);
                rtk_config_.glonass_ar_mode = saved_mode;
                return dd_pairs;
            };

            auto try_ar_mode = [&](RTKConfig::GlonassARMode mode,
                                   const std::vector<DDPair>* prebuilt_pairs) {
                const auto saved_mode = rtk_config_.glonass_ar_mode;
                rtk_config_.glonass_ar_mode = mode;
                has_fixed_solution_ = false;
                last_ar_ratio_ = 0.0;
                last_num_fixed_ambiguities_ = 0;
                const bool resolved =
                    (prebuilt_pairs ? resolveAmbiguities(*prebuilt_pairs) : resolveAmbiguities()) &&
                    has_fixed_solution_;
                ARCandidate candidate;
                if (resolved) {
                    candidate = capture_candidate();
                }
                rtk_config_.glonass_ar_mode = saved_mode;
                return candidate;
            };

            bool have_fix_candidate = false;
            if (rtk_config_.glonass_ar_mode == RTKConfig::GlonassARMode::AUTOCAL) {
                const auto preview_pairs =
                    build_pairs_for_mode(RTKConfig::GlonassARMode::AUTOCAL);
                const int glonass_pair_count = static_cast<int>(std::count_if(
                    preview_pairs.begin(), preview_pairs.end(), [](const DDPair& pair) {
                        return pair.ref_sat.system == GNSSSystem::GLONASS;
                    }));

                const auto autocal_candidate =
                    glonass_pair_count > 0
                        ? try_ar_mode(RTKConfig::GlonassARMode::AUTOCAL, &preview_pairs)
                        : ARCandidate{};
                const bool confident_autocal =
                    autocal_candidate.valid &&
                    autocal_candidate.ratio >= rtk_config_.ambiguity_ratio_threshold + 0.4 &&
                    autocal_candidate.num_fixed_ambiguities >=
                        std::max(4, rtk_config_.min_satellites_for_ar - 1);
                std::vector<DDPair> classic_pairs;
                const auto classic_candidate = [&]() {
                    if (glonass_pair_count != 0 && confident_autocal) {
                        return ARCandidate{};
                    }
                    if (glonass_pair_count == 0) {
                        return try_ar_mode(RTKConfig::GlonassARMode::OFF, &preview_pairs);
                    }
                    classic_pairs = build_pairs_for_mode(RTKConfig::GlonassARMode::OFF);
                    return try_ar_mode(RTKConfig::GlonassARMode::OFF, &classic_pairs);
                }();
                ARCandidate chosen;
                if (autocal_candidate.valid && classic_candidate.valid) {
                    chosen = (autocal_candidate.ratio + 1e-6 >= classic_candidate.ratio)
                                 ? autocal_candidate
                                 : classic_candidate;
                } else if (autocal_candidate.valid) {
                    chosen = autocal_candidate;
                } else if (classic_candidate.valid) {
                    chosen = classic_candidate;
                }
                if (chosen.valid) {
                    restore_candidate(chosen);
                    have_fix_candidate = true;
                } else {
                    has_fixed_solution_ = false;
                    last_ar_ratio_ = 0.0;
                    last_num_fixed_ambiguities_ = 0;
                }
            } else if (resolveAmbiguities() && has_fixed_solution_) {
                have_fix_candidate = true;
            }

            bool applied_fix_solution = false;
            if (have_fix_candidate && has_fixed_solution_) {
                debug_telemetry_.validation_attempted = true;
                const bool validation_ok = validateFixedSolution(sat_data, rover_obs.time);
                debug_telemetry_.validation_passed = validation_ok;
                if (validation_ok) {
                    Vector3d saved_baseline = filter_state_.state.head<3>();
                    filter_state_.state.head<3>() = fixed_baseline_;
                    solution = generateSolution(rover_obs.time, SolutionStatus::FIXED, n_sats);
                    filter_state_.state.head<3>() = saved_baseline;
                    const double fixed_float_jump =
                        (solution.position_ecef - float_solution.position_ecef).norm();
                    debug_telemetry_.fixed_float_jump_m = fixed_float_jump;
                    bool exceeds_trusted_jump = false;
                    if (saved_has_last_trusted && saved_has_last_trusted_time) {
                        const double dt = rover_obs.time - saved_last_trusted_time;
                        exceeds_trusted_jump = rtk_validation::exceedsAdaptiveJump(
                            solution.position_ecef,
                            saved_last_trusted_position,
                            dt,
                            20.0,
                            25.0);
                    }
                    if (!finitePosition(solution) ||
                        deviatesTooFarFromSPP(solution, 150.0) ||
                        fixed_float_jump > 20.0 ||
                        exceeds_trusted_jump) {
                        debug_telemetry_.post_validation_rejected = true;
                        if (!finitePosition(solution)) {
                            debug_telemetry_.reject_reason = "nonfinite_fixed";
                        } else if (deviatesTooFarFromSPP(solution, 150.0)) {
                            debug_telemetry_.reject_reason = "fixed_spp_distance";
                        } else if (fixed_float_jump > 20.0) {
                            debug_telemetry_.reject_reason = "fixed_float_jump";
                        } else if (exceeds_trusted_jump) {
                            debug_telemetry_.reject_reason = "trusted_jump";
                        }
                        has_fixed_solution_ = false;
                        restoreHoldState(saved_hold_state);
                        last_trusted_position_ = saved_last_trusted_position;
                        has_last_trusted_position_ = saved_has_last_trusted;
                        last_trusted_time_ = saved_last_trusted_time;
                        has_last_trusted_time_ = saved_has_last_trusted_time;
                    } else if (!moving_base_mode && shouldResetAfterFixedResidualGate()) {
                        if (rtk_config_.fixed_prefit_quarantine_only) {
                            debug_telemetry_.post_validation_rejected = true;
                            debug_telemetry_.reject_reason = "fixed_prefit_quarantine";
                            has_fixed_solution_ = false;
                            restoreRememberedState();
                            fixed_prefit_quarantine = true;
                        } else {
                            emitReacquisitionFloat();
                        }
                    } else {
                        updateStatistics(SolutionStatus::FIXED);
                        consecutive_fix_count_++;
                        consecutive_float_count_ = 0;
                        recordFixedEpoch();
                        debug_telemetry_.final_fixed_applied = true;

                        // Save fixed position for next epoch's position reset
                        last_fixed_position_ = base_position_ + fixed_baseline_;
                        has_last_fixed_position_ = true;
                        last_fixed_time_ = rover_obs.time;
                        has_last_fixed_time_ = true;

                        // holdamb: constrain SD ambiguities toward validated DD integers
                        if (consecutive_fix_count_ >= rtk_config_.min_hold_count) {
                            applyHoldAmbiguity();
                        }
                        applied_fix_solution = true;
                    }
                } else {
                    if (debug_telemetry_.reject_reason.empty()) {
                        debug_telemetry_.reject_reason = "fixed_validation";
                    }
                    has_fixed_solution_ = false;
                    // Mirror the post-validation reject branch above: validateFixedSolution
                    // ran after generateSolution(FIXED) which updated last_trusted_position_
                    // via rememberSolution; if we don't roll those back, downstream epochs
                    // see the rejected fix as "trusted" and fail deviatesTooFarFromSPP /
                    // trusted-jump gates, cascading into fallback_spp and aborting the run.
                    last_trusted_position_ = saved_last_trusted_position;
                    has_last_trusted_position_ = saved_has_last_trusted;
                    last_trusted_time_ = saved_last_trusted_time;
                    has_last_trusted_time_ = saved_has_last_trusted_time;
                }
            }

            if (!moving_base_mode &&
                !applied_fix_solution &&
                !forced_fixed_reacquisition_reset &&
                !fixed_prefit_quarantine &&
                rtk_config_.ar_policy != RTKConfig::ARPolicy::DEMO5_CONTINUOUS &&
                rtk_validation::canAttemptHoldFix(consecutive_fix_count_,
                                                  rtk_config_.min_hold_count,
                                                  saved_hold_state.has_last_fixed_position,
                                                  saved_hold_state.hasHeldIntegers())) {
                restoreHoldState(saved_hold_state);
                if (tryHoldFix(sat_data, rover_obs.time, n_sats, solution)) {
                    if (shouldResetAfterFixedResidualGate()) {
                        emitReacquisitionFloat();
                    } else {
                        updateStatistics(SolutionStatus::FIXED);
                        consecutive_fix_count_++;
                        consecutive_float_count_ = 0;
                        recordFixedEpoch();
                        debug_telemetry_.final_fixed_applied = true;
                        if (consecutive_fix_count_ >= rtk_config_.min_hold_count) {
                            applyHoldAmbiguity();
                        }
                        applied_fix_solution = true;
                    }
                }
            }

            if (!applied_fix_solution && !forced_fixed_reacquisition_reset) {
                restoreHoldState(
                    fixed_prefit_quarantine ? HoldStateSnapshot{} : saved_hold_state);
                solution = float_solution;
                const bool reset_after_high_float =
                    !moving_base_mode &&
                    shouldResetAfterFloatResidualGate(float_solution,
                                                      saved_last_trusted_position,
                                                      saved_has_last_trusted,
                                                      saved_last_trusted_time,
                                                      saved_has_last_trusted_time);
                if (reset_after_high_float) {
                    restoreRememberedState();
                } else {
                    rememberSolution(float_solution);
                }
                updateStatistics(SolutionStatus::FLOAT);
                consecutive_fix_count_ = 0;
                if (reset_after_high_float) {
                    resetAmbiguityStatesForReacquisition(rover_obs, nav);
                } else {
                    recordFloatEpoch(rover_obs, nav);
                    if (fixed_prefit_quarantine) {
                        consecutive_high_fixed_residual_count_ = std::max(
                            1, rtk_config_.fixed_prefit_reset_streak);
                    }
                }
            }
        } else {
            return fallback_spp();
        }
    } catch (const std::exception& e) {
        std::cerr << "RTK exception: " << e.what() << std::endl;
        auto spp = spp_processor_.processEpoch(rover_obs, nav);
        rememberSolution(spp);
        consecutive_fix_count_ = 0;
        consecutive_float_count_ = 0;
        consecutive_nonfix_count_ = 0;
        consecutive_high_float_residual_count_ = 0;
        consecutive_high_fixed_residual_count_ = 0;
        adaptive_dynamic_slip_hold_count_ = 0;
        return spp;
    }
    return solution;
}

// ============================================================
// KF update: DD observation model with H mapping to SD states
// ============================================================
std::vector<rtk_measurement::MeasurementBlock> RTKProcessor::buildMeasurementBlocks(
    const std::map<SatelliteId, SatelliteData>& sat_data) const {
    const Vector3d rover_pos = base_position_ + filter_state_.state.head<3>();
    const bool estimate_iono = usesEstimatedIono(rtk_config_);
    const auto selection_snapshot = buildSelectionSnapshot(sat_data);
    std::vector<rtk_measurement::MeasurementBlock> blocks;

    // navi.776 B2: rover-only between-satellite SD Doppler rows. Skipped
    // until the first INS position/velocity time update has initialized the
    // velocity covariance -- before that the rows would carry residuals with
    // no active velocity columns and pollute NIS.
    const bool build_doppler_rows =
        rtk_config_.enable_doppler_measurement_rows &&
        rtk_config_.enable_velocity_states &&
        filter_state_.state.size() >= VELOCITY_STATE_INDEX + VELOCITY_STATES &&
        filter_state_.covariance(VELOCITY_STATE_INDEX, VELOCITY_STATE_INDEX) > 0.0;
    const Vector3d velocity_estimate =
        build_doppler_rows ? Vector3d(filter_state_.state.segment<3>(VELOCITY_STATE_INDEX))
                           : Vector3d::Zero();
    // Predicted range rate at the current velocity estimate (RTKLIB resdop
    // form): e.(v_sat - v_rx) + Sagnac rate term - c * sat clock drift. The
    // receiver clock drift term is omitted -- it cancels exactly in the
    // between-satellite difference.
    auto predicted_range_rate_mps = [&](const SatelliteData& s) -> double {
        const Vector3d e = (s.sat_pos - rover_pos).normalized();
        const double sagnac_rate =
            constants::OMEGA_E / constants::SPEED_OF_LIGHT *
            (s.sat_vel.y() * rover_pos.x() + s.sat_pos.y() * velocity_estimate.x() -
             s.sat_vel.x() * rover_pos.y() - s.sat_pos.x() * velocity_estimate.y());
        return (s.sat_vel - velocity_estimate).dot(e) + sagnac_rate -
               constants::SPEED_OF_LIGHT * s.sat_clock_drift;
    };

    // WP7: NLOS/multipath sigma inflation. Returns 1.0 (no-op) whenever the
    // feature is off or no table/entry is available, so this is bit-identical
    // to pre-WP7 behavior by construction when nlos_weight_mode == OFF.
    const bool nlos_weighting_active =
        rtk_config_.nlos_weight_mode != nlos_weights::NlosWeightMode::OFF &&
        nlos_weight_table_ && !nlos_weight_table_->empty();
    auto nlos_variance_factor = [&](const SatelliteId& sat) -> double {
        if (!nlos_weighting_active) return 1.0;
        const double los_prob = nlos_weights::lookupLosProb(
            *nlos_weight_table_, current_epoch_time_.tow, sat.toString(),
            rtk_config_.nlos_tow_tolerance_s);
        return nlos_weights::nlosVarianceInflationFactor(
            los_prob,
            rtk_config_.nlos_weight_mode,
            rtk_config_.nlos_continuous_los_prob_floor,
            rtk_config_.nlos_two_tier_los_threshold,
            rtk_config_.nlos_two_tier_sigma_inflation);
    };

    for (GNSSSystem system : kRTKSupportedSystems) {
        if (!isEnabledRTKSystem(rtk_config_, system)) continue;
        SatelliteId ref_sat;
        const SatelliteId* forced_ref = nullptr;
        if (rtk_config_.cmc_aware_reference_selection) {
            const auto forced_it = cmc_aware_ref_by_system_.find(system);
            if (forced_it != cmc_aware_ref_by_system_.end()) forced_ref = &forced_it->second;
        }
        if (forced_ref != nullptr) {
            ref_sat = *forced_ref;
        } else if (!rtk_selection::selectSystemReferenceSatellite(selection_snapshot, system, 0,
                                                                    ref_sat)) {
            continue;
        }

        auto ref_it = sat_data.find(ref_sat);
        if (ref_it == sat_data.end()) continue;
        const auto& ref_sd = ref_it->second;
        const auto system_pairs = rtk_selection::buildDoubleDifferencePairsForSystem(
            selection_snapshot,
            system,
            0,
            requiresMatchedCarrierWavelength(rtk_config_, system),
            forced_ref);
        const double rr_ref = geodist_range(ref_sd.sat_pos, rover_pos) +
                              tropModel(rover_pos, ref_sd.elevation);
        const double br_ref = geodist_range(ref_sd.sat_pos_base, base_position_) +
                              tropModel(base_position_, ref_sd.base_elevation);
        const Vector3d los_ref = (ref_sd.sat_pos - rover_pos).normalized();
        auto signal_snr_dbhz = [](const SatelliteData& sd, int freq) {
            if (freq == 0) return combinedSnrDbHz(sd.rover_l1_snr, sd.base_l1_snr);
            if (freq == 1) return combinedSnrDbHz(sd.rover_l2_snr, sd.base_l2_snr);
            return combinedSnrDbHz(sd.rover_l5_snr, sd.base_l5_snr);  // Phase 18 Step 4
        };
        // Phase 18 Step 4: per-frequency accessors (parallel to updateBias). freq=2 returns L5.
        auto freq_wavelength_local = [](const SatelliteData& sd, int freq) -> double {
            if (freq == 0) return sd.l1_wavelength;
            if (freq == 1) return sd.l2_wavelength;
            return sd.l5_wavelength;
        };
        auto freq_frequency_hz_local = [](const SatelliteData& sd, int freq) -> double {
            if (freq == 0) return sd.l1_frequency_hz;
            if (freq == 1) return sd.l2_frequency_hz;
            return sd.l5_frequency_hz;
        };
        auto freq_phase_diff_local = [](const SatelliteData& sd, int freq) -> double {
            if (freq == 0) return sd.rover_l1_phase - sd.base_l1_phase;
            if (freq == 1) return sd.rover_l2_phase - sd.base_l2_phase;
            return sd.rover_l5_phase - sd.base_l5_phase;
        };
        auto freq_code_diff_local = [](const SatelliteData& sd, int freq) -> double {
            if (freq == 0) return sd.rover_l1_code - sd.base_l1_code;
            if (freq == 1) return sd.rover_l2_code - sd.base_l2_code;
            return sd.rover_l5_code - sd.base_l5_code;
        };
        auto freq_rover_doppler_local = [](const SatelliteData& sd, int freq) -> double {
            if (freq == 0) return sd.rover_l1_doppler;
            if (freq == 1) return sd.rover_l2_doppler;
            return sd.rover_l5_doppler;
        };
        auto freq_has_doppler_local = [](const SatelliteData& sd, int freq) -> bool {
            if (freq == 0) return sd.has_l1_doppler;
            if (freq == 1) return sd.has_l2_doppler;
            return sd.has_l5_doppler;
        };
        // Elevation-law Doppler variance, same 1/sin^2(el) family as varerr.
        auto doppler_variance = [&](double elevation) -> double {
            double sin_el = std::sin(elevation);
            if (sin_el < 0.1) sin_el = 0.1;
            const double sigma = rtk_config_.doppler_row_sigma_mps;
            return sigma * sigma * (1.0 + 1.0 / (sin_el * sin_el));
        };

        auto append_frequency_blocks = [&](int freq) {
            rtk_measurement::MeasurementBlock phase_block;
            phase_block.kind = rtk_measurement::MeasurementKind::PHASE;
            phase_block.frequency_index = freq;
            rtk_measurement::MeasurementBlock code_block;
            code_block.kind = rtk_measurement::MeasurementKind::CODE;
            code_block.frequency_index = freq;
            rtk_measurement::MeasurementBlock doppler_block;
            doppler_block.kind = rtk_measurement::MeasurementKind::DOPPLER;
            doppler_block.frequency_index = freq;
            const bool ref_doppler_ok =
                build_doppler_rows && freq_has_doppler_local(ref_sd, freq) &&
                ref_sd.has_sat_velocity &&
                freq_frequency_hz_local(ref_sd, freq) > 0.0;
            // Between-satellite SD reference terms: measured range rate
            // (RINEX sign: rr = -D * c / f) and prediction at current v_hat.
            const double rr_obs_ref =
                ref_doppler_ok
                    ? -freq_rover_doppler_local(ref_sd, freq) *
                          (constants::SPEED_OF_LIGHT / freq_frequency_hz_local(ref_sd, freq))
                    : 0.0;
            const double rr_pred_ref = ref_doppler_ok ? predicted_range_rate_mps(ref_sd) : 0.0;
            const Vector3d e_ref_doppler =
                ref_doppler_ok ? Vector3d((ref_sd.sat_pos - rover_pos).normalized())
                               : Vector3d::Zero();
            const double ref_doppler_variance =
                ref_doppler_ok ? doppler_variance(ref_sd.elevation) : 0.0;
            const auto& ref_indices = (freq == 0) ? filter_state_.n1_indices :
                                      (freq == 1) ? filter_state_.n2_indices :
                                                    filter_state_.n5_indices;
            auto ref_state_it = ref_indices.find(ref_sat);
            if (ref_state_it == ref_indices.end()) {
                blocks.push_back(std::move(phase_block));
                blocks.push_back(std::move(code_block));
                return;
            }

            const double ref_wavelength = freq_wavelength_local(ref_sd, freq);
            if (ref_wavelength <= 0.0) {
                blocks.push_back(std::move(phase_block));
                blocks.push_back(std::move(code_block));
                return;
            }
            const int ref_iono_idx =
                estimate_iono ? II(ref_sat) : -1;
            const double ref_iono_scale =
                estimate_iono
                    ? ionoFrequencyScale(
                          freq,
                          ref_sd.l1_frequency_hz,
                          freq_frequency_hz_local(ref_sd, freq))
                    : 0.0;
            if (estimate_iono && filter_state_.covariance(ref_iono_idx, ref_iono_idx) <= 0.0) {
                blocks.push_back(std::move(phase_block));
                blocks.push_back(std::move(code_block));
                return;
            }
            const double ref_snr = signal_snr_dbhz(ref_sd, freq);
            const double ref_nlos_factor = nlos_variance_factor(ref_sat);
            const double ref_phase_variance = varerr(ref_sd.elevation, true, ref_snr) * ref_nlos_factor;
            const double ref_code_variance = varerr(ref_sd.elevation, false, ref_snr) * ref_nlos_factor;

            for (const auto& pair : system_pairs) {
                if (pair.freq != freq) continue;
                const auto sat_it = sat_data.find(pair.sat);
                if (sat_it == sat_data.end()) continue;
                const auto& sat = pair.sat;
                const auto& sd = sat_it->second;
                const auto& sat_indices = (freq == 0) ? filter_state_.n1_indices :
                                          (freq == 1) ? filter_state_.n2_indices :
                                                        filter_state_.n5_indices;
                auto sat_state_it = sat_indices.find(sat);
                if (sat_state_it == sat_indices.end()) continue;

                const double sat_wavelength = freq_wavelength_local(sd, freq);
                if (sat_wavelength <= 0.0) continue;
                const double sat_snr = signal_snr_dbhz(sd, freq);
                const double sat_nlos_factor = nlos_variance_factor(sat);
                double sat_phase_variance = varerr(sd.elevation, true, sat_snr) * sat_nlos_factor;
                double sat_code_variance = varerr(sd.elevation, false, sat_snr) * sat_nlos_factor;
                // navi.776 A2: replace the satellite-side model variance with
                // the innovation-adapted one. The reference-side variance
                // stays at the model value so the DD block structure
                // (ref_var*11' + diag) keeps its known correlated part.
                const int adaptive_key = freq * MAXSAT + satelliteSlot(sat);
                const double sat_phase_model_variance = sat_phase_variance;
                const double sat_code_model_variance = sat_code_variance;
                if (rtk_config_.enable_adaptive_measurement_noise) {
                    const auto adaptive_config = adaptiveNoiseConfig();
                    sat_phase_variance = adaptive_noise_tracker_.adaptedVariance(
                        adaptive_key, rtk_measurement::MeasurementKind::PHASE,
                        sat_phase_model_variance, adaptive_config);
                    sat_code_variance = adaptive_noise_tracker_.adaptedVariance(
                        adaptive_key, rtk_measurement::MeasurementKind::CODE,
                        sat_code_model_variance, adaptive_config);
                }
                const int sat_iono_idx = estimate_iono ? II(sat) : -1;
                const double sat_iono_scale =
                    estimate_iono
                        ? ionoFrequencyScale(
                              freq,
                              sd.l1_frequency_hz,
                              freq_frequency_hz_local(sd, freq))
                        : 0.0;
                if (estimate_iono &&
                    filter_state_.covariance(sat_iono_idx, sat_iono_idx) <= 0.0) {
                    continue;
                }

                const double rr = geodist_range(sd.sat_pos, rover_pos) + tropModel(rover_pos, sd.elevation);
                const double br = geodist_range(sd.sat_pos_base, base_position_) +
                                  tropModel(base_position_, sd.base_elevation);
                const double geom_dd = (rr_ref - br_ref) - (rr - br);
                const Vector3d dd_los = -los_ref + (sd.sat_pos - rover_pos).normalized();
                const double ref_iono_state =
                    estimate_iono ? filter_state_.state(ref_iono_idx) : 0.0;
                const double sat_iono_state =
                    estimate_iono ? filter_state_.state(sat_iono_idx) : 0.0;
                const double ref_phase = freq_phase_diff_local(ref_sd, freq);
                const double sat_phase = freq_phase_diff_local(sd, freq);
                const double ref_code = freq_code_diff_local(ref_sd, freq);
                const double sat_code = freq_code_diff_local(sd, freq);
                // Glonass autocal/ICB only meaningful on L1/L2 (no GLO L5 path); freq==2 → false.
                const bool autocal_glonass =
                    usesGlonassAutocal(rtk_config_) &&
                    ref_sd.satellite.system == GNSSSystem::GLONASS &&
                    sd.satellite.system == GNSSSystem::GLONASS &&
                    freq < GLO_HWBIAS_STATES;
                const double df_mhz =
                    autocal_glonass
                        ? (freq_frequency_hz_local(ref_sd, freq) -
                           freq_frequency_hz_local(sd, freq)) / 1e6
                        : 0.0;
                const double glonass_icb =
                    glonassInterChannelBiasMeters(
                        rtk_config_,
                        ref_sd.satellite.system,
                        sd.satellite.system,
                        freq_frequency_hz_local(ref_sd, freq),
                        freq_frequency_hz_local(sd, freq),
                        freq);
                const double phase_iono_term =
                    estimate_iono ?
                        (-ref_iono_scale * ref_iono_state + sat_iono_scale * sat_iono_state) :
                        0.0;
                const double code_iono_term =
                    estimate_iono ?
                        (ref_iono_scale * ref_iono_state - sat_iono_scale * sat_iono_state) :
                        0.0;

                rtk_measurement::MeasurementRow phase_row;
                const double amb_term =
                    ref_wavelength * filter_state_.state(ref_state_it->second) -
                    sat_wavelength * filter_state_.state(sat_state_it->second);
                phase_row.residual = ref_phase * ref_wavelength - sat_phase * sat_wavelength -
                                     geom_dd - amb_term - glonass_icb - phase_iono_term;
                if (autocal_glonass) {
                    phase_row.residual -= df_mhz * filter_state_.state(IL(freq));
                    phase_row.state_coefficients.push_back({IL(freq), df_mhz});
                }
                phase_row.state_coefficients.push_back({ref_state_it->second, ref_wavelength});
                phase_row.state_coefficients.push_back({sat_state_it->second, -sat_wavelength});
                if (estimate_iono) {
                    phase_row.state_coefficients.push_back({ref_iono_idx, -ref_iono_scale});
                    phase_row.state_coefficients.push_back({sat_iono_idx, sat_iono_scale});
                }
                phase_row.baseline_coefficients = dd_los;
                phase_row.reference_variance = ref_phase_variance;
                phase_row.satellite_variance = sat_phase_variance;
                phase_row.adaptive_key = adaptive_key;
                phase_row.adaptive_model_variance = sat_phase_model_variance;
                phase_block.rows.push_back(std::move(phase_row));

                rtk_measurement::MeasurementRow code_row;
                code_row.residual = (ref_code - sat_code) - geom_dd - code_iono_term;
                if (estimate_iono) {
                    code_row.state_coefficients.push_back({ref_iono_idx, ref_iono_scale});
                    code_row.state_coefficients.push_back({sat_iono_idx, -sat_iono_scale});
                }
                code_row.baseline_coefficients = dd_los;
                code_row.reference_variance = ref_code_variance;
                code_row.satellite_variance = sat_code_variance;
                code_row.adaptive_key = adaptive_key;
                code_row.adaptive_model_variance = sat_code_model_variance;
                code_block.rows.push_back(std::move(code_row));

                // navi.776 B2: SD Doppler row for this satellite pair.
                if (ref_doppler_ok && freq_has_doppler_local(sd, freq) &&
                    sd.has_sat_velocity &&
                    freq_frequency_hz_local(sd, freq) > 0.0) {
                    const double rr_obs_sat =
                        -freq_rover_doppler_local(sd, freq) *
                        (constants::SPEED_OF_LIGHT / freq_frequency_hz_local(sd, freq));
                    const double rr_pred_sat = predicted_range_rate_mps(sd);
                    const Vector3d e_sat = (sd.sat_pos - rover_pos).normalized();

                    rtk_measurement::MeasurementRow doppler_row;
                    doppler_row.residual =
                        (rr_obs_sat - rr_pred_sat) - (rr_obs_ref - rr_pred_ref);
                    // d(h)/d(v_rx) = e_ref - e_sat (h = rr_sat - rr_ref,
                    // d(rr)/d(v_rx) = -e). Position/ambiguity/iono columns
                    // stay zero: baseline_coefficients untouched.
                    for (int axis = 0; axis < 3; ++axis) {
                        doppler_row.state_coefficients.push_back(
                            {VELOCITY_STATE_INDEX + axis,
                             e_ref_doppler(axis) - e_sat(axis)});
                    }
                    doppler_row.reference_variance = ref_doppler_variance;
                    double sat_doppler_variance = doppler_variance(sd.elevation);
                    const double sat_doppler_model_variance = sat_doppler_variance;
                    if (rtk_config_.enable_adaptive_measurement_noise) {
                        sat_doppler_variance = adaptive_noise_tracker_.adaptedVariance(
                            adaptive_key, rtk_measurement::MeasurementKind::DOPPLER,
                            sat_doppler_model_variance, adaptiveNoiseConfig());
                    }
                    doppler_row.satellite_variance = sat_doppler_variance;
                    doppler_row.adaptive_key = adaptive_key;
                    doppler_row.adaptive_model_variance = sat_doppler_model_variance;
                    doppler_block.rows.push_back(std::move(doppler_row));
                }
            }
            blocks.push_back(std::move(phase_block));
            blocks.push_back(std::move(code_block));
            if (!doppler_block.rows.empty()) {
                blocks.push_back(std::move(doppler_block));
            }
        };

        append_frequency_blocks(0);
        append_frequency_blocks(1);
        if (rtk_config_.enable_l5) {
            append_frequency_blocks(2);  // Phase 18 Step 4: L5 phase + code DD measurement rows
        }
    }

    return blocks;
}

bool RTKProcessor::updateFilter(const std::map<SatelliteId, SatelliteData>& sat_data) {
    if (sat_data.size() < 4) return false;

    const auto blocks = buildMeasurementBlocks(sat_data);
    const auto measurement_diagnostics = rtk_measurement::summarizeMeasurementBlocks(blocks);
    auto measurement_system = rtk_measurement::assembleMeasurementSystem(
        blocks, filter_state_.state.size());

    // navi.776 B2: Doppler rows live in the m/s domain -- give them their
    // own outlier threshold instead of the metre-domain scalar.
    debug_telemetry_.float_update_doppler_observation_count =
        measurement_diagnostics.doppler_observation_count;
    if (measurement_diagnostics.doppler_observation_count > 0) {
        measurement_system.row_outlier_thresholds.assign(
            static_cast<size_t>(measurement_system.residuals.size()), 0.0);
        double doppler_residual_sum_sq = 0.0;
        int row_index = 0;
        for (const auto& block : blocks) {
            for (const auto& row : block.rows) {
                if (block.kind == rtk_measurement::MeasurementKind::DOPPLER) {
                    measurement_system.row_outlier_thresholds[static_cast<size_t>(row_index)] =
                        rtk_config_.doppler_row_outlier_threshold_mps;
                    doppler_residual_sum_sq += row.residual * row.residual;
                }
                ++row_index;
            }
        }
        debug_telemetry_.doppler_row_residual_rms_mps = std::sqrt(
            doppler_residual_sum_sq /
            static_cast<double>(measurement_diagnostics.doppler_observation_count));
    }
    std::vector<bool> force_active;
    if (rtk_config_.enable_velocity_states &&
        filter_state_.state.size() >= VELOCITY_STATE_INDEX + VELOCITY_STATES) {
        force_active.assign(filter_state_.state.size(), false);
        for (int i = 0; i < VELOCITY_STATES; ++i) {
            force_active[VELOCITY_STATE_INDEX + i] = true;
        }
    }
    // navi.776 C: position before the measurement update, for the Kalman
    // position-correction statistic driving the offline time-offset search.
    const Vector3d position_before_update = filter_state_.state.head<3>();

    const auto update_result = rtk_update::applyMeasurementUpdate(filter_state_.state,
                                                                  filter_state_.covariance,
                                                                  measurement_system,
                                                                  rtk_config_.outlier_threshold > 0.0
                                                                      ? rtk_config_.outlier_threshold
                                                                      : 30.0,
                                                                  6,
                                                                  rtk_config_.max_update_nis_per_observation,
                                                                  force_active,
                                                                  rtk_config_.enable_adaptive_measurement_noise);

    if (update_result.ok) {
        const double correction_norm_m =
            (filter_state_.state.head<3>() - position_before_update).norm();
        debug_telemetry_.position_correction_norm_m = correction_norm_m;
        if (ins_time_update_applied_last_epoch_) {
            ++position_correction_count_;
            position_correction_sum_sq_m2_ += correction_norm_m * correction_norm_m;
        }
    }

    // navi.776 A2: feed this epoch's innovations back into the adaptive
    // noise tracker (consumed by the NEXT epoch's buildMeasurementBlocks —
    // the paper's R_{k+1} recursion). Never learn from rejected or failed
    // updates, and skip rows zeroed by suppressOutlierRows.
    if (rtk_config_.enable_adaptive_measurement_noise && update_result.ok &&
        !update_result.rejected_by_innovation_gate &&
        update_result.row_innovations.size() > 0 &&
        update_result.row_hph_diagonal.size() == update_result.row_innovations.size()) {
        const auto adaptive_config = adaptiveNoiseConfig();
        const double tow = current_epoch_time_.tow;
        int row_index = 0;
        for (const auto& block : blocks) {
            for (const auto& row : block.rows) {
                if (row_index >= update_result.row_innovations.size()) break;
                const bool suppressed =
                    measurement_system.design_matrix.row(row_index).isZero(0.0);
                if (row.adaptive_key >= 0 && !suppressed) {
                    adaptive_noise_tracker_.update(
                        row.adaptive_key,
                        block.kind,
                        update_result.row_innovations(row_index),
                        update_result.row_hph_diagonal(row_index),
                        row.reference_variance,
                        row.adaptive_model_variance,
                        tow,
                        adaptive_config);
                }
                ++row_index;
            }
        }
        adaptive_noise_tracker_.pruneStale(tow, rtk_config_.adaptive_noise_reset_gap_s);
        debug_telemetry_.adaptive_noise_tracked_entries =
            static_cast<int>(adaptive_noise_tracker_.size());
        debug_telemetry_.adaptive_noise_mean_phase_scale =
            adaptive_noise_tracker_.meanVarianceScale(
                rtk_measurement::MeasurementKind::PHASE);
        debug_telemetry_.adaptive_noise_mean_code_scale =
            adaptive_noise_tracker_.meanVarianceScale(
                rtk_measurement::MeasurementKind::CODE);
    }
    current_update_diagnostics_.observation_count = update_result.observation_count;
    current_update_diagnostics_.phase_observation_count =
        measurement_diagnostics.phase_observation_count;
    current_update_diagnostics_.code_observation_count =
        measurement_diagnostics.code_observation_count;
    current_update_diagnostics_.suppressed_outliers += update_result.suppressed_outliers;
    current_update_diagnostics_.prefit_residual_rms_m = update_result.prefit_residual_rms_m;
    current_update_diagnostics_.prefit_residual_max_m =
        std::max(current_update_diagnostics_.prefit_residual_max_m,
                 update_result.prefit_residual_max_abs_m);
    current_update_diagnostics_.post_suppression_residual_rms_m =
        update_result.post_suppression_residual_rms_m;
    current_update_diagnostics_.post_suppression_residual_max_m =
        std::max(current_update_diagnostics_.post_suppression_residual_max_m,
                 update_result.post_suppression_residual_max_abs_m);
    current_update_diagnostics_.normalized_innovation_squared =
        update_result.normalized_innovation_squared;
    current_update_diagnostics_.normalized_innovation_squared_per_observation =
        update_result.normalized_innovation_squared_per_observation;
    current_update_diagnostics_.rejected_by_innovation_gate =
        update_result.rejected_by_innovation_gate;

    // WP8 canyon forensics: mirror into the public per-epoch debug
    // telemetry (see EpochDebugTelemetry's float_update_* fields).
    debug_telemetry_.float_update_observation_count = update_result.observation_count;
    debug_telemetry_.float_update_prefit_residual_rms_m = update_result.prefit_residual_rms_m;
    debug_telemetry_.float_update_post_suppression_residual_rms_m =
        update_result.post_suppression_residual_rms_m;
    debug_telemetry_.float_update_nis_per_observation =
        update_result.normalized_innovation_squared_per_observation;
    debug_telemetry_.float_update_suppressed_outliers = update_result.suppressed_outliers;
    debug_telemetry_.float_position_covariance_trace_m2 =
        filter_state_.covariance.rows() >= 3 && filter_state_.covariance.cols() >= 3
            ? filter_state_.covariance(0, 0) + filter_state_.covariance(1, 1) +
                  filter_state_.covariance(2, 2)
            : std::numeric_limits<double>::quiet_NaN();

    return update_result.ok;
}

// ============================================================
// Resolve ambiguities: SD->DD transform + LAMBDA
// ============================================================
bool RTKProcessor::resolveAmbiguities() {
    if (!filter_initialized_) {
        debug_telemetry_.ar_skip_reason = ARSkipReason::FILTER_NOT_INIT;
        return false;
    }
    if (usesEstimatedIono(rtk_config_)) {
        debug_telemetry_.ar_skip_reason = ARSkipReason::ESTIMATED_IONO_MODE;
        return false;
    }

    const auto& sat_data = current_sat_data_;

    const int min_lock = std::max(1, rtk_config_.min_lock_count);
    return resolveAmbiguities(buildDoubleDifferencePairs(sat_data, min_lock));
}

bool RTKProcessor::resolveAmbiguities(std::vector<DDPair> dd_pairs) {
    if (!filter_initialized_) {
        debug_telemetry_.ar_skip_reason = ARSkipReason::FILTER_NOT_INIT;
        return false;
    }
    if (usesEstimatedIono(rtk_config_)) {
        debug_telemetry_.ar_skip_reason = ARSkipReason::ESTIMATED_IONO_MODE;
        return false;
    }

    const auto& sat_data = current_sat_data_;
    debug_telemetry_.ar_attempted = true;
    debug_telemetry_.input_pair_count = static_cast<int>(dd_pairs.size());

    const int na = usesGlonassAutocal(rtk_config_) ? REAL_STATES : BASE_STATES;

    dd_pairs.erase(
        std::remove_if(dd_pairs.begin(), dd_pairs.end(),
                       [&](const DDPair& pair) {
                           return !isAmbiguityResolutionSystem(rtk_config_, pair.ref_sat.system);
                       }),
        dd_pairs.end());

    int nb = dd_pairs.size();
    debug_telemetry_.pair_count = nb;
    if (nb < 4) {
        debug_telemetry_.reject_reason = "too_few_pairs";
        debug_telemetry_.ar_skip_reason = ARSkipReason::DD_PAIRS_LT_4_BEFORE_VAR_FILTER;
        return false;
    }

    // WP10 (WP8 recommendation 2): --nlos-min-los-sats AR-acceptance gate.
    // Gates AR only -- it never touches buildSelectionSnapshot()/
    // buildMeasurementBlocks(), so the float-KF update for this epoch is
    // completely unaffected either way. No-op unless nlos_min_los_sats > 0
    // and a weight table is loaded (mirrors WP8 EXCLUDE's own absent-flag
    // guard style).
    if (rtk_config_.nlos_min_los_sats > 0 && nlos_weight_table_ &&
        !nlos_weight_table_->empty()) {
        std::set<SatelliteId> candidate_sats;
        for (const auto& pair : dd_pairs) {
            candidate_sats.insert(pair.ref_sat);
            candidate_sats.insert(pair.sat);
        }
        int los_count = 0;
        for (const auto& sat : candidate_sats) {
            const double los_prob = nlos_weights::lookupLosProb(
                *nlos_weight_table_, current_epoch_time_.tow, sat.toString(),
                rtk_config_.nlos_tow_tolerance_s);
            if (los_prob >= 0.5) ++los_count;
        }
        if (!nlos_weights::nlosMinLosSatsGateAllows(los_count, rtk_config_.nlos_min_los_sats)) {
            debug_telemetry_.reject_reason = "too_few_los_sats";
            debug_telemetry_.ar_skip_reason = ARSkipReason::TOO_FEW_LOS_SATS;
            return false;
        }
    }

    std::vector<rtk_measurement::AmbiguityDifference> differences;
    differences.reserve(nb);
    for (const auto& pair : dd_pairs) {
        differences.push_back({pair.ref_idx, pair.sat_idx});
    }

    const auto ambiguity_transform = rtk_measurement::buildAmbiguityTransform(
        filter_state_.state, filter_state_.covariance, na, differences);
    VectorXd head_state = ambiguity_transform.head_state;
    VectorXd dd_float = ambiguity_transform.dd_float;
    MatrixXd Qb = ambiguity_transform.ambiguity_covariance;
    MatrixXd Qab = ambiguity_transform.head_ambiguity_covariance;

    Qb = (Qb + Qb.transpose()) / 2.0;
    for (int i = 0; i < nb; ++i)
        if (Qb(i, i) < 1e-6) Qb(i, i) = 1e-6;

    // Variance check
    double max_var = 0;
    for (int i = 0; i < nb; ++i) max_var = std::max(max_var, Qb(i, i));
    debug_telemetry_.max_ambiguity_variance = max_var;

    // Exclude DD pairs with outlier variance (relative to median)
    // This removes newly-appearing satellites that haven't converged
    {
        std::vector<rtk_ar_selection::PairDescriptor> descriptors;
        descriptors.reserve(nb);
        for (int i = 0; i < nb; ++i) {
            descriptors.push_back({dd_pairs[i].ref_sat.system, Qb(i, i)});
        }
        std::vector<int> good_pairs =
            rtk_ar_selection::filterPairsByRelativeVariance(descriptors);

        if (!good_pairs.empty() && (int)good_pairs.size() < nb) {
            int nb_new = good_pairs.size();
            VectorXd new_dd_float(nb_new);
            MatrixXd new_Qb(nb_new, nb_new);
            MatrixXd new_Qab(na, nb_new);
            std::vector<DDPair> new_dd_pairs;
            for (int i = 0; i < nb_new; ++i) {
                new_dd_float(i) = dd_float(good_pairs[i]);
                new_dd_pairs.push_back(dd_pairs[good_pairs[i]]);
                for (int j = 0; j < nb_new; ++j)
                    new_Qb(i, j) = Qb(good_pairs[i], good_pairs[j]);
                for (int j = 0; j < na; ++j)
                    new_Qab(j, i) = Qab(j, good_pairs[i]);
            }
            new_Qb = (new_Qb + new_Qb.transpose()) / 2.0;

            dd_float = new_dd_float;
            Qb = new_Qb;
            Qab = new_Qab;
            dd_pairs.clear();
            dd_pairs = new_dd_pairs;
            nb = nb_new;
            max_var = 0;
            for (int i = 0; i < nb; ++i) max_var = std::max(max_var, Qb(i, i));
            debug_telemetry_.pair_count = nb;
            debug_telemetry_.max_ambiguity_variance = max_var;
            if (nb < 4) {
                // Diagnostic-only: mark root cause; do NOT early-return.
                // Preserve original solver flow so LAMBDA still attempts AR
                // (and fails the usual way). LAMBDA_FAILED assignment below
                // is guarded so this more-specific reason is not overwritten.
                debug_telemetry_.ar_skip_reason = ARSkipReason::DD_PAIRS_LT_4_AFTER_VAR_FILTER;
            }
        }
    }

    // Try full set first
    VectorXd dd_fixed;
    double ratio = 0.0;
    bool fixed = false;

    // Use lower ratio threshold when holdamb is active (more confidence in solution)
    double effective_ratio_threshold = rtk_config_.ambiguity_ratio_threshold;
    if (rtk_config_.enable_satellite_count_ratio_threshold) {
        std::set<SatelliteId> ratio_satellites;
        for (const auto& pair : dd_pairs) {
            ratio_satellites.insert(pair.ref_sat);
            ratio_satellites.insert(pair.sat);
        }
        const int satellite_count = static_cast<int>(ratio_satellites.size());
        debug_telemetry_.ratio_satellite_count = satellite_count;
        if (satellite_count >= 20) {
            effective_ratio_threshold = 1.5;
        } else if (satellite_count >= 15) {
            effective_ratio_threshold = 2.0;
        } else if (satellite_count >= 10) {
            effective_ratio_threshold = 2.5;
        } else {
            effective_ratio_threshold = 3.0;
        }
    }
    if (rtk_config_.ar_policy != RTKConfig::ARPolicy::DEMO5_CONTINUOUS) {
        if (consecutive_fix_count_ >= rtk_config_.min_hold_count && has_last_fixed_position_) {
            // WP7 dead-knob fix: honor the configured hold-ambiguity ratio
            // threshold (--hold-ratio-threshold) instead of a hardcoded 2.0.
            // Default hold_ambiguity_ratio_threshold is 2.0 (rtk.hpp), so this
            // is bit-identical unless a caller explicitly overrides the flag.
            effective_ratio_threshold =
                std::isfinite(rtk_config_.hold_ambiguity_ratio_threshold) &&
                rtk_config_.hold_ambiguity_ratio_threshold > 0.0
                    ? rtk_config_.hold_ambiguity_ratio_threshold
                    : 2.0;
        }
    }
    debug_telemetry_.effective_ratio_threshold = effective_ratio_threshold;
    const int min_subset_pairs_for_ar = std::max(4, rtk_config_.min_subset_pairs_for_ar);
    debug_telemetry_.min_subset_pair_count = min_subset_pairs_for_ar;
    const double min_full_ratio_for_subset_ar =
        std::isfinite(rtk_config_.min_full_ratio_for_subset_ar)
            ? std::max(0.0, rtk_config_.min_full_ratio_for_subset_ar)
            : 0.0;
    debug_telemetry_.min_full_ratio_for_subset_ar = min_full_ratio_for_subset_ar;

    struct SubsetDiversity {
        int distinct_sats = 0;
        int distinct_systems = 0;
        int distinct_frequencies = 0;
        int dual_frequency_sats = 0;
    };
    auto compute_subset_diversity = [&](const std::vector<int>& subset) {
        SubsetDiversity diversity;
        std::set<SatelliteId> sats;
        std::set<GNSSSystem> systems;
        std::set<int> frequencies;
        std::map<SatelliteId, std::set<int>> frequencies_by_sat;
        for (int index : subset) {
            if (index < 0 || index >= static_cast<int>(dd_pairs.size())) {
                continue;
            }
            const auto& pair = dd_pairs[index];
            sats.insert(pair.sat);
            systems.insert(pair.sat.system);
            frequencies.insert(pair.freq);
            frequencies_by_sat[pair.sat].insert(pair.freq);
        }
        for (const auto& [sat, sat_frequencies] : frequencies_by_sat) {
            (void)sat;
            if (sat_frequencies.size() >= 2) {
                diversity.dual_frequency_sats++;
            }
        }
        diversity.distinct_sats = static_cast<int>(sats.size());
        diversity.distinct_systems = static_cast<int>(systems.size());
        diversity.distinct_frequencies = static_cast<int>(frequencies.size());
        return diversity;
    };
    auto passes_subset_diversity_gate = [&](const SubsetDiversity& diversity) {
        if (rtk_config_.min_subset_sats_for_ar > 0 &&
            diversity.distinct_sats < rtk_config_.min_subset_sats_for_ar) {
            return false;
        }
        if (rtk_config_.min_subset_systems_for_ar > 0 &&
            diversity.distinct_systems < rtk_config_.min_subset_systems_for_ar) {
            return false;
        }
        if (rtk_config_.min_subset_frequencies_for_ar > 0 &&
            diversity.distinct_frequencies < rtk_config_.min_subset_frequencies_for_ar) {
            return false;
        }
        if (rtk_config_.min_subset_dual_frequency_sats_for_ar > 0 &&
            diversity.dual_frequency_sats < rtk_config_.min_subset_dual_frequency_sats_for_ar) {
            return false;
        }
        return true;
    };

    // === Wide-lane AR pre-step (default-off) ===
    // Frozen copies of full-size matrices for use in build_search_problem
    // (best_candidate may update dd_float/Qb/Qab to subset-sized values later)
    const VectorXd base_dd_float = dd_float;
    const MatrixXd base_Qb = Qb;
    const MatrixXd base_Qab = Qab;
    const VectorXd base_head_state = head_state;

    std::vector<int> full_subset(nb);
    for (int i = 0; i < nb; ++i) {
        full_subset[i] = i;
    }
    std::vector<int> initial_candidate_subset = full_subset;

    struct WideLaneConstraint {
        int l1_index = -1;
        int l2_index = -1;
        double fixed_integer = 0.0;
    };
    std::vector<WideLaneConstraint> wide_lane_constraints;
    int wide_lane_total = 0;
    int wide_lane_fixed = 0;
    int wide_lane_rejected = 0;
    double wide_lane_min_distance = std::numeric_limits<double>::infinity();
    double wide_lane_max_distance = 0.0;

    auto compute_wide_lane_float = [&](int l1_index, int l2_index, double& wide_lane_float) {
        if (l1_index < 0 || l2_index < 0 ||
            l1_index >= nb || l2_index >= nb ||
            dd_pairs[l1_index].freq != 0 || dd_pairs[l2_index].freq != 1) {
            return false;
        }

        const auto ref_it = sat_data.find(dd_pairs[l1_index].ref_sat);
        const auto sat_it = sat_data.find(dd_pairs[l1_index].sat);
        if (ref_it == sat_data.end() || sat_it == sat_data.end()) {
            return false;
        }
        const auto& ref_sd = ref_it->second;
        const auto& sd = sat_it->second;
        if (!ref_sd.has_l1 || !ref_sd.has_l2 || !sd.has_l1 || !sd.has_l2) {
            return false;
        }

        const double f1 = ref_sd.l1_frequency_hz;
        const double f2 = ref_sd.l2_frequency_hz;
        const double lambda_wl_m = wideLaneWavelength(f1, f2);
        if (f1 <= 0.0 || f2 <= 0.0 || lambda_wl_m <= 0.0) {
            return false;
        }

        auto single_difference_wide_lane = [&](const SatelliteData& data) {
            const double phi1_m =
                (data.rover_l1_phase - data.base_l1_phase) * data.l1_wavelength;
            const double phi2_m =
                (data.rover_l2_phase - data.base_l2_phase) * data.l2_wavelength;
            const double code_term =
                (f1 * (data.rover_l1_code - data.base_l1_code) +
                 f2 * (data.rover_l2_code - data.base_l2_code)) / (f1 + f2);
            return ((f1 * phi1_m - f2 * phi2_m) / (f1 - f2) - code_term) / lambda_wl_m;
        };

        wide_lane_float = single_difference_wide_lane(ref_sd) -
                          single_difference_wide_lane(sd);
        return std::isfinite(wide_lane_float);
    };

    // Phase 18 Step 6: L1-L5 wide-lane Melbourne-Wübbena combination.
    // Parallel to the L1-L2 path; uses the dd_pairs entry with freq==2 as the L5 leg.
    auto compute_wide_lane_l5_float = [&](int l1_index, int l5_index, double& wide_lane_float) {
        if (l1_index < 0 || l5_index < 0 ||
            l1_index >= nb || l5_index >= nb ||
            dd_pairs[l1_index].freq != 0 || dd_pairs[l5_index].freq != 2) {
            return false;
        }
        const auto ref_it = sat_data.find(dd_pairs[l1_index].ref_sat);
        const auto sat_it = sat_data.find(dd_pairs[l1_index].sat);
        if (ref_it == sat_data.end() || sat_it == sat_data.end()) {
            return false;
        }
        const auto& ref_sd = ref_it->second;
        const auto& sd = sat_it->second;
        if (!ref_sd.has_l1 || !ref_sd.has_l5 || !sd.has_l1 || !sd.has_l5) {
            return false;
        }

        const double f1 = ref_sd.l1_frequency_hz;
        const double f5 = ref_sd.l5_frequency_hz;
        const double lambda_wl_m = wideLaneWavelength(f1, f5);  // ~0.751 m for GPS L1-L5
        if (f1 <= 0.0 || f5 <= 0.0 || lambda_wl_m <= 0.0) {
            return false;
        }

        auto single_difference_wide_lane_l5 = [&](const SatelliteData& data) {
            const double phi1_m =
                (data.rover_l1_phase - data.base_l1_phase) * data.l1_wavelength;
            const double phi5_m =
                (data.rover_l5_phase - data.base_l5_phase) * data.l5_wavelength;
            const double code_term =
                (f1 * (data.rover_l1_code - data.base_l1_code) +
                 f5 * (data.rover_l5_code - data.base_l5_code)) / (f1 + f5);
            return ((f1 * phi1_m - f5 * phi5_m) / (f1 - f5) - code_term) / lambda_wl_m;
        };

        wide_lane_float = single_difference_wide_lane_l5(ref_sd) -
                          single_difference_wide_lane_l5(sd);
        return std::isfinite(wide_lane_float);
    };

    if (rtk_config_.enable_wide_lane_ar) {
        const double wide_lane_threshold =
            std::max(0.0, rtk_config_.wide_lane_acceptance_threshold);
        for (int i = 0; i < nb; ++i) {
            if (dd_pairs[i].freq != 0 || dd_pairs[i].ref_sat.system == GNSSSystem::GLONASS) {
                continue;
            }
            int l2_pair = -1;
            for (int j = 0; j < nb; ++j) {
                if (dd_pairs[j].freq == 1 &&
                    dd_pairs[j].sat == dd_pairs[i].sat &&
                    dd_pairs[j].ref_sat == dd_pairs[i].ref_sat) {
                    l2_pair = j;
                    break;
                }
            }
            if (l2_pair < 0) {
                continue;
            }

            wide_lane_total++;
            double wide_lane_float = 0.0;
            if (!compute_wide_lane_float(i, l2_pair, wide_lane_float)) {
                continue;
            }
            const double fixed_integer = std::round(wide_lane_float);
            const double wl_distance = distanceToNearestInteger(wide_lane_float);
            wide_lane_min_distance = std::min(wide_lane_min_distance, wl_distance);
            wide_lane_max_distance = std::max(wide_lane_max_distance, wl_distance);
            if (wl_distance >= wide_lane_threshold) {
                wide_lane_rejected++;
                continue;
            }

            wide_lane_constraints.push_back({i, l2_pair, fixed_integer});
            wide_lane_fixed++;
        }
        // Phase 18 Step 6: parallel L1-L5 wide-lane fixing for sats lacking an L1-L2
        // pair (or in addition to it). Constraint structure is identical — l2_index
        // simply now points at a freq=2 dd_pair instead of freq=1.
        if (rtk_config_.enable_l5) {
            for (int i = 0; i < nb; ++i) {
                if (dd_pairs[i].freq != 0 || dd_pairs[i].ref_sat.system == GNSSSystem::GLONASS) {
                    continue;
                }
                int l5_pair = -1;
                for (int j = 0; j < nb; ++j) {
                    if (dd_pairs[j].freq == 2 &&
                        dd_pairs[j].sat == dd_pairs[i].sat &&
                        dd_pairs[j].ref_sat == dd_pairs[i].ref_sat) {
                        l5_pair = j;
                        break;
                    }
                }
                if (l5_pair < 0) {
                    continue;
                }

                wide_lane_total++;
                double wide_lane_float = 0.0;
                if (!compute_wide_lane_l5_float(i, l5_pair, wide_lane_float)) {
                    continue;
                }
                const double fixed_integer = std::round(wide_lane_float);
                const double wl_distance = distanceToNearestInteger(wide_lane_float);
                wide_lane_min_distance = std::min(wide_lane_min_distance, wl_distance);
                wide_lane_max_distance = std::max(wide_lane_max_distance, wl_distance);
                if (wl_distance >= wide_lane_threshold) {
                    wide_lane_rejected++;
                    continue;
                }

                wide_lane_constraints.push_back({i, l5_pair, fixed_integer});
                wide_lane_fixed++;
            }
        }
        if (wide_lane_total > 0) {
            std::clog << "[RTK-AR] WL fixed " << wide_lane_fixed
                      << "/" << wide_lane_total << "\n";
        }
    }
    debug_telemetry_.wide_lane_total = wide_lane_total;
    debug_telemetry_.wide_lane_fixed = wide_lane_fixed;
    debug_telemetry_.wide_lane_rejected = wide_lane_rejected;
    if (std::isfinite(wide_lane_min_distance)) {
        debug_telemetry_.wide_lane_min_distance = wide_lane_min_distance;
        debug_telemetry_.wide_lane_max_distance = wide_lane_max_distance;
    }

    struct SearchProblem {
        VectorXd head_state;
        VectorXd dd_float;
        MatrixXd Qb;
        MatrixXd Qab;
    };
    auto build_search_problem = [&](const std::vector<int>& subset) {
        SearchProblem problem;
        problem.head_state = base_head_state;
        if (subset.size() == static_cast<size_t>(nb)) {
            problem.dd_float = base_dd_float;
            problem.Qb = base_Qb;
            problem.Qab = base_Qab;
        } else {
            const auto subset_matrices =
                rtk_ar_evaluation::extractSubset(base_dd_float, base_Qb, base_Qab, subset);
            problem.dd_float = subset_matrices.dd_float;
            problem.Qb = subset_matrices.Qb;
            problem.Qab = subset_matrices.Qab;
        }

        if (!wide_lane_constraints.empty()) {
            std::map<int, int> local_index_by_full_index;
            for (int local = 0; local < static_cast<int>(subset.size()); ++local) {
                local_index_by_full_index[subset[local]] = local;
            }

            for (const auto& constraint : wide_lane_constraints) {
                const auto l1_it = local_index_by_full_index.find(constraint.l1_index);
                const auto l2_it = local_index_by_full_index.find(constraint.l2_index);
                if (l1_it == local_index_by_full_index.end() ||
                    l2_it == local_index_by_full_index.end()) {
                    continue;
                }
                applyAmbiguityConstraintUpdate(problem.head_state,
                                               problem.dd_float,
                                               problem.Qb,
                                               problem.Qab,
                                               l1_it->second,
                                               l2_it->second,
                                               constraint.fixed_integer,
                                               1e-4);
            }
        }

        if (wide_lane_constraints.empty()) {
            return problem;
        }

        problem.Qb = (problem.Qb + problem.Qb.transpose()) * 0.5;
        for (int i = 0; i < problem.Qb.rows(); ++i) {
            if (problem.Qb(i, i) < 1e-6) {
                problem.Qb(i, i) = 1e-6;
            }
        }
        return problem;
    };

    // Standard LAMBDA path
    if (!fixed) {
        const auto full_problem = build_search_problem(full_subset);
        const bool full_solved =
            lambdaMethod(full_problem.dd_float, full_problem.Qb, dd_fixed, ratio);
        debug_telemetry_.full_lambda_solved = full_solved;
        if (full_solved) {
            debug_telemetry_.full_ratio = ratio;
            // WP7 dead-knob fix: passesArFilter is a no-op AND term when
            // enable_ar_filter is false (its default), so this preserves the
            // exact base ratio >= threshold gate unless --arfilter is set.
            if (ratio >= effective_ratio_threshold &&
                rtk_ar_evaluation::passesArFilter(
                    rtk_config_.enable_ar_filter, ratio, effective_ratio_threshold,
                    rtk_config_.ar_filter_margin)) {
                fixed = true;

                // WL-NL cross-validation: verify LAMBDA integers match WL-NL
                if (fixed && rtk_config_.ionoopt == RTKConfig::IonoOpt::IFLC) {
                    int mismatch = 0, checked = 0;
                    for (int i = 0; i < nb; ++i) {
                        if (dd_pairs[i].freq != 0 || dd_pairs[i].ref_sat.system == GNSSSystem::GLONASS) continue;
                        int l2p = -1;
                        for (int j = 0; j < nb; ++j)
                            if (dd_pairs[j].freq == 1 &&
                                dd_pairs[j].sat == dd_pairs[i].sat &&
                                dd_pairs[j].ref_sat == dd_pairs[i].ref_sat) { l2p = j; break; }
                        if (l2p < 0) continue;
                        // LAMBDA N_wl vs MW N_wl
                        double lambda_nwl = dd_fixed(i) - dd_fixed(l2p);

                        auto rit = sat_data.find(dd_pairs[i].ref_sat);
                        auto sit = sat_data.find(dd_pairs[i].sat);
                        if (rit == sat_data.end() || sit == sat_data.end()) continue;
                        if (!rit->second.has_l2 || !sit->second.has_l2) continue;
                        const double f1 = rit->second.l1_frequency_hz;
                        const double f2 = rit->second.l2_frequency_hz;
                        const double lam_wl = wideLaneWavelength(f1, f2);
                        const double c1 = ionoFreeCoeff1(f1, f2);
                        const double c2 = ionoFreeCoeff2(f1, f2);
                        const double lam_nl = narrowLaneWavelength(f1, f2);
                        if (lam_wl <= 0.0 || lam_nl <= 0.0) continue;
                        auto mw_sd = [&](const SatelliteData& d) {
                            return (d.rover_l1_phase - d.base_l1_phase) - (d.rover_l2_phase - d.base_l2_phase)
                                 - (f1 * (d.rover_l1_code - d.base_l1_code) +
                                    f2 * (d.rover_l2_code - d.base_l2_code))
                                   / ((f1 + f2) * lam_wl);
                        };
                        double dd_mw = mw_sd(rit->second) - mw_sd(sit->second);
                        double mw_nwl = std::round(dd_mw);
                        if (std::abs(dd_mw - mw_nwl) < 0.25) {
                            checked++;
                            // Check WL consistency
                            if (std::abs(lambda_nwl - mw_nwl) > 0.5) mismatch++;
                            // Also check NL: use IF to derive expected N2
                            double if_val = c1 * rit->second.l1_wavelength * dd_float(i) +
                                            c2 * rit->second.l2_wavelength * dd_float(l2p);
                            double n2_if = (if_val - c1 * rit->second.l1_wavelength * mw_nwl) / lam_nl;
                            double n2_lambda = dd_fixed(l2p);
                            // If IF-derived N2 differs from LAMBDA N2, it's wrong
                            if (std::abs(n2_if - n2_lambda) > 0.5) mismatch++;
                        }
                    }
                    // If majority of checked satellites have WL mismatch, reject LAMBDA fix
                    if (checked >= 3 && mismatch > checked / 2) {
                        fixed = false;
                    }
                }
            }
        }
    }

    // WL-NL fallback: when LAMBDA fails on long baseline, try MW wide-lane
    // followed by a narrow-lane integer check. Historically this path was
    // tied to IFLC; enable_wlnl_fallback lets iono-off runs test it without
    // also forcing the wide-lane constraint pre-step.
    if (!fixed &&
        (rtk_config_.ionoopt == RTKConfig::IonoOpt::IFLC ||
         rtk_config_.enable_wlnl_fallback) &&
        max_var < 1.0) {
        // Only attempt when KF has converged (max_var < 1 = several epochs in)
        VectorXd wlnl_fixed = dd_float;
        int wl_ok = 0, wl_total = 0;
        std::set<int> wlnl_fixed_indices;
        const double wlnl_acceptance_threshold =
            std::max(0.0, rtk_config_.wide_lane_acceptance_threshold);

        for (int i = 0; i < nb; ++i) {
            if (dd_pairs[i].freq != 0 || dd_pairs[i].ref_sat.system == GNSSSystem::GLONASS) continue;
            wl_total++;
            int l2p = -1;
            for (int j = 0; j < nb; ++j)
                if (dd_pairs[j].freq == 1 &&
                    dd_pairs[j].sat == dd_pairs[i].sat &&
                    dd_pairs[j].ref_sat == dd_pairs[i].ref_sat) { l2p = j; break; }
            if (l2p < 0) continue;

            auto rit = sat_data.find(dd_pairs[i].ref_sat);
            auto sit = sat_data.find(dd_pairs[i].sat);
            if (rit == sat_data.end() || sit == sat_data.end()) continue;
            if (!rit->second.has_l2 || !sit->second.has_l2) continue;
            const double f1 = rit->second.l1_frequency_hz;
            const double f2 = rit->second.l2_frequency_hz;
            const double lam_wl = wideLaneWavelength(f1, f2);
            const double c1_if = ionoFreeCoeff1(f1, f2);
            const double c2_if = ionoFreeCoeff2(f1, f2);
            const double lam_nl = narrowLaneWavelength(f1, f2);
            if (lam_wl <= 0.0 || lam_nl <= 0.0) continue;

            // MW wide-lane
            auto mw_sd = [&](const SatelliteData& d) {
                return (d.rover_l1_phase - d.base_l1_phase) - (d.rover_l2_phase - d.base_l2_phase)
                     - (f1 * (d.rover_l1_code - d.base_l1_code) +
                        f2 * (d.rover_l2_code - d.base_l2_code))
                       / ((f1 + f2) * lam_wl);
            };
            double dd_mw = mw_sd(rit->second) - mw_sd(sit->second);
            double nw = std::round(dd_mw);
            if (std::abs(dd_mw - nw) > wlnl_acceptance_threshold) continue;

            // IF → NL
            double if_dd = c1_if * rit->second.l1_wavelength * dd_float(i) +
                           c2_if * rit->second.l2_wavelength * dd_float(l2p);
            double n2f = (if_dd - c1_if * rit->second.l1_wavelength * nw) / lam_nl;
            double n2 = std::round(n2f);
            if (std::abs(n2f - n2) > wlnl_acceptance_threshold) continue;

            wlnl_fixed(i) = nw + n2;
            wlnl_fixed(l2p) = n2;
            wlnl_fixed_indices.insert(i);
            wlnl_fixed_indices.insert(l2p);
            wl_ok++;
        }

        if (wl_ok >= 3) {
            // Use resolved pairs for position
            std::vector<int> resolved(
                wlnl_fixed_indices.begin(), wlnl_fixed_indices.end());
            if ((int)resolved.size() >= 4) {
                int ns = resolved.size();
                VectorXd sf(ns), sx(ns);
                MatrixXd sQb(ns, ns), sQab(na, ns);
                for (int i = 0; i < ns; ++i) {
                    sf(i) = dd_float(resolved[i]);
                    sx(i) = wlnl_fixed(resolved[i]);
                    for (int j = 0; j < ns; ++j) sQb(i,j) = Qb(resolved[i], resolved[j]);
                    for (int j = 0; j < na; ++j) sQab(j,i) = Qab(j, resolved[i]);
                }
                sQb = (sQb + sQb.transpose()) / 2.0;
                for (int i = 0; i < ns; ++i) if (sQb(i,i) < 1e-6) sQb(i,i) = 1e-6;
                Eigen::LDLT<MatrixXd> slv(sQb);
                if (slv.info() == Eigen::Success) {
                    VectorXd xa = head_state - sQab * slv.solve(sf - sx);
                    fixed_baseline_ = xa.head<3>();
                    has_fixed_solution_ = true;
                    dd_float = sf;
                    Qb = sQb;
                    Qab = sQab;
                    dd_fixed = sx;
                    fixed = true;
                    ratio = 999.9;
                    debug_telemetry_.used_wlnl_fallback = true;
                    initial_candidate_subset = resolved;
                    last_dd_pairs_ = dd_pairs;
                    last_best_subset_ = resolved;
                    last_dd_fixed_ = dd_fixed;
                }
            }
        }
    }

    // Partial AR: try removing worst satellites if full set fails
    rtk_ar_evaluation::CandidateState best_candidate;
    best_candidate.fixed = fixed;
    best_candidate.ratio = ratio;
    best_candidate.subset = initial_candidate_subset;
    best_candidate.dd_float = dd_float;
    best_candidate.Qb = Qb;
    best_candidate.Qab = Qab;
    best_candidate.dd_fixed = dd_fixed;

    const bool search_preferred_subsets =
        rtk_config_.ar_policy != RTKConfig::ARPolicy::DEMO5_CONTINUOUS &&
        rtk_ar_evaluation::shouldSearchPreferredSubsets(
            fixed, ratio, effective_ratio_threshold);
    const bool search_drop_subsets =
        rtk_config_.ar_policy != RTKConfig::ARPolicy::DEMO5_CONTINUOUS &&
        rtk_ar_evaluation::shouldSearchDropSubsets(
            fixed, ratio, effective_ratio_threshold, max_var);

    if (nb > min_subset_pairs_for_ar && (search_preferred_subsets || search_drop_subsets)) {
        auto passes_full_ratio_gate = [&]() {
            if (min_full_ratio_for_subset_ar <= 0.0) {
                return true;
            }
            return debug_telemetry_.full_lambda_solved &&
                std::isfinite(debug_telemetry_.full_ratio) &&
                debug_telemetry_.full_ratio >= min_full_ratio_for_subset_ar;
        };

        auto try_subset = [&](const std::vector<int>& subset) {
            const int ns = subset.size();
            if (ns < min_subset_pairs_for_ar) {
                return false;
            }
            const auto subset_diversity = compute_subset_diversity(subset);
            debug_telemetry_.subset_candidates_evaluated++;
            if (!passes_subset_diversity_gate(subset_diversity)) {
                debug_telemetry_.subset_candidates_rejected_by_diversity++;
                return false;
            }
            if (!passes_full_ratio_gate()) {
                debug_telemetry_.subset_candidates_rejected_by_full_ratio++;
                return false;
            }

            const auto subset_problem = build_search_problem(subset);
            VectorXd sub_fixed;
            double sub_ratio = 0.0;
            if (!lambdaMethod(subset_problem.dd_float, subset_problem.Qb, sub_fixed, sub_ratio)) {
                return false;
            }
            if (sub_ratio < effective_ratio_threshold) {
                return false;
            }
            if (!rtk_ar_evaluation::passesArFilter(
                    rtk_config_.enable_ar_filter, sub_ratio, effective_ratio_threshold,
                    rtk_config_.ar_filter_margin)) {
                return false;
            }

            if (rtk_ar_evaluation::preferCandidate(
                    best_candidate.ratio, best_candidate.fixed, sub_ratio)) {
                rtk_ar_evaluation::SubsetMatrices subset_matrices;
                subset_matrices.dd_float = subset_problem.dd_float;
                subset_matrices.Qb = subset_problem.Qb;
                subset_matrices.Qab = subset_problem.Qab;
                rtk_ar_evaluation::adoptCandidate(
                    best_candidate, subset, subset_matrices, sub_fixed, sub_ratio);
                return true;
            }
            return false;
        };

        std::vector<rtk_ar_selection::PairDescriptor> descriptors;
        descriptors.reserve(nb);
        for (int i = 0; i < nb; ++i) {
            descriptors.push_back({dd_pairs[i].ref_sat.system, Qb(i, i)});
        }

        const std::vector<std::vector<int>> preferred_subsets =
            rtk_config_.enable_paper_constellation_fallback_ar
                ? rtk_ar_selection::buildPaperConstellationFallbackSubsets(descriptors)
                : rtk_ar_selection::buildPreferredSubsets(descriptors);
        const bool compare_preferred_subsets =
            rtk_config_.enable_paper_constellation_fallback_ar ||
            usesGlonassAutocal(rtk_config_) ||
            std::any_of(dd_pairs.begin(), dd_pairs.end(), [](const DDPair& pair) {
                return pair.ref_sat.system == GNSSSystem::GLONASS ||
                       pair.ref_sat.system == GNSSSystem::BeiDou;
            });
        if (compare_preferred_subsets && search_preferred_subsets) {
            for (const auto& subset : preferred_subsets) {
                if (subset.size() < static_cast<size_t>(min_subset_pairs_for_ar) ||
                    subset.size() >= static_cast<size_t>(nb)) {
                    continue;
                }
                try_subset(subset);
            }
        }

        if (search_drop_subsets) {
            // A full-set fix already passed the Ratio/AR-filter gates. Keep the
            // legacy six-step refinement ceiling in that case so an opt-in
            // deeper search cannot replace a healthy solution with a much
            // smaller, higher-Ratio subset and perturb subsequent filter
            // states. Deeper Partial AR remains available when the full set
            // failed, which is the recovery case the wider search targets.
            const int max_progressive_drop_steps =
                rtk_ar_evaluation::progressiveDropStepLimit(
                    rtk_config_.max_subset_drop_steps_for_ar, fixed);
            const auto progressive_subsets =
                rtk_ar_selection::buildProgressiveVarianceDropSubsets(
                    descriptors,
                    min_subset_pairs_for_ar,
                    max_progressive_drop_steps);
            for (const auto& subset : progressive_subsets) {
                try_subset(subset);
            }

            if (rtk_config_.enable_bsr_guided_decimation &&
                Qb.rows() == nb && Qb.cols() == nb) {
                const int max_drop = std::max(
                    0, std::min(rtk_config_.bsr_guided_max_drop_steps,
                                nb - min_subset_pairs_for_ar));
                const int worst_axes = std::max(1, rtk_config_.bsr_guided_worst_axes);
                const auto bsr_subsets =
                    rtk_ar_selection::buildBSRGuidedDropSubsets(
                        descriptors, Qb, min_subset_pairs_for_ar,
                        max_drop, worst_axes);
                for (const auto& subset : bsr_subsets) {
                    debug_telemetry_.bsr_guided_candidates_evaluated++;
                    if (try_subset(subset)) {
                        debug_telemetry_.bsr_guided_candidates_accepted++;
                    }
                }
            }
        }

        if (best_candidate.fixed) {
            fixed = true;
            dd_fixed = best_candidate.dd_fixed;
            ratio = best_candidate.ratio;
            Qab = best_candidate.Qab;
            Qb = best_candidate.Qb;
            dd_float = best_candidate.dd_float;
        } else if (!fixed && best_candidate.subset.size() < static_cast<size_t>(nb)) {
            Qab = best_candidate.Qab;
            Qb = best_candidate.Qb;
            dd_float = best_candidate.dd_float;
        }
    }

    debug_telemetry_.selected_ratio = ratio;
    debug_telemetry_.selected_pair_count = static_cast<int>(best_candidate.subset.size());
    const auto selected_diversity = compute_subset_diversity(best_candidate.subset);
    debug_telemetry_.selected_distinct_sats = selected_diversity.distinct_sats;
    debug_telemetry_.selected_distinct_systems = selected_diversity.distinct_systems;
    debug_telemetry_.selected_distinct_frequencies = selected_diversity.distinct_frequencies;
    debug_telemetry_.selected_dual_frequency_sats = selected_diversity.dual_frequency_sats;
    debug_telemetry_.selected_used_subset = best_candidate.subset.size() < static_cast<size_t>(nb);
    std::set<SatelliteId> selected_references;
    for (const int index : best_candidate.subset) {
        if (index >= 0 && index < static_cast<int>(dd_pairs.size())) {
            selected_references.insert(dd_pairs[static_cast<size_t>(index)].ref_sat);
        }
    }
    for (const auto& reference : selected_references) {
        if (!debug_telemetry_.selected_reference_satellites.empty()) {
            debug_telemetry_.selected_reference_satellites += ';';
        }
        debug_telemetry_.selected_reference_satellites += reference.toString();
    }
    if (!fixed) {
        debug_telemetry_.selected_fixed = false;
        if (debug_telemetry_.reject_reason.empty()) {
            debug_telemetry_.reject_reason = "lambda_not_fixed";
        }
        // First-cause wins: a more specific skip reason set earlier
        // (e.g. DD_PAIRS_LT_4_AFTER_VAR_FILTER) should not be overwritten.
        if (debug_telemetry_.ar_skip_reason == ARSkipReason::NONE) {
            debug_telemetry_.ar_skip_reason = ARSkipReason::LAMBDA_FAILED;
        }
        return false;
    }
    debug_telemetry_.selected_fixed = true;

    // Fixed solution: xa = y[:na] - Qab * Qb^{-1} * (dd_float - dd_fixed)
    const auto fixed_problem = build_search_problem(best_candidate.subset);
    VectorXd xa;
    if (!rtk_ar_evaluation::solveFixedHeadState(fixed_problem.head_state,
                                                fixed_problem.Qab,
                                                fixed_problem.Qb,
                                                fixed_problem.dd_float,
                                                dd_fixed, xa)) {
        debug_telemetry_.reject_reason = "fixed_head_solve";
        if (debug_telemetry_.ar_skip_reason == ARSkipReason::NONE) {
            debug_telemetry_.ar_skip_reason = ARSkipReason::RATIO_COMPUTATION_FAILED;
        }
        return false;
    }
    fixed_baseline_ = xa.head<3>();
    has_fixed_solution_ = true;
    last_ar_ratio_ = ratio;
    last_num_fixed_ambiguities_ = dd_fixed.size();
    // Clear any diagnostic-only skip reason set earlier (e.g. DD_PAIRS_LT_4_AFTER_VAR_FILTER
    // at line 1803, which marks the cause without early-returning so LAMBDA can still try).
    debug_telemetry_.ar_skip_reason = ARSkipReason::NONE;
    debug_telemetry_.selected_fixed_ambiguities = static_cast<int>(dd_fixed.size());

    // Store fix info for hold and validation
    last_dd_pairs_ = dd_pairs;
    last_best_subset_ = best_candidate.subset;
    last_dd_fixed_ = dd_fixed;

    return true;
}

// ============================================================
// Validate fixed solution
// ============================================================
bool RTKProcessor::validateCpPrFixedCandidate(
    const std::map<SatelliteId, SatelliteData>& sat_data,
    const GNSSTime& current_time) {
    if (!rtk_config_.enable_cp_pr_fixed_gate) {
        consecutive_cp_pr_gate_rejections_ = 0;
        return true;
    }

    auto wavelength = [](const SatelliteData& data, int freq) {
        return freq == 0 ? data.l1_wavelength
             : freq == 1 ? data.l2_wavelength
                         : data.l5_wavelength;
    };
    auto phaseSd = [](const SatelliteData& data, int freq) {
        return freq == 0 ? data.rover_l1_phase - data.base_l1_phase
             : freq == 1 ? data.rover_l2_phase - data.base_l2_phase
                         : data.rover_l5_phase - data.base_l5_phase;
    };
    auto codeSd = [](const SatelliteData& data, int freq) {
        return freq == 0 ? data.rover_l1_code - data.base_l1_code
             : freq == 1 ? data.rover_l2_code - data.base_l2_code
                         : data.rover_l5_code - data.base_l5_code;
    };

    std::vector<rtk_cp_pr_gate::Observation> gate_observations;
    if (last_best_subset_.size() == static_cast<std::size_t>(last_dd_fixed_.size())) {
        gate_observations.reserve(last_best_subset_.size());
        for (int i = 0; i < static_cast<int>(last_best_subset_.size()); ++i) {
            const int pair_index = last_best_subset_[i];
            if (pair_index < 0 || pair_index >= static_cast<int>(last_dd_pairs_.size())) {
                continue;
            }
            const auto& pair = last_dd_pairs_[pair_index];
            if (pair.ref_sat.system == GNSSSystem::GLONASS) {
                continue;
            }
            const auto ref_it = sat_data.find(pair.ref_sat);
            const auto sat_it = sat_data.find(pair.sat);
            if (ref_it == sat_data.end() || sat_it == sat_data.end()) {
                continue;
            }
            const double ref_wavelength = wavelength(ref_it->second, pair.freq);
            const double sat_wavelength = wavelength(sat_it->second, pair.freq);
            if (!(ref_wavelength > 0.0) || !(sat_wavelength > 0.0) ||
                std::abs(ref_wavelength - sat_wavelength) > 1e-6) {
                continue;
            }
            rtk_cp_pr_gate::Observation observation;
            observation.dd_pseudorange_m =
                codeSd(ref_it->second, pair.freq) - codeSd(sat_it->second, pair.freq);
            observation.dd_carrier_m =
                ref_wavelength * phaseSd(ref_it->second, pair.freq) -
                sat_wavelength * phaseSd(sat_it->second, pair.freq);
            observation.fixed_ambiguity_m = ref_wavelength * last_dd_fixed_(i);
            gate_observations.push_back(observation);
        }
    }

    rtk_cp_pr_gate::Config gate_config;
    gate_config.innovation_threshold_m = rtk_config_.cp_pr_fixed_gate_threshold_m;
    gate_config.min_pairs = static_cast<std::size_t>(
        std::max(1, rtk_config_.cp_pr_fixed_gate_min_pairs));
    gate_config.max_bad_pairs = static_cast<std::size_t>(
        std::max(0, rtk_config_.cp_pr_fixed_gate_max_bad_pairs));
    gate_config.escalation_epochs = static_cast<std::size_t>(
        std::max(1, rtk_config_.cp_pr_fixed_gate_escalation_epochs));
    const auto gate_result = rtk_cp_pr_gate::evaluate(gate_observations, gate_config);
    if (!gate_result.valid) {
        consecutive_cp_pr_gate_rejections_ = 0;
        return true;
    }

    debug_telemetry_.cp_pr_gate_evaluated = true;
    debug_telemetry_.cp_pr_gate_checked_pairs =
        static_cast<int>(gate_result.checked_pairs);
    debug_telemetry_.cp_pr_gate_bad_pairs = static_cast<int>(gate_result.bad_pairs);
    debug_telemetry_.cp_pr_gate_rms_m = gate_result.rms_innovation_m;
    debug_telemetry_.cp_pr_gate_max_m = gate_result.max_abs_innovation_m;
    if (gate_result.consistent) {
        consecutive_cp_pr_gate_rejections_ = 0;
        return true;
    }

    debug_telemetry_.cp_pr_gate_rejected = true;
    ++consecutive_cp_pr_gate_rejections_;
    const bool escalated = consecutive_cp_pr_gate_rejections_ >=
        std::max(1, rtk_config_.cp_pr_fixed_gate_escalation_epochs);
    debug_telemetry_.cp_pr_gate_escalated = escalated;
    if (escalated) {
        std::set<std::pair<SatelliteId, SatelliteId>> used_pairs;
        std::vector<rtk_ddpr_anchor::Observation> anchor_observations;
        for (const auto& pair : last_dd_pairs_) {
            if (pair.freq != 0 || !used_pairs.insert({pair.ref_sat, pair.sat}).second) {
                continue;
            }
            const auto ref_it = sat_data.find(pair.ref_sat);
            const auto sat_it = sat_data.find(pair.sat);
            if (ref_it == sat_data.end() || sat_it == sat_data.end()) {
                continue;
            }
            rtk_ddpr_anchor::Observation observation;
            observation.reference_satellite_rover_ecef = ref_it->second.sat_pos;
            observation.target_satellite_rover_ecef = sat_it->second.sat_pos;
            observation.reference_satellite_base_ecef = ref_it->second.sat_pos_base;
            observation.target_satellite_base_ecef = sat_it->second.sat_pos_base;
            observation.dd_pseudorange_m =
                codeSd(ref_it->second, 0) - codeSd(sat_it->second, 0);
            anchor_observations.push_back(observation);
        }
        rtk_ddpr_anchor::Config anchor_config;
        anchor_config.fde_threshold_m = rtk_config_.ddpr_anchor_fde_threshold_m;
        anchor_config.max_fde_removals = static_cast<std::size_t>(
            std::max(0, rtk_config_.ddpr_anchor_max_fde_removals));
        const Vector3d initial_position = base_position_ + filter_state_.state.head<3>();
        const auto anchor_result = rtk_ddpr_anchor::solve(
            anchor_observations, base_position_, initial_position, anchor_config);
        if (anchor_result.valid) {
            last_ddpr_anchor_position_ecef_ = anchor_result.position_ecef;
            last_ddpr_anchor_covariance_ecef_ = anchor_result.covariance_ecef;
            last_ddpr_anchor_time_ = current_time;
            has_last_ddpr_anchor_ = true;
            debug_telemetry_.ddpr_anchor_valid = true;
            debug_telemetry_.ddpr_anchor_observations =
                static_cast<int>(anchor_result.observations_used);
            debug_telemetry_.ddpr_anchor_residual_rms_m = anchor_result.residual_rms_m;
            debug_telemetry_.ddpr_anchor_fixed_distance_m =
                (anchor_result.position_ecef - (base_position_ + fixed_baseline_)).norm();
        }
    }
    debug_telemetry_.reject_reason = "cp_pr_innovation";
    return false;
}

RTKProcessor::HoldStateSnapshot RTKProcessor::captureHoldState() const {
    HoldStateSnapshot snapshot;
    snapshot.last_fixed_position = last_fixed_position_;
    snapshot.has_last_fixed_position = has_last_fixed_position_;
    snapshot.last_fixed_time = last_fixed_time_;
    snapshot.has_last_fixed_time = has_last_fixed_time_;
    snapshot.dd_pairs = last_dd_pairs_;
    snapshot.best_subset = last_best_subset_;
    snapshot.dd_fixed = last_dd_fixed_;
    snapshot.ar_ratio = last_ar_ratio_;
    snapshot.num_fixed_ambiguities = last_num_fixed_ambiguities_;
    return snapshot;
}

void RTKProcessor::restoreHoldState(const HoldStateSnapshot& snapshot) {
    last_fixed_position_ = snapshot.last_fixed_position;
    has_last_fixed_position_ = snapshot.has_last_fixed_position;
    last_fixed_time_ = snapshot.last_fixed_time;
    has_last_fixed_time_ = snapshot.has_last_fixed_time;
    last_dd_pairs_ = snapshot.dd_pairs;
    last_best_subset_ = snapshot.best_subset;
    last_dd_fixed_ = snapshot.dd_fixed;
    last_ar_ratio_ = snapshot.ar_ratio;
    last_num_fixed_ambiguities_ = snapshot.num_fixed_ambiguities;
}

bool RTKProcessor::validateFixedSolution(const std::map<SatelliteId, SatelliteData>& sat_data,
                                         const GNSSTime& current_time) {
    if (!has_fixed_solution_) {
        debug_telemetry_.reject_reason = "no_fixed_solution";
        return false;
    }

    const Vector3d new_pos = base_position_ + fixed_baseline_;
    debug_telemetry_.fixed_candidate_position_ecef = new_pos;
    debug_telemetry_.fixed_candidate_position_valid = new_pos.allFinite();
    if (filter_state_.state.size() >= 3) {
        const Vector3d float_pos = base_position_ + filter_state_.state.head<3>();
        debug_telemetry_.fixed_candidate_float_separation_m =
            (new_pos - float_pos).norm();
    }
    if (has_last_fixed_position_) {
        debug_telemetry_.fixed_candidate_history_jump_m =
            (new_pos - last_fixed_position_).norm();
    }
    if (has_last_fixed_time_) {
        debug_telemetry_.fixed_candidate_history_dt_s = current_time - last_fixed_time_;
    }

    const bool low_ratio_guard_enabled =
        std::isfinite(rtk_config_.low_ratio_guard_threshold) &&
        rtk_config_.low_ratio_guard_threshold > 0.0 &&
        rtk_config_.low_ratio_min_fixed_ambiguities > 0;
    const bool low_count_rescue_enabled =
        std::isfinite(rtk_config_.low_count_rescue_ratio_threshold) &&
        rtk_config_.low_count_rescue_ratio_threshold > 0.0 &&
        rtk_config_.low_count_rescue_min_fixed_ambiguities > 0 &&
        std::isfinite(rtk_config_.low_count_rescue_max_history_speed_mps) &&
        rtk_config_.low_count_rescue_max_history_speed_mps > 0.0;
    const bool low_count_candidate =
        low_ratio_guard_enabled && std::isfinite(last_ar_ratio_) &&
        last_ar_ratio_ < rtk_config_.low_ratio_guard_threshold &&
        last_num_fixed_ambiguities_ < rtk_config_.low_ratio_min_fixed_ambiguities;
    bool low_count_rescue_pass = false;
    if (low_count_candidate && low_count_rescue_enabled) {
        debug_telemetry_.low_count_rescue_evaluated = true;
        const double history_dt = debug_telemetry_.fixed_candidate_history_dt_s;
        const double history_jump = debug_telemetry_.fixed_candidate_history_jump_m;
        const double history_speed =
            (std::isfinite(history_dt) && history_dt > 0.0 &&
             std::isfinite(history_jump))
                ? history_jump / history_dt
                : std::numeric_limits<double>::infinity();
        low_count_rescue_pass =
            last_num_fixed_ambiguities_ >=
                rtk_config_.low_count_rescue_min_fixed_ambiguities &&
            last_ar_ratio_ >= rtk_config_.low_count_rescue_ratio_threshold &&
            history_speed <= rtk_config_.low_count_rescue_max_history_speed_mps;
        debug_telemetry_.low_count_rescue_passed = low_count_rescue_pass;
    }
    if (low_ratio_guard_enabled &&
        (!std::isfinite(last_ar_ratio_) ||
         (low_count_candidate && !low_count_rescue_pass))) {
        debug_telemetry_.reject_reason = "low_ratio_weak_integer_system";
        return false;
    }

    if (!validateCpPrFixedCandidate(sat_data, current_time)) {
        return false;
    }

    const double fixed_candidate_baseline_m = fixed_baseline_.norm();
    const auto window_enabled = [](double max_ratio,
                                   double min_baseline,
                                   double max_baseline,
                                   double min_speed,
                                   double max_speed) {
        return (std::isfinite(max_ratio) && max_ratio > 0.0) ||
               (std::isfinite(min_baseline) && min_baseline > 0.0) ||
               (std::isfinite(max_baseline) && max_baseline > 0.0) ||
               (std::isfinite(min_speed) && min_speed > 0.0) ||
               (std::isfinite(max_speed) && max_speed > 0.0);
    };
    const bool primary_window_enabled = window_enabled(
        rtk_config_.max_fixed_update_gate_ratio,
        rtk_config_.min_fixed_update_gate_baseline_m,
        rtk_config_.max_fixed_update_gate_baseline_m,
        rtk_config_.min_fixed_update_gate_speed_mps,
        rtk_config_.max_fixed_update_gate_speed_mps);
    const bool secondary_window_enabled = window_enabled(
        rtk_config_.max_fixed_update_secondary_gate_ratio,
        rtk_config_.min_fixed_update_secondary_gate_baseline_m,
        rtk_config_.max_fixed_update_secondary_gate_baseline_m,
        rtk_config_.min_fixed_update_secondary_gate_speed_mps,
        rtk_config_.max_fixed_update_secondary_gate_speed_mps);
    const bool any_speed_window_enabled =
        (std::isfinite(rtk_config_.min_fixed_update_gate_speed_mps) &&
         rtk_config_.min_fixed_update_gate_speed_mps > 0.0) ||
        (std::isfinite(rtk_config_.max_fixed_update_gate_speed_mps) &&
         rtk_config_.max_fixed_update_gate_speed_mps > 0.0) ||
        (std::isfinite(rtk_config_.min_fixed_update_secondary_gate_speed_mps) &&
         rtk_config_.min_fixed_update_secondary_gate_speed_mps > 0.0) ||
        (std::isfinite(rtk_config_.max_fixed_update_secondary_gate_speed_mps) &&
         rtk_config_.max_fixed_update_secondary_gate_speed_mps > 0.0);
    double fixed_update_gate_speed_mps = std::numeric_limits<double>::quiet_NaN();
    if (any_speed_window_enabled &&
        has_fixed_update_gate_previous_solution_ &&
        new_pos.allFinite() &&
        fixed_update_gate_previous_position_.allFinite()) {
        const double dt = current_time - fixed_update_gate_previous_time_;
        if (std::isfinite(dt) && dt > 1e-3) {
            fixed_update_gate_speed_mps =
                (new_pos - fixed_update_gate_previous_position_).norm() / dt;
        }
    }
    const auto window_passes = [&](double max_ratio,
                                   double min_baseline,
                                   double max_baseline,
                                   double min_speed,
                                   double max_speed) {
        const bool ratio_passes =
            !std::isfinite(max_ratio) ||
            max_ratio <= 0.0 ||
            (std::isfinite(last_ar_ratio_) && last_ar_ratio_ <= max_ratio);
        const bool baseline_passes =
            std::isfinite(fixed_candidate_baseline_m) &&
            (!std::isfinite(min_baseline) ||
             min_baseline <= 0.0 ||
             fixed_candidate_baseline_m >= min_baseline) &&
            (!std::isfinite(max_baseline) ||
             max_baseline <= 0.0 ||
             fixed_candidate_baseline_m <= max_baseline);
        const bool speed_window_enabled =
            (std::isfinite(min_speed) && min_speed > 0.0) ||
            (std::isfinite(max_speed) && max_speed > 0.0);
        const bool speed_passes =
            !speed_window_enabled ||
            (std::isfinite(fixed_update_gate_speed_mps) &&
             (!std::isfinite(min_speed) ||
              min_speed <= 0.0 ||
              fixed_update_gate_speed_mps >= min_speed) &&
             (!std::isfinite(max_speed) ||
              max_speed <= 0.0 ||
              fixed_update_gate_speed_mps <= max_speed));
        return ratio_passes && baseline_passes && speed_passes;
    };
    const bool fixed_update_gate_window_passes =
        (!primary_window_enabled && !secondary_window_enabled) ||
        (primary_window_enabled &&
         window_passes(rtk_config_.max_fixed_update_gate_ratio,
                       rtk_config_.min_fixed_update_gate_baseline_m,
                       rtk_config_.max_fixed_update_gate_baseline_m,
                       rtk_config_.min_fixed_update_gate_speed_mps,
                       rtk_config_.max_fixed_update_gate_speed_mps)) ||
        (secondary_window_enabled &&
         window_passes(rtk_config_.max_fixed_update_secondary_gate_ratio,
                       rtk_config_.min_fixed_update_secondary_gate_baseline_m,
                       rtk_config_.max_fixed_update_secondary_gate_baseline_m,
                       rtk_config_.min_fixed_update_secondary_gate_speed_mps,
                       rtk_config_.max_fixed_update_secondary_gate_speed_mps));
    if (fixed_update_gate_window_passes) {
        const double max_fixed_update_nis =
            rtk_config_.max_fixed_update_nis_per_observation;
        if (std::isfinite(max_fixed_update_nis) &&
            max_fixed_update_nis > 0.0 &&
            std::isfinite(
                current_update_diagnostics_.normalized_innovation_squared_per_observation) &&
            current_update_diagnostics_.normalized_innovation_squared_per_observation >
                max_fixed_update_nis) {
            debug_telemetry_.reject_reason = "fixed_update_nis";
            return false;
        }
        const double max_fixed_update_post_rms =
            rtk_config_.max_fixed_update_post_residual_rms_m;
        if (std::isfinite(max_fixed_update_post_rms) &&
            max_fixed_update_post_rms > 0.0 &&
            std::isfinite(current_update_diagnostics_.post_suppression_residual_rms_m) &&
            current_update_diagnostics_.post_suppression_residual_rms_m >
                max_fixed_update_post_rms) {
            debug_telemetry_.reject_reason = "fixed_update_post_rms";
            return false;
        }
    }

    // Reject fixes that jump too much from the previous fix position
    // This catches wrong integers that pass the ratio test
    const bool has_fixed_jump_dt = has_last_fixed_time_ &&
        std::isfinite(current_time - last_fixed_time_);
    const double fixed_jump_dt =
        has_fixed_jump_dt ? current_time - last_fixed_time_ : 0.0;
    const bool fixed_anchor_usable = rtk_validation::fixedAnchorUsable(
        has_last_fixed_position_,
        has_last_fixed_time_,
        fixed_jump_dt,
        rtk_config_.max_fixed_anchor_age_s);
    const bool fixed_residual_overconfidence_suspect =
        rtk_config_.max_fixed_prefit_residual_rms_m > 0.0 &&
        rtk_config_.min_fixed_prefit_outliers > 0 &&
        current_update_diagnostics_.prefit_residual_rms_m >
            rtk_config_.max_fixed_prefit_residual_rms_m &&
        current_update_diagnostics_.suppressed_outliers >=
            rtk_config_.min_fixed_prefit_outliers &&
        rtk_config_.max_fixed_overconfidence_covariance_trace_m2 > 0.0 &&
        debug_telemetry_.float_position_covariance_trace_m2 <=
            rtk_config_.max_fixed_overconfidence_covariance_trace_m2;
    const bool require_doppler_consensus =
        !fixed_anchor_usable || fixed_residual_overconfidence_suspect;
    if (!isMovingBasePositionMode(rtk_config_) &&
        rtk_config_.max_fixed_doppler_consensus_m > 0.0 &&
        require_doppler_consensus &&
        has_doppler_continuity_position_) {
        const double consensus_distance =
            (new_pos - doppler_continuity_position_ecef_).norm();
        debug_telemetry_.fixed_candidate_doppler_consensus_distance_m =
            consensus_distance;
        if (!std::isfinite(consensus_distance) ||
            consensus_distance > rtk_config_.max_fixed_doppler_consensus_m) {
            debug_telemetry_.reject_reason = "fixed_doppler_consensus";
            return false;
        }
    }
    const bool use_adaptive_position_jump =
        rtk_config_.max_position_jump_rate_mps > 0.0 &&
        fixed_anchor_usable &&
        has_fixed_jump_dt;
    if (!isMovingBasePositionMode(rtk_config_) &&
        !use_adaptive_position_jump &&
        rtk_validation::exceedsFixHistoryJump(
            new_pos,
            last_fixed_position_,
            fixed_anchor_usable,
            rtk_config_.position_mode == RTKConfig::PositionMode::STATIC,
            consecutive_fix_count_)) {
        debug_telemetry_.reject_reason = "fix_history_jump";
        return false;
    }
    if (!isMovingBasePositionMode(rtk_config_) && fixed_anchor_usable) {
        double position_jump_limit = rtk_config_.max_position_jump_m;
        if (use_adaptive_position_jump) {
            const double adaptive_limit = rtk_validation::adaptiveJumpLimit(
                fixed_jump_dt,
                rtk_config_.max_position_jump_min_m,
                rtk_config_.max_position_jump_rate_mps);
            position_jump_limit = std::max(position_jump_limit, adaptive_limit);
        }
        if (position_jump_limit > 0.0 &&
            rtk_validation::exceedsAbsoluteJump(
                new_pos, last_fixed_position_, true, position_jump_limit)) {
            debug_telemetry_.reject_reason =
                use_adaptive_position_jump ? "adaptive_position_jump" : "max_position_jump";
            return false;
        }
    }

    // Sanity check: reject fixes where any component is unreasonably large
    if (fixed_baseline_.norm() > rtk_config_.max_baseline_length) {
        debug_telemetry_.reject_reason = "max_baseline";
        return false;
    }

    // IFLC: validate fixed L1/L2 DD ambiguities using geometry-free consistency
    // For each satellite pair, check that fixed_N1*λ1 - fixed_N2*λ2 is consistent
    // with the measured geometry-free phase (which contains iono + N1*λ1 - N2*λ2)
    // If the fixed wide-lane (N1-N2) doesn't match the Melbourne-Wubbena estimate,
    // the fix is likely corrupted by ionosphere
    if (rtk_config_.ionoopt == RTKConfig::IonoOpt::IFLC &&
        last_dd_fixed_.size() > 0 && last_dd_pairs_.size() > 0) {
        // Build map: satellite -> (fixed_DD_N1, fixed_DD_N2)
        // from last_dd_pairs_ and last_dd_fixed_
        std::map<SatelliteId, std::pair<SatelliteId, double>> fixed_dd_n1;
        std::map<SatelliteId, std::pair<SatelliteId, double>> fixed_dd_n2;
        for (int i = 0; i < (int)last_best_subset_.size(); ++i) {
            int dd_idx = last_best_subset_[i];
            if (dd_idx >= (int)last_dd_pairs_.size()) continue;
            const auto& pair = last_dd_pairs_[dd_idx];
            if (pair.freq == 0) {
                fixed_dd_n1[pair.sat] = {pair.ref_sat, last_dd_fixed_(i)};
            } else if (pair.freq == 1) {
                fixed_dd_n2[pair.sat] = {pair.ref_sat, last_dd_fixed_(i)};
            }
        }

        // For each satellite with both L1 and L2 fixed DD:
        // Compute fixed wide-lane DD_NW = DD_N1 - DD_N2
        // Compare with Melbourne-Wubbena DD_MW from observations
        int bad_wl_count = 0;
        int checked_count = 0;
        for (const auto& [sat, l1_entry] : fixed_dd_n1) {
            auto n2_it = fixed_dd_n2.find(sat);
            if (n2_it == fixed_dd_n2.end()) continue;
            if (!(n2_it->second.first == l1_entry.first)) continue;
            double dd_n1 = l1_entry.second;
            double dd_n2 = n2_it->second.second;
            double fixed_wl = dd_n1 - dd_n2;  // should be integer

            // Compute DD Melbourne-Wubbena from observations
            auto sat_it = sat_data.find(sat);
            auto ref_it = sat_data.find(l1_entry.first);
            if (sat_it == sat_data.end() || ref_it == sat_data.end()) continue;
            if (!sat_it->second.has_l2 || !ref_it->second.has_l2) continue;

            const auto& ref_sd = ref_it->second;
            const auto& sd = sat_it->second;
            const double f1 = ref_sd.l1_frequency_hz;
            const double f2 = ref_sd.l2_frequency_hz;
            const double lambda_wl_m = wideLaneWavelength(f1, f2);
            if (lambda_wl_m <= 0.0) continue;

            double ref_L1_sd = ref_sd.rover_l1_phase - ref_sd.base_l1_phase;
            double ref_L2_sd = ref_sd.rover_l2_phase - ref_sd.base_l2_phase;
            double ref_P1_sd = ref_sd.rover_l1_code - ref_sd.base_l1_code;
            double ref_P2_sd = ref_sd.rover_l2_code - ref_sd.base_l2_code;
            double ref_MW = (ref_L1_sd - ref_L2_sd)
                          - (f1 * ref_P1_sd + f2 * ref_P2_sd) / (lambda_wl_m * (f1 + f2));

            double sat_L1_sd = sd.rover_l1_phase - sd.base_l1_phase;
            double sat_L2_sd = sd.rover_l2_phase - sd.base_l2_phase;
            double sat_P1_sd = sd.rover_l1_code - sd.base_l1_code;
            double sat_P2_sd = sd.rover_l2_code - sd.base_l2_code;
            double sat_MW = (sat_L1_sd - sat_L2_sd)
                          - (f1 * sat_P1_sd + f2 * sat_P2_sd) / (lambda_wl_m * (f1 + f2));

            double dd_MW = ref_MW - sat_MW;
            double mw_int = std::round(dd_MW);

            // Check: fixed wide-lane should match MW estimate
            // MW noise depends on code multipath (typically 0.1-0.5 WL cycles)
            // Use threshold of 1.0 to avoid false rejections from code noise
            if (std::abs(fixed_wl - mw_int) >= 1.0) {
                bad_wl_count++;
            }
            checked_count++;
        }

        // If more than half of checked satellites have wrong WL, reject the fix
        if (checked_count > 0 && bad_wl_count > checked_count / 2) {
            debug_telemetry_.reject_reason = "iflc_wl_consistency";
            return false;
        }
    }

    auto computePostFixResidualRms = [&]() {
        if (last_dd_fixed_.size() == 0 ||
            last_best_subset_.size() != static_cast<size_t>(last_dd_fixed_.size()) ||
            filter_state_.state.size() < 3) {
            return std::numeric_limits<double>::infinity();
        }

        VectorXd fixed_state = filter_state_.state;
        fixed_state.head<3>() = fixed_baseline_;
        for (int i = 0; i < static_cast<int>(last_best_subset_.size()); ++i) {
            const int dd_idx = last_best_subset_[i];
            if (dd_idx < 0 || dd_idx >= static_cast<int>(last_dd_pairs_.size())) {
                return std::numeric_limits<double>::infinity();
            }
            const auto& pair = last_dd_pairs_[dd_idx];
            if (pair.ref_idx < 0 || pair.sat_idx < 0 ||
                pair.ref_idx >= fixed_state.size() || pair.sat_idx >= fixed_state.size()) {
                return std::numeric_limits<double>::infinity();
            }
            fixed_state(pair.sat_idx) = fixed_state(pair.ref_idx) - last_dd_fixed_(i);
        }

        const Vector3d rover_pos = base_position_ + fixed_baseline_;
        double l1_sum_sq = 0.0;
        int l1_count = 0;
        double all_sum_sq = 0.0;
        int all_count = 0;

        for (int i = 0; i < static_cast<int>(last_best_subset_.size()); ++i) {
            const int dd_idx = last_best_subset_[i];
            if (dd_idx < 0 || dd_idx >= static_cast<int>(last_dd_pairs_.size())) {
                continue;
            }
            const auto& pair = last_dd_pairs_[dd_idx];
            const auto ref_it = sat_data.find(pair.ref_sat);
            const auto sat_it = sat_data.find(pair.sat);
            if (ref_it == sat_data.end() || sat_it == sat_data.end()) {
                continue;
            }

            const auto& ref_sd = ref_it->second;
            const auto& sd = sat_it->second;
            const bool use_l1 = pair.freq == 0;
            if ((use_l1 && (!ref_sd.has_l1 || !sd.has_l1)) ||
                (!use_l1 && (!ref_sd.has_l2 || !sd.has_l2))) {
                continue;
            }

            const double ref_wavelength = use_l1 ? ref_sd.l1_wavelength : ref_sd.l2_wavelength;
            const double sat_wavelength = use_l1 ? sd.l1_wavelength : sd.l2_wavelength;
            if (ref_wavelength <= 0.0 || sat_wavelength <= 0.0 ||
                pair.ref_idx < 0 || pair.sat_idx < 0 ||
                pair.ref_idx >= fixed_state.size() || pair.sat_idx >= fixed_state.size()) {
                continue;
            }

            const double ref_phase = use_l1 ? ref_sd.rover_l1_phase - ref_sd.base_l1_phase
                                            : ref_sd.rover_l2_phase - ref_sd.base_l2_phase;
            const double sat_phase = use_l1 ? sd.rover_l1_phase - sd.base_l1_phase
                                            : sd.rover_l2_phase - sd.base_l2_phase;
            const double rr_ref =
                geodist_range(ref_sd.sat_pos, rover_pos) + tropModel(rover_pos, ref_sd.elevation);
            const double br_ref = geodist_range(ref_sd.sat_pos_base, base_position_) +
                                  tropModel(base_position_, ref_sd.base_elevation);
            const double rr =
                geodist_range(sd.sat_pos, rover_pos) + tropModel(rover_pos, sd.elevation);
            const double br = geodist_range(sd.sat_pos_base, base_position_) +
                              tropModel(base_position_, sd.base_elevation);
            const double geom_dd = (rr_ref - br_ref) - (rr - br);
            const double amb_term =
                ref_wavelength * fixed_state(pair.ref_idx) -
                sat_wavelength * fixed_state(pair.sat_idx);
            const bool autocal_glonass =
                usesGlonassAutocal(rtk_config_) &&
                ref_sd.satellite.system == GNSSSystem::GLONASS &&
                sd.satellite.system == GNSSSystem::GLONASS &&
                pair.freq < GLO_HWBIAS_STATES;
            const double df_mhz =
                autocal_glonass
                    ? (((use_l1 ? ref_sd.l1_frequency_hz : ref_sd.l2_frequency_hz) -
                        (use_l1 ? sd.l1_frequency_hz : sd.l2_frequency_hz)) / 1e6)
                    : 0.0;
            const double glonass_icb =
                glonassInterChannelBiasMeters(
                    rtk_config_,
                    ref_sd.satellite.system,
                    sd.satellite.system,
                    use_l1 ? ref_sd.l1_frequency_hz : ref_sd.l2_frequency_hz,
                    use_l1 ? sd.l1_frequency_hz : sd.l2_frequency_hz,
                    pair.freq);
            const double phase_iono_term =
                usesEstimatedIono(rtk_config_)
                    ? (-ionoFrequencyScale(
                           pair.freq,
                           ref_sd.l1_frequency_hz,
                           use_l1 ? ref_sd.l1_frequency_hz : ref_sd.l2_frequency_hz) *
                           fixed_state(II(pair.ref_sat)) +
                       ionoFrequencyScale(
                           pair.freq,
                           sd.l1_frequency_hz,
                           use_l1 ? sd.l1_frequency_hz : sd.l2_frequency_hz) *
                           fixed_state(II(pair.sat)))
                    : 0.0;

            double residual =
                ref_phase * ref_wavelength - sat_phase * sat_wavelength -
                geom_dd - amb_term - glonass_icb - phase_iono_term;
            if (autocal_glonass) {
                residual -= df_mhz * fixed_state(IL(pair.freq));
            }
            if (!std::isfinite(residual)) {
                continue;
            }

            all_sum_sq += residual * residual;
            all_count++;
            if (use_l1) {
                l1_sum_sq += residual * residual;
                l1_count++;
            }
        }

        if (l1_count > 0) {
            return std::sqrt(l1_sum_sq / static_cast<double>(l1_count));
        }
        if (all_count > 0) {
            return std::sqrt(all_sum_sq / static_cast<double>(all_count));
        }
        return std::numeric_limits<double>::infinity();
    };

    const double postfix_residual_rms =
        (filter_initialized_ && filter_state_.state.size() >= 3)
            ? computePostFixResidualRms()
            : std::numeric_limits<double>::infinity();
    debug_telemetry_.postfix_residual_rms = postfix_residual_rms;

    const double max_postfix_residual_rms = rtk_config_.max_postfix_residual_rms;
    if (filter_initialized_ &&
        filter_state_.state.size() >= 3 &&
        std::isfinite(max_postfix_residual_rms) &&
        max_postfix_residual_rms > 0.0 &&
        postfix_residual_rms > max_postfix_residual_rms) {
        debug_telemetry_.reject_reason = "postfix_rms";
        return false;
    }

    return true;
}

// ============================================================
// Hold ambiguities (RTKLIB holdamb equivalent)
// ============================================================
void RTKProcessor::applyHoldAmbiguity() {
    if (last_dd_fixed_.size() == 0) return;

    int n = filter_state_.state.size();

    auto& x = filter_state_.state;
    auto& P = filter_state_.covariance;

    // Direct state adjustment: move SD ambiguities to satisfy DD integer constraints
    // Only adjust the satellite (non-reference) SD ambiguity for each DD pair
    // This avoids modifying the reference satellite state which is shared across all DDs
    for (int i = 0; i < (int)last_best_subset_.size(); ++i) {
        int dd_idx = last_best_subset_[i];
        if (dd_idx >= (int)last_dd_pairs_.size()) continue;
        if (!usesHoldAmbiguitySystem(rtk_config_, last_dd_pairs_[dd_idx].ref_sat.system)) continue;
        int ri = last_dd_pairs_[dd_idx].ref_idx;
        int si = last_dd_pairs_[dd_idx].sat_idx;
        if (ri >= n || si >= n) continue;
        if (x(ri) == 0.0 || x(si) == 0.0) continue;

        // Set sat SD state so that DD = dd_fixed
        // DD = x[ri] - x[si] = dd_fixed  =>  x[si] = x[ri] - dd_fixed
        double dd_fixed = last_dd_fixed_(i);
        x(si) = x(ri) - dd_fixed;

        // Tighten the satellite ambiguity covariance (leave ref untouched)
        // Reduce cross-correlations with position to prevent position distortion
        constexpr double VAR_HOLDAMB = 0.001;
        for (int j = 0; j < n; ++j) {
            if (j == si) continue;
            P(si, j) = 0.0;
            P(j, si) = 0.0;
        }
        P(si, si) = VAR_HOLDAMB;
    }
}

// ============================================================
// Hold fix: use held DD integers when LAMBDA fails
// ============================================================
bool RTKProcessor::tryHoldFix(const std::map<SatelliteId, SatelliteData>& sat_data,
                               const GNSSTime& time, int n_sats, PositionSolution& solution) {
    debug_telemetry_.hold_fix_attempted = true;
    const auto reject_hold = [&](const char* reason) {
        debug_telemetry_.hold_fix_reject_reason = reason;
        return false;
    };
    if (isMovingBasePositionMode(rtk_config_)) {
        return reject_hold("moving_base");
    }
    if (!rtk_validation::canAttemptHoldFix(consecutive_fix_count_,
                                           rtk_config_.min_hold_count,
                                           has_last_fixed_position_,
                                           last_dd_fixed_.size() > 0)) {
        return reject_hold("preconditions");
    }

    const int na = usesGlonassAutocal(rtk_config_) ? REAL_STATES : BASE_STATES;

    std::vector<DDPair> dd_pairs = buildDoubleDifferencePairs(sat_data, 1);
    dd_pairs.erase(
        std::remove_if(dd_pairs.begin(), dd_pairs.end(),
                       [&](const DDPair& pair) {
                           return !usesHoldAmbiguitySystem(rtk_config_, pair.ref_sat.system);
                       }),
        dd_pairs.end());

    int nb = dd_pairs.size();
    debug_telemetry_.hold_fix_candidate_pairs = nb;
    if (nb < 4) return reject_hold("candidate_pairs_lt4");

    // Match current DD pairs with last held DD pairs
    // Use the held DD integers for matching satellites
    VectorXd dd_fixed(nb);
    int matched = 0;
    for (int i = 0; i < nb; ++i) {
        bool found = false;
        for (int j = 0; j < (int)last_best_subset_.size(); ++j) {
            int dd_idx = last_best_subset_[j];
            if (dd_idx >= (int)last_dd_pairs_.size()) continue;
            if (dd_pairs[i].ref_sat == last_dd_pairs_[dd_idx].ref_sat &&
                dd_pairs[i].sat == last_dd_pairs_[dd_idx].sat &&
                dd_pairs[i].freq == last_dd_pairs_[dd_idx].freq) {
                dd_fixed(i) = last_dd_fixed_(j);
                found = true;
                matched++;
                break;
            }
        }
        if (!found) {
            // For unmatched DD pairs, round the current float DD to nearest integer
            double dd_float = filter_state_.state(dd_pairs[i].ref_idx) -
                            filter_state_.state(dd_pairs[i].sat_idx);
            dd_fixed(i) = std::round(dd_float);
        }
    }

    debug_telemetry_.hold_fix_matched_pairs = matched;
    if (matched < 4) return reject_hold("matched_pairs_lt4");

    std::vector<rtk_measurement::AmbiguityDifference> differences;
    differences.reserve(nb);
    for (const auto& pair : dd_pairs) {
        differences.push_back({pair.ref_idx, pair.sat_idx});
    }

    const auto ambiguity_transform = rtk_measurement::buildAmbiguityTransform(
        filter_state_.state, filter_state_.covariance, na, differences);
    const VectorXd head_state = ambiguity_transform.head_state;
    VectorXd dd_float_v = ambiguity_transform.dd_float;
    MatrixXd Qb = ambiguity_transform.ambiguity_covariance;
    MatrixXd Qab = ambiguity_transform.head_ambiguity_covariance;
    Qb = (Qb + Qb.transpose()) / 2.0;
    for (int i = 0; i < nb; ++i)
        if (Qb(i, i) < 1e-6) Qb(i, i) = 1e-6;
    VectorXd db = dd_float_v - dd_fixed;
    Eigen::LDLT<MatrixXd> Qb_solver(Qb);
    if (Qb_solver.info() != Eigen::Success) return reject_hold("covariance_factorization");
    VectorXd Qb_inv_db = Qb_solver.solve(db);

    VectorXd xa = head_state - Qab * Qb_inv_db;
    Vector3d test_pos = base_position_ + xa.head<3>();

    // Position validation: only accept if close to last fix
    const double max_hold_jump_m =
        (rtk_config_.position_mode == RTKConfig::PositionMode::STATIC) ? 0.1 : 1.0;
    debug_telemetry_.hold_fix_jump_m = (test_pos - last_fixed_position_).norm();
    if (rtk_validation::exceedsAbsoluteJump(
            test_pos, last_fixed_position_, has_last_fixed_position_, max_hold_jump_m)) {
        return reject_hold("position_jump");
    }
    if (filter_state_.state.size() >= 3) {
        const Vector3d float_pos = base_position_ + filter_state_.state.head<3>();
        debug_telemetry_.hold_fix_float_divergence_m = (test_pos - float_pos).norm();
        if (rtk_config_.max_hold_divergence_m > 0.0 &&
            debug_telemetry_.hold_fix_float_divergence_m >
                rtk_config_.max_hold_divergence_m) {
            return reject_hold("float_divergence");
        }
    }

    // Accept hold fix
    fixed_baseline_ = xa.head<3>();
    has_fixed_solution_ = true;
    last_ar_ratio_ = std::max(last_ar_ratio_, rtk_config_.ambiguity_ratio_threshold);
    last_num_fixed_ambiguities_ = matched;
    last_dd_pairs_ = dd_pairs;
    // Rebuild best_subset as all indices
    last_best_subset_.clear();
    for (int i = 0; i < nb; ++i) last_best_subset_.push_back(i);
    last_dd_fixed_ = dd_fixed;

    Vector3d saved_baseline = filter_state_.state.head<3>();
    filter_state_.state.head<3>() = fixed_baseline_;
    solution = generateSolution(time, SolutionStatus::FIXED, n_sats);
    filter_state_.state.head<3>() = saved_baseline;

    last_fixed_position_ = base_position_ + fixed_baseline_;
    last_fixed_time_ = time;
    has_last_fixed_time_ = true;
    debug_telemetry_.hold_fix_applied = true;

    return true;
}

// ============================================================
// Solution
// ============================================================
PositionSolution RTKProcessor::generateSolution(const GNSSTime& time, SolutionStatus status, int num_satellites) {
    PositionSolution solution;
    solution.time = time;
    solution.status = status;
    solution.position_ecef = (filter_state_.state.size() >= 3) ?
        Vector3d(base_position_ + filter_state_.state.head<3>()) : base_position_;
    const double a = constants::WGS84_A, f = constants::WGS84_F, b_axis = a*(1-f), e2 = constants::WGS84_E2;
    double x = solution.position_ecef(0), y = solution.position_ecef(1), z = solution.position_ecef(2);
    double p = std::sqrt(x*x + y*y), theta = std::atan2(z*a, p*b_axis);
    solution.position_geodetic.longitude = std::atan2(y, x);
    solution.position_geodetic.latitude = std::atan2(z + e2*b_axis/(1-e2)*std::pow(std::sin(theta),3), p - e2*a*std::pow(std::cos(theta),3));
    double N = a / std::sqrt(1 - e2*std::sin(solution.position_geodetic.latitude)*std::sin(solution.position_geodetic.latitude));
    solution.position_geodetic.height = p / std::cos(solution.position_geodetic.latitude) - N;
    solution.num_satellites = num_satellites;
    solution.position_covariance = Matrix3d::Identity() * 0.01;
    solution.pdop = 2.0; solution.hdop = 1.5; solution.vdop = 2.5;
    if (filter_state_.state.size() >= 3) solution.baseline_length = filter_state_.state.head<3>().norm();
    solution.ratio = last_ar_ratio_;
    solution.num_fixed_ambiguities = last_num_fixed_ambiguities_;
    solution.iterations = current_update_diagnostics_.iterations;
    solution.residual_rms = current_update_diagnostics_.post_suppression_residual_rms_m;
    solution.rtk_update_observations = current_update_diagnostics_.observation_count;
    solution.rtk_update_phase_observations = current_update_diagnostics_.phase_observation_count;
    solution.rtk_update_code_observations = current_update_diagnostics_.code_observation_count;
    solution.rtk_update_suppressed_outliers = current_update_diagnostics_.suppressed_outliers;
    solution.rtk_update_prefit_residual_rms_m =
        current_update_diagnostics_.prefit_residual_rms_m;
    solution.rtk_update_prefit_residual_max_m =
        current_update_diagnostics_.prefit_residual_max_m;
    solution.rtk_update_post_suppression_residual_rms_m =
        current_update_diagnostics_.post_suppression_residual_rms_m;
    solution.rtk_update_post_suppression_residual_max_m =
        current_update_diagnostics_.post_suppression_residual_max_m;
    solution.rtk_update_normalized_innovation_squared =
        current_update_diagnostics_.normalized_innovation_squared;
    solution.rtk_update_normalized_innovation_squared_per_observation =
        current_update_diagnostics_.normalized_innovation_squared_per_observation;
    solution.rtk_update_rejected_by_innovation_gate =
        current_update_diagnostics_.rejected_by_innovation_gate ? 1 : 0;
    if (rtk_config_.enable_velocity_states &&
        filter_state_.state.size() >= VELOCITY_STATE_INDEX + VELOCITY_STATES &&
        filter_state_.covariance.rows() == filter_state_.state.size()) {
        const Vector3d velocity = filter_state_.state.segment<3>(VELOCITY_STATE_INDEX);
        const Matrix3d velocity_covariance =
            filter_state_.covariance.block<3, 3>(
                VELOCITY_STATE_INDEX, VELOCITY_STATE_INDEX);
        if (velocity.allFinite() && velocity_covariance.allFinite() &&
            velocity_covariance.diagonal().minCoeff() > 0.0) {
            solution.velocity_ecef = velocity;
            solution.velocity_covariance = velocity_covariance;
            solution.has_velocity = true;
        }
    }
    rememberSolution(solution);
    return solution;
}

void RTKProcessor::rememberSolution(const PositionSolution& solution) {
    if (!solution.isValid()) return;
    last_solution_position_ = solution.position_ecef;
    has_last_solution_position_ = true;
    last_epoch_time_ = solution.time;
    has_last_epoch_ = true;
    bool refresh_trusted = solution.status == SolutionStatus::FIXED;
    if (!refresh_trusted &&
        solution.status == SolutionStatus::FLOAT &&
        solution.num_satellites >= 5) {
        if (isMovingBasePositionMode(rtk_config_)) {
            refresh_trusted = true;
        } else if (!has_last_trusted_position_ || !has_last_trusted_time_) {
            refresh_trusted = true;
        } else {
            double dt = solution.time - last_trusted_time_;
            if (!std::isfinite(dt) || dt < 0.5) {
                dt = 1.0;
            }
            const double trusted_jump =
                (solution.position_ecef - last_trusted_position_).norm();
            // WP9 optional lever: relax the jump gate 2x when a majority of
            // this epoch's tracked satellites are NLOS-flagged (per the
            // loaded weight table). No-op (multiplier stays 1.0) unless
            // both --trust-gate-nlos-relax is set and current_epoch_nlos_
            // fraction_ was actually computed this epoch (finite).
            double jump_gate_multiplier = 1.0;
            if (rtk_config_.trust_gate_nlos_relax &&
                std::isfinite(current_epoch_nlos_fraction_) &&
                current_epoch_nlos_fraction_ > 0.5) {
                jump_gate_multiplier = 2.0;
            }
            refresh_trusted = trusted_jump <= std::max(3.0, 6.0 * dt) * jump_gate_multiplier;
        }
    }
    if (refresh_trusted) {
        // WP9: remember the trust sample being replaced so CV_PREDICT can
        // derive a two-point constant-velocity estimate from the last two
        // trusted deltas. No effect on any exported solution.
        if (has_last_trusted_position_ && has_last_trusted_time_) {
            prev_trusted_position_ = last_trusted_position_;
            prev_trusted_time_ = last_trusted_time_;
            has_prev_trusted_position_ = true;
        }
        last_trusted_position_ = solution.position_ecef;
        has_last_trusted_position_ = true;
        last_trusted_time_ = solution.time;
        has_last_trusted_time_ = true;
    }
}

void RTKProcessor::updateStatistics(SolutionStatus status) const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    const_cast<size_t&>(total_epochs_processed_)++;
    if (status == SolutionStatus::FIXED) const_cast<size_t&>(fixed_solutions_)++;
    else if (status == SolutionStatus::FLOAT) const_cast<size_t&>(float_solutions_)++;
}

// ============================================================
// LAMBDA - C++ implementation (ported from RTKLIB lambda.c)
// ============================================================
bool RTKProcessor::lambdaMethod(const VectorXd& float_ambiguities, const MatrixXd& covariance,
    VectorXd& fixed_ambiguities, double& success_rate) {
    int n = float_ambiguities.size();
    if (n == 0 || n > MAXSAT * 2) return false;

    if (rtk_config_.ar_policy == RTKConfig::ARPolicy::DEMO5_CONTINUOUS) {
        // demo5-continuous: pass raw covariance directly to LAMBDA (no regularization)
        return lambdaSearch(float_ambiguities, covariance, fixed_ambiguities, success_rate);
    }

    // Regularize covariance to ensure positive-definiteness for LAMBDA
    MatrixXd Q_reg = covariance;
    Q_reg = (Q_reg + Q_reg.transpose()) * 0.5;

    // Ensure minimum diagonal values
    constexpr double MIN_VAR = 1e-6;
    for (int i = 0; i < n; ++i) {
        if (Q_reg(i, i) < MIN_VAR) Q_reg(i, i) = MIN_VAR;
    }

    // Prefer cheap diagonal ridge retries over an eigensolver fallback.
    double ridge = 0.0;
    Eigen::LLT<MatrixXd> llt;
    bool ok = false;
    for (int attempt = 0; attempt < 5; ++attempt) {
        MatrixXd candidate = Q_reg;
        if (ridge > 0.0) {
            candidate.diagonal().array() += ridge;
        }
        llt.compute(candidate);
        if (llt.info() == Eigen::Success) {
            Q_reg.swap(candidate);
            ok = true;
            break;
        }
        ridge = (ridge == 0.0) ? MIN_VAR : ridge * 10.0;
    }
    if (!ok) {
        Eigen::SelfAdjointEigenSolver<MatrixXd> eig(Q_reg);
        if (eig.info() != Eigen::Success) {
            return false;
        }
        const double min_eig = eig.eigenvalues().minCoeff();
        if (!std::isfinite(min_eig)) {
            return false;
        }
        if (min_eig < MIN_VAR) {
            Q_reg.diagonal().array() += (MIN_VAR - min_eig);
        }
    }

    return lambdaSearch(float_ambiguities, Q_reg, fixed_ambiguities, success_rate);
}

// Old updateFilter signature stub
bool RTKProcessor::updateFilter(const ObservationData&, const ObservationData&, const NavigationData&) {
    return updateFilter(current_sat_data_);
}

// Legacy stubs
std::vector<RTKProcessor::DoubleDifference> RTKProcessor::formDoubleDifferences(
    const ObservationData& rover_obs, const ObservationData& base_obs, const NavigationData& nav,
    const Vector3d* evaluation_rover_position_ecef) {
    if (!base_position_known_) {
        if (base_obs.receiver_position.norm() > 1e6) {
            const_cast<RTKProcessor*>(this)->setBasePosition(base_obs.receiver_position);
        } else {
            return {};
        }
    }

    if (!filter_initialized_) {
        if (!const_cast<RTKProcessor*>(this)->initializeFilter(rover_obs, base_obs, nav)) {
            return {};
        }
    }

    const bool reuse_processed_epoch = !current_sat_data_.empty() &&
        std::abs(current_epoch_time_ - rover_obs.time) <= 1e-6;
    const auto sat_data = reuse_processed_epoch
        ? current_sat_data_
        : const_cast<RTKProcessor*>(this)->collectSatelliteData(rover_obs, base_obs, nav);
    if (sat_data.size() < 4) {
        return {};
    }

    if (!reuse_processed_epoch) {
        const_cast<RTKProcessor*>(this)->current_sat_data_ = sat_data;
        const double bias_dt =
            has_last_epoch_ ? std::max(rover_obs.time - last_epoch_time_, 1e-3) : 1.0;
        const_cast<RTKProcessor*>(this)->updateBias(sat_data, bias_dt);
    }

    std::vector<DoubleDifference> measurements;
    const auto dd_pairs = buildDoubleDifferencePairs(sat_data, 0);
    if (dd_pairs.empty()) {
        return measurements;
    }

    const Vector3d rover_pos = evaluation_rover_position_ecef != nullptr
        ? *evaluation_rover_position_ecef
        : base_position_ + filter_state_.state.head<3>();
    for (const auto& pair : dd_pairs) {
        const auto ref_it = sat_data.find(pair.ref_sat);
        const auto sat_it = sat_data.find(pair.sat);
        if (ref_it == sat_data.end() || sat_it == sat_data.end()) {
            continue;
        }

        const auto& ref_sd = ref_it->second;
        const auto& sd = sat_it->second;
        const bool use_l1 = pair.freq == 0;
        if ((use_l1 && (!ref_sd.has_l1 || !sd.has_l1)) ||
            (!use_l1 && (!ref_sd.has_l2 || !sd.has_l2))) {
            continue;
        }

        const double ref_code = use_l1 ? ref_sd.rover_l1_code - ref_sd.base_l1_code
                                       : ref_sd.rover_l2_code - ref_sd.base_l2_code;
        const double sat_code = use_l1 ? sd.rover_l1_code - sd.base_l1_code
                                       : sd.rover_l2_code - sd.base_l2_code;
        const double ref_phase = use_l1 ? ref_sd.rover_l1_phase - ref_sd.base_l1_phase
                                        : ref_sd.rover_l2_phase - ref_sd.base_l2_phase;
        const double sat_phase = use_l1 ? sd.rover_l1_phase - sd.base_l1_phase
                                        : sd.rover_l2_phase - sd.base_l2_phase;

        const double rr_ref =
            geodist_range(ref_sd.sat_pos, rover_pos) + tropModel(rover_pos, ref_sd.elevation);
        const double br_ref = geodist_range(ref_sd.sat_pos_base, base_position_) +
                              tropModel(base_position_, ref_sd.base_elevation);
        const double rr =
            geodist_range(sd.sat_pos, rover_pos) + tropModel(rover_pos, sd.elevation);
        const double br = geodist_range(sd.sat_pos_base, base_position_) +
                          tropModel(base_position_, sd.base_elevation);

        DoubleDifference measurement;
        measurement.reference_satellite = pair.ref_sat;
        measurement.satellite = pair.sat;
        measurement.signal = use_l1 ? sd.l1_signal : sd.l2_signal;
        measurement.pseudorange_dd = ref_code - sat_code;
        measurement.carrier_phase_dd = ref_phase - sat_phase;
        measurement.geometric_range = (rr_ref - br_ref) - (rr - br);
        measurement.unit_vector =
            -(ref_sd.sat_pos - rover_pos).normalized() + (sd.sat_pos - rover_pos).normalized();
        measurement.elevation = std::min(ref_sd.elevation, sd.elevation);
        const double ref_snr = use_l1
            ? combinedSnrDbHz(ref_sd.rover_l1_snr, ref_sd.base_l1_snr)
            : combinedSnrDbHz(ref_sd.rover_l2_snr, ref_sd.base_l2_snr);
        const double sat_snr = use_l1
            ? combinedSnrDbHz(sd.rover_l1_snr, sd.base_l1_snr)
            : combinedSnrDbHz(sd.rover_l2_snr, sd.base_l2_snr);
        const double dd_snr = combinedSnrDbHz(ref_snr, sat_snr);
        measurement.variance = varerr(measurement.elevation, true, dd_snr);
        measurement.code_variance = varerr(measurement.elevation, false, dd_snr);
        measurement.wavelength = use_l1 ? sd.l1_wavelength : sd.l2_wavelength;
        measurement.frequency_index = pair.freq;
        const auto& locks = use_l1 ? lock_count_l1_ : lock_count_l2_;
        const auto sat_lock = locks.find(pair.sat);
        const auto ref_lock = locks.find(pair.ref_sat);
        measurement.lock_count = std::min(
            sat_lock == locks.end() ? 0 : sat_lock->second,
            ref_lock == locks.end() ? 0 : ref_lock->second);
        measurement.cycle_slip =
            (use_l1 ? (sd.l1_lli | ref_sd.l1_lli) : (sd.l2_lli | ref_sd.l2_lli)) != 0 ||
            measurement.lock_count <= 0;
        measurement.valid = true;
        measurements.push_back(std::move(measurement));
    }

    return measurements;
}

std::vector<dd_imu_bridge::DDObservation> RTKProcessor::formTightlyCoupledObservations(
    const ObservationData& rover_obs, const ObservationData& base_obs,
    const NavigationData& nav, const Vector3d& rover_position_ecef,
    const Matrix3d& ecef_to_enu, const Eigen::Quaterniond& attitude_body_to_enu) {
    std::vector<dd_imu_bridge::DDObservation> rows;
    if (!rover_position_ecef.allFinite() || !ecef_to_enu.allFinite() ||
        !attitude_body_to_enu.coeffs().allFinite()) {
        return rows;
    }
    const auto measurements = formDoubleDifferences(
        rover_obs, base_obs, nav, &rover_position_ecef);
    rows.reserve(measurements.size());
    for (const auto& measurement : measurements) {
        if (!measurement.valid || !(measurement.wavelength > 0.0) ||
            !std::isfinite(measurement.geometric_range)) {
            continue;
        }
        dd_imu_bridge::DDObservation row;
        row.key.satellite_prn = measurement.satellite.prn;
        row.key.frequency_index = measurement.frequency_index;
        row.key.satellite_system = static_cast<int>(measurement.satellite.system);
        row.key.reference_satellite_prn = measurement.reference_satellite.prn;
        row.key.reference_satellite_system =
            static_cast<int>(measurement.reference_satellite.system);
        row.key.signal_type = static_cast<int>(measurement.signal);
        const Vector3d geometry_enu = ecef_to_enu * measurement.unit_vector;
        row.geometry_enu = geometry_enu.transpose();
        row.code_residual_m = measurement.pseudorange_dd - measurement.geometric_range;
        row.code_variance_m2 = measurement.code_variance;
        row.carrier_residual_m =
            measurement.carrier_phase_dd * measurement.wavelength -
            measurement.geometric_range;
        // A newly formed DD arc can appear numerically precise while its
        // reference relationship and slip history are not yet stable. Keep
        // code rows active immediately, but delay carrier/ambiguity state
        // augmentation until roughly one minute at 5 Hz.
        constexpr int kTightCarrierMinLockCount = 300;
        row.carrier_variance_m2 = measurement.lock_count >= kTightCarrierMinLockCount
            ? measurement.variance
            : 0.0;
        row.wavelength_m = measurement.wavelength;
        row.elevation_rad = measurement.elevation;
        const Vector3d geometry_body = attitude_body_to_enu.conjugate() * geometry_enu;
        row.body_azimuth_rad = std::atan2(geometry_body.y(), geometry_body.x());
        const double ambiguity_cycles = row.carrier_residual_m / row.wavelength_m;
        row.posterior_abs_residual_m = row.wavelength_m *
            std::abs(ambiguity_cycles - std::round(ambiguity_cycles));
        row.lock_count = measurement.lock_count;
        row.cycle_slip = measurement.cycle_slip;
        rows.push_back(std::move(row));
    }
    return rows;
}
void RTKProcessor::detectCycleSlips(const ObservationData&, const ObservationData&) {}
bool RTKProcessor::resolveAmbiguities(int) { return resolveAmbiguities(); }
RTKProcessor::LAMBDAResult RTKProcessor::solveLAMBDA(const VectorXd& float_ambiguities,
                                                     const MatrixXd& ambiguity_covariance) {
    LAMBDAResult result;
    result.success = lambdaMethod(float_ambiguities, ambiguity_covariance,
                                  result.fixed_ambiguities, result.ratio);
    return result;
}
bool RTKProcessor::validateAmbiguityResolution(const VectorXd& fixed_ambiguities,
                                               const VectorXd& float_ambiguities,
                                               const MatrixXd& covariance,
                                               double ratio) {
    if (fixed_ambiguities.size() == 0 || fixed_ambiguities.size() != float_ambiguities.size()) {
        return false;
    }
    if (covariance.rows() != covariance.cols() ||
        covariance.rows() != fixed_ambiguities.size()) {
        return false;
    }
    if (!std::isfinite(ratio) || ratio < rtk_config_.ambiguity_ratio_threshold) {
        return false;
    }

    for (int i = 0; i < fixed_ambiguities.size(); ++i) {
        if (!std::isfinite(fixed_ambiguities(i)) || !std::isfinite(float_ambiguities(i))) {
            return false;
        }
        if (std::abs(fixed_ambiguities(i) - std::round(fixed_ambiguities(i))) > 1e-6) {
            return false;
        }
    }

    MatrixXd Q = (covariance + covariance.transpose()) * 0.5;
    for (int i = 0; i < Q.rows(); ++i) {
        if (!std::isfinite(Q(i, i)) || Q(i, i) <= 0.0) {
            return false;
        }
    }
    Eigen::LDLT<MatrixXd> solver(Q);
    if (solver.info() != Eigen::Success) {
        return false;
    }
    const VectorXd diff = float_ambiguities - fixed_ambiguities;
    const double norm = diff.transpose() * solver.solve(diff);
    return std::isfinite(norm) && norm >= 0.0;
}
void RTKProcessor::updateAmbiguityStates(const std::vector<DoubleDifference>&) {}
Vector3d RTKProcessor::calculateBaseline() const {
    if (!filter_initialized_ || filter_state_.state.size() < 3) {
        return Vector3d::Zero();
    }
    return filter_state_.state.head<3>();
}
VectorXd RTKProcessor::calculateResiduals(const std::vector<DoubleDifference>& measurements,
                                          const Vector3d& baseline) const {
    VectorXd residuals = VectorXd::Zero(static_cast<int>(measurements.size()));
    for (int i = 0; i < static_cast<int>(measurements.size()); ++i) {
        const auto& measurement = measurements[i];
        const double observed =
            std::abs(measurement.pseudorange_dd) > 0.0 ? measurement.pseudorange_dd
                                                       : measurement.carrier_phase_dd;
        const double predicted =
            measurement.geometric_range + measurement.unit_vector.dot(baseline);
        residuals(i) = observed - predicted;
    }
    return residuals;
}
MatrixXd RTKProcessor::formMeasurementMatrix(const std::vector<DoubleDifference>& measurements,
                                             const NavigationData&,
                                             const GNSSTime&) const {
    MatrixXd H = MatrixXd::Zero(static_cast<int>(measurements.size()), 3);
    for (int i = 0; i < static_cast<int>(measurements.size()); ++i) {
        H.row(i) = measurements[i].unit_vector.transpose();
    }
    return H;
}
MatrixXd RTKProcessor::calculateMeasurementWeights(
    const std::vector<DoubleDifference>& measurements) const {
    MatrixXd W = MatrixXd::Zero(static_cast<int>(measurements.size()),
                                static_cast<int>(measurements.size()));
    for (int i = 0; i < static_cast<int>(measurements.size()); ++i) {
        const double variance = std::max(measurements[i].variance, 1e-6);
        W(i, i) = 1.0 / variance;
    }
    return W;
}
bool RTKProcessor::hasSufficientSatellites(
    const std::vector<DoubleDifference>& measurements) const {
    std::set<SatelliteId> satellites;
    for (const auto& measurement : measurements) {
        satellites.insert(measurement.reference_satellite);
        satellites.insert(measurement.satellite);
    }
    return satellites.size() >= 4;
}
void RTKProcessor::resetAmbiguity(const SatelliteId&, SignalType) {}
bool RTKProcessor::applyFixedAmbiguities(const VectorXd& fixed_n1,
                                         const VectorXd& fixed_n2,
                                         const std::map<SatelliteId, SatelliteData>& sat_data) {
    if (!filter_initialized_) {
        return false;
    }

    std::vector<SatelliteId> satellites;
    for (const auto& [sat, sd] : sat_data) {
        if (sd.has_l1 || sd.has_l2) {
            satellites.push_back(sat);
        }
    }
    std::sort(satellites.begin(), satellites.end());

    int n1_applied = 0;
    int n2_applied = 0;
    for (const auto& sat : satellites) {
        if (n1_applied < fixed_n1.size()) {
            auto it = filter_state_.n1_indices.find(sat);
            if (it != filter_state_.n1_indices.end()) {
                filter_state_.state(it->second) = fixed_n1(n1_applied++);
            }
        }
        if (n2_applied < fixed_n2.size()) {
            auto it = filter_state_.n2_indices.find(sat);
            if (it != filter_state_.n2_indices.end()) {
                filter_state_.state(it->second) = fixed_n2(n2_applied++);
            }
        }
    }
    return n1_applied == fixed_n1.size() && n2_applied == fixed_n2.size();
}
void RTKProcessor::solvePositionWithAmbiguities(const std::map<SatelliteId, SatelliteData>&) {
    if (has_fixed_solution_ && filter_initialized_ && filter_state_.state.size() >= 3) {
        filter_state_.state.head<3>() = fixed_baseline_;
    }
}
bool RTKProcessor::trySingleEpochAR(const std::map<SatelliteId, SatelliteData>& sat_data) {
    if (!filter_initialized_) {
        return false;
    }
    current_sat_data_ = sat_data;
    if (!resolveAmbiguities(buildDoubleDifferencePairs(sat_data, 0))) {
        return false;
    }
    return has_fixed_solution_ && validateFixedSolution(sat_data, last_epoch_time_);
}

} // namespace libgnss
