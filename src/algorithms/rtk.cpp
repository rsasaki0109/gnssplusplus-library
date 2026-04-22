#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/algorithms/rtk_ar_evaluation.hpp>
#include <libgnss++/algorithms/rtk_ar_selection.hpp>
#include <libgnss++/algorithms/lambda.hpp>
#include <libgnss++/algorithms/rtk_measurement.hpp>
#include <libgnss++/algorithms/rtk_selection.hpp>
#include <libgnss++/algorithms/rtk_update.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/signal_policy.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/models/troposphere.hpp>
#include <iostream>
#include <algorithm>
#include <cmath>

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

bool isPrimaryRTKSignal(GNSSSystem system, SignalType signal) {
    return signal_policy::isPrimarySignal(system, signal);
}

bool isSecondaryRTKSignal(GNSSSystem system, SignalType signal) {
    return signal_policy::isSecondarySignal(system, signal);
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
    filter_state_ = RTKState{};
    filter_state_.next_state_idx = REAL_STATES + IONO_STATES;
    ambiguity_states_.clear();
    lock_count_l1_.clear();
    lock_count_l2_.clear();
    has_fixed_solution_ = false;
    has_last_fixed_position_ = false;
    has_last_fixed_time_ = false;
    has_last_solution_position_ = false;
    has_last_trusted_position_ = false;
    has_ref_satellite_ = false;
    has_last_epoch_ = false;
    has_last_trusted_time_ = false;
    current_sat_data_.clear();
    gf_l1l2_history_.clear();
    doppler_phase_history_l1_m_.clear();
    doppler_phase_history_l2_m_.clear();
    code_phase_history_l1_m_.clear();
    code_phase_history_l2_m_.clear();
    consecutive_fix_count_ = 0;
    consecutive_float_count_ = 0;
    consecutive_nonfix_count_ = 0;
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

// RTKLIB varerr: SD measurement error variance
double RTKProcessor::varerr(double elevation, bool is_phase) const {
    double sin_el = std::sin(elevation);
    if (sin_el < 0.1) sin_el = 0.1;
    double a = is_phase ? rtk_config_.carrier_phase_sigma : rtk_config_.pseudorange_sigma;
    double b = is_phase ? rtk_config_.carrier_phase_sigma : rtk_config_.pseudorange_sigma;
    return 2.0 * (a * a + b * b / (sin_el * sin_el));
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
    for (int j = 0; j < NX; ++j) {
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
    for (int j = 0; j < NX; ++j) {
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
    for (int j = 0; j < NX; ++j) {
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
        for (int j = 0; j < NX; ++j) {
            filter_state_.covariance(idx, j) = 0.0;
            filter_state_.covariance(j, idx) = 0.0;
        }
        filter_state_.iono_indices.erase(it0);
    }
    auto it1 = filter_state_.n1_indices.find(sat);
    if (it1 != filter_state_.n1_indices.end()) {
        int idx = it1->second;
        filter_state_.state(idx) = 0.0;
        for (int j = 0; j < NX; ++j) {
            filter_state_.covariance(idx, j) = 0.0;
            filter_state_.covariance(j, idx) = 0.0;
        }
        filter_state_.n1_indices.erase(it1);
    }
    auto it2 = filter_state_.n2_indices.find(sat);
    if (it2 != filter_state_.n2_indices.end()) {
        int idx = it2->second;
        filter_state_.state(idx) = 0.0;
        for (int j = 0; j < NX; ++j) {
            filter_state_.covariance(idx, j) = 0.0;
            filter_state_.covariance(j, idx) = 0.0;
        }
        filter_state_.n2_indices.erase(it2);
    }
}

// ============================================================
// Satellite data collection
// ============================================================
std::map<SatelliteId, RTKProcessor::SatelliteData> RTKProcessor::collectSatelliteData(
    const ObservationData& rover_obs, const ObservationData& base_obs, const NavigationData& nav) {
    std::map<SatelliteId, SatelliteData> result;
    std::map<SatelliteId, std::vector<const Observation*>> rover_l1, rover_l2, base_l1, base_l2;
    for (const auto& obs : rover_obs.observations) {
        if (!isEnabledRTKSystem(rtk_config_, obs.satellite.system)) continue;
        if (!isUsableRTKSatellite(obs.satellite)) continue;
        if (isPrimaryRTKSignal(obs.satellite.system, obs.signal) &&
            obs.has_carrier_phase && obs.has_pseudorange) {
            rover_l1[obs.satellite].push_back(&obs);
        }
        if (isSecondaryRTKSignal(obs.satellite.system, obs.signal) &&
            obs.has_carrier_phase && obs.has_pseudorange) {
            rover_l2[obs.satellite].push_back(&obs);
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
            base_l2[obs.satellite].push_back(&obs);
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
                sd.has_l2 = sd.l2_wavelength > 0.0;
                sd.l2_lli = r_l2_obs->lli | b_l2_obs->lli;
                sd.has_l2_doppler = r_l2_obs->has_doppler && b_l2_obs->has_doppler;
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
        item.l1_wavelength = sd.l1_wavelength;
        item.l2_wavelength = sd.l2_wavelength;
        item.elevation = sd.elevation;
        auto n1_it = filter_state_.n1_indices.find(sat);
        item.n1_active = n1_it != filter_state_.n1_indices.end() &&
                         filter_state_.state(n1_it->second) != 0.0;
        auto n2_it = filter_state_.n2_indices.find(sat);
        item.n2_active = n2_it != filter_state_.n2_indices.end() &&
                         filter_state_.state(n2_it->second) != 0.0;
        auto l1_it = lock_count_l1_.find(sat);
        item.lock_count_l1 = l1_it != lock_count_l1_.end() ? l1_it->second : 0;
        auto l2_it = lock_count_l2_.find(sat);
        item.lock_count_l2 = l2_it != lock_count_l2_.end() ? l2_it->second : 0;
        snapshot.push_back(item);
    }
    return snapshot;
}

std::vector<RTKProcessor::DDPair> RTKProcessor::buildDoubleDifferencePairs(
    const std::map<SatelliteId, SatelliteData>& sat_data,
    int min_lock_count) const {
    std::vector<DDPair> dd_pairs;
    const auto snapshot = buildSelectionSnapshot(sat_data);

    for (GNSSSystem system : kRTKSupportedSystems) {
        if (!isEnabledRTKSystem(rtk_config_, system)) continue;
        const auto system_pairs = rtk_selection::buildDoubleDifferencePairsForSystem(
            snapshot,
            system,
            min_lock_count,
            requiresMatchedCarrierWavelength(rtk_config_, system));
        for (const auto& pair : system_pairs) {
            if (pair.freq == 0) {
                auto ref_idx = filter_state_.n1_indices.find(pair.ref_sat);
                auto sat_idx = filter_state_.n1_indices.find(pair.sat);
                if (ref_idx == filter_state_.n1_indices.end() || sat_idx == filter_state_.n1_indices.end()) {
                    continue;
                }
                dd_pairs.push_back({pair.ref_sat, ref_idx->second, sat_idx->second, pair.sat, pair.freq});
            } else {
                auto ref_idx = filter_state_.n2_indices.find(pair.ref_sat);
                auto sat_idx = filter_state_.n2_indices.find(pair.sat);
                if (ref_idx == filter_state_.n2_indices.end() || sat_idx == filter_state_.n2_indices.end()) {
                    continue;
                }
                dd_pairs.push_back({pair.ref_sat, ref_idx->second, sat_idx->second, pair.sat, pair.freq});
            }
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
    filter_state_.state = VectorXd::Zero(NX);
    filter_state_.covariance = MatrixXd::Zero(NX, NX);
    filter_state_.iono_indices.clear();
    filter_state_.n1_indices.clear();
    filter_state_.n2_indices.clear();
    filter_state_.next_state_idx = REAL_STATES + IONO_STATES;

    auto spp = spp_processor_.processEpoch(rover_obs, nav);
    Vector3d rover_pos;
    if (spp.isValid()) {
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
            for (int j = 0; j < NX; ++j) {
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
void RTKProcessor::updateBias(const std::map<SatelliteId, SatelliteData>& sat_data, double dt_s) {
    std::vector<SatelliteId> sats_to_remove;
    for (const auto& [sat, idx] : filter_state_.n1_indices) {
        if (sat_data.find(sat) == sat_data.end()) sats_to_remove.push_back(sat);
    }
    for (const auto& sat : sats_to_remove) {
        removeSatelliteFromState(sat);
        lock_count_l1_.erase(sat);
        lock_count_l2_.erase(sat);
        gf_l1l2_history_.erase(sat);
        doppler_phase_history_l1_m_.erase(sat);
        doppler_phase_history_l2_m_.erase(sat);
        code_phase_history_l1_m_.erase(sat);
        code_phase_history_l2_m_.erase(sat);
    }

    std::set<SatelliteId> gf_slips;
    if (rtk_config_.enable_cycle_slip_detection) {
        const double gf_slip_threshold =
            isDynamicPositionMode(rtk_config_)
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
    }

    std::set<SatelliteId> doppler_slips_l1;
    std::set<SatelliteId> doppler_slips_l2;
    if (rtk_config_.enable_doppler_slip_detection &&
        std::isfinite(dt_s) &&
        dt_s > 0.0 &&
        dt_s <= 5.0) {
        const double doppler_slip_threshold =
            isDynamicPositionMode(rtk_config_)
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
        }
    }

    std::set<SatelliteId> code_slips_l1;
    std::set<SatelliteId> code_slips_l2;
    if (rtk_config_.enable_code_slip_detection) {
        const double code_slip_threshold =
            isDynamicPositionMode(rtk_config_)
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
        }
    }

    for (int freq = 0; freq < 2; ++freq) {
        auto& indices = (freq == 0) ? filter_state_.n1_indices : filter_state_.n2_indices;
        auto& lock_counts = (freq == 0) ? lock_count_l1_ : lock_count_l2_;

        // Detect cycle slips and reset
        for (const auto& [sat, sd] : sat_data) {
            bool has_freq = (freq == 0) ? sd.has_l1 : sd.has_l2;
            if (!has_freq) continue;
            int lli = (freq == 0) ? sd.l1_lli : sd.l2_lli;
            bool slip = (lli & 0x01) != 0 ||
                        gf_slips.find(sat) != gf_slips.end() ||
                        (freq == 0 ? code_slips_l1.find(sat) != code_slips_l1.end()
                                   : code_slips_l2.find(sat) != code_slips_l2.end()) ||
                        (freq == 0 ? doppler_slips_l1.find(sat) != doppler_slips_l1.end()
                                   : doppler_slips_l2.find(sat) != doppler_slips_l2.end());
            auto idx_it = indices.find(sat);
            if (idx_it != indices.end() && slip) {
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

        for (GNSSSystem system : kRTKSupportedSystems) {
            if (!isEnabledRTKSystem(rtk_config_, system)) continue;
            std::map<SatelliteId, double> bias;
            double offset = 0.0;
            int offset_count = 0;

            for (const auto& [sat, sd] : sat_data) {
                if (sat.system != system) continue;
                bool has_freq = (freq == 0) ? sd.has_l1 : sd.has_l2;
                const double wavelength = (freq == 0) ? sd.l1_wavelength : sd.l2_wavelength;
                if (!has_freq || wavelength <= 0.0) continue;
                double cp, pr;
                if (freq == 0) {
                    cp = sd.rover_l1_phase - sd.base_l1_phase;
                    pr = sd.rover_l1_code - sd.base_l1_code;
                } else {
                    cp = sd.rover_l2_phase - sd.base_l2_phase;
                    pr = sd.rover_l2_code - sd.base_l2_code;
                }
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
                else getOrCreateN2Index(sat, b);
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
                                                        const NavigationData& nav) {
    if (!filter_initialized_) {
        consecutive_float_count_ = 0;
        consecutive_nonfix_count_ = 0;
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
    // Also reset position to SPP to prevent baseline drift.
    resetPositionToSPP(rover_obs, nav);
    // DO NOT clear held DD integers (last_dd_fixed_, last_dd_pairs_,
    // last_best_subset_) — they enable hold fix after reset.
    consecutive_float_count_ = 0;
    consecutive_nonfix_count_ = 0;
}

void RTKProcessor::recordFixedEpoch() {
    consecutive_float_count_ = 0;
    consecutive_nonfix_count_ = 0;
}

void RTKProcessor::recordFloatEpoch(const ObservationData& rover_obs, const NavigationData& nav) {
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
    if (rtk_config_.position_mode == RTKConfig::PositionMode::STATIC) {
        // Static: position accumulates with process noise
        double pos_pnoise = rtk_config_.process_noise_position;  // default 1e-4 m^2/s
        for (int i = 0; i < BASE_STATES; ++i)
            filter_state_.covariance(i, i) += pos_pnoise;
        return;
    }

    const bool moving_base_mode = isMovingBasePositionMode(rtk_config_);
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
        if (spp.isValid()) {
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
    PositionSolution solution;
    solution.time = rover_obs.time;
    solution.status = SolutionStatus::NONE;
    current_update_diagnostics_ = RTKUpdateDiagnostics{};

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
                last_solution_position_ = saved_last_solution_position;
                has_last_solution_position_ = saved_has_last_solution;
                last_epoch_time_ = saved_last_solution_time;
                has_last_epoch_ = saved_has_last_solution_time;
                last_trusted_position_ = saved_last_trusted_position;
                has_last_trusted_position_ = saved_has_last_trusted;
                last_trusted_time_ = saved_last_trusted_time;
                has_last_trusted_time_ = saved_has_last_trusted_time;
                return fallback_spp();
            }

            if (!moving_base_mode &&
                n_sats <= 4 && has_last_trusted_position_ && has_last_trusted_time_) {
                const double trusted_jump =
                    (solution.position_ecef - saved_last_trusted_position).norm();
                if (trusted_jump > 25.0) {
                    last_solution_position_ = saved_last_solution_position;
                    has_last_solution_position_ = saved_has_last_solution;
                    last_epoch_time_ = saved_last_solution_time;
                    has_last_epoch_ = saved_has_last_solution_time;
                    last_trusted_position_ = saved_last_trusted_position;
                    has_last_trusted_position_ = saved_has_last_trusted;
                    last_trusted_time_ = saved_last_trusted_time;
                    has_last_trusted_time_ = saved_has_last_trusted_time;
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
                        last_solution_position_ = saved_last_solution_position;
                        has_last_solution_position_ = saved_has_last_solution;
                        last_epoch_time_ = saved_last_solution_time;
                        has_last_epoch_ = saved_has_last_solution_time;
                        last_trusted_position_ = saved_last_trusted_position;
                        has_last_trusted_position_ = saved_has_last_trusted;
                        last_trusted_time_ = saved_last_trusted_time;
                        has_last_trusted_time_ = saved_has_last_trusted_time;
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
                    last_solution_position_ = saved_last_solution_position;
                    has_last_solution_position_ = saved_has_last_solution;
                    last_epoch_time_ = saved_last_solution_time;
                    has_last_epoch_ = saved_has_last_solution_time;
                    return fallback_spp();
                }
            }

            const auto saved_hold_state = captureHoldState();
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
                if (validateFixedSolution(sat_data, rover_obs.time)) {
                    Vector3d saved_baseline = filter_state_.state.head<3>();
                    filter_state_.state.head<3>() = fixed_baseline_;
                    solution = generateSolution(rover_obs.time, SolutionStatus::FIXED, n_sats);
                    filter_state_.state.head<3>() = saved_baseline;
                    const double fixed_float_jump =
                        (solution.position_ecef - float_solution.position_ecef).norm();
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
                        has_fixed_solution_ = false;
                        restoreHoldState(saved_hold_state);
                        last_trusted_position_ = saved_last_trusted_position;
                        has_last_trusted_position_ = saved_has_last_trusted;
                        last_trusted_time_ = saved_last_trusted_time;
                        has_last_trusted_time_ = saved_has_last_trusted_time;
                    } else {
                        updateStatistics(SolutionStatus::FIXED);
                        consecutive_fix_count_++;
                        recordFixedEpoch();

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
                    has_fixed_solution_ = false;
                }
            }

            if (!moving_base_mode &&
                !applied_fix_solution &&
                rtk_config_.ar_policy != RTKConfig::ARPolicy::DEMO5_CONTINUOUS &&
                rtk_validation::canAttemptHoldFix(consecutive_fix_count_,
                                                  rtk_config_.min_hold_count,
                                                  saved_hold_state.has_last_fixed_position,
                                                  saved_hold_state.hasHeldIntegers())) {
                restoreHoldState(saved_hold_state);
                if (tryHoldFix(sat_data, rover_obs.time, n_sats, solution)) {
                    updateStatistics(SolutionStatus::FIXED);
                    consecutive_fix_count_++;
                    recordFixedEpoch();
                    if (consecutive_fix_count_ >= rtk_config_.min_hold_count) {
                        applyHoldAmbiguity();
                    }
                    applied_fix_solution = true;
                }
            }

            if (!applied_fix_solution) {
                restoreHoldState(saved_hold_state);
                rememberSolution(float_solution);
                updateStatistics(SolutionStatus::FLOAT);
                consecutive_fix_count_ = 0;
                recordFloatEpoch(rover_obs, nav);
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

    for (GNSSSystem system : kRTKSupportedSystems) {
        if (!isEnabledRTKSystem(rtk_config_, system)) continue;
        SatelliteId ref_sat;
        if (!rtk_selection::selectSystemReferenceSatellite(selection_snapshot, system, 0, ref_sat)) continue;

        auto ref_it = sat_data.find(ref_sat);
        if (ref_it == sat_data.end()) continue;
        const auto& ref_sd = ref_it->second;
        const auto system_pairs = rtk_selection::buildDoubleDifferencePairsForSystem(
            selection_snapshot,
            system,
            0,
            requiresMatchedCarrierWavelength(rtk_config_, system));
        const double rr_ref = geodist_range(ref_sd.sat_pos, rover_pos) +
                              tropModel(rover_pos, ref_sd.elevation);
        const double br_ref = geodist_range(ref_sd.sat_pos_base, base_position_) +
                              tropModel(base_position_, ref_sd.base_elevation);
        const Vector3d los_ref = (ref_sd.sat_pos - rover_pos).normalized();

        auto append_frequency_blocks = [&](int freq) {
            rtk_measurement::MeasurementBlock phase_block;
            phase_block.kind = rtk_measurement::MeasurementKind::PHASE;
            phase_block.frequency_index = freq;
            rtk_measurement::MeasurementBlock code_block;
            code_block.kind = rtk_measurement::MeasurementKind::CODE;
            code_block.frequency_index = freq;
            const auto& ref_indices = (freq == 0) ? filter_state_.n1_indices : filter_state_.n2_indices;
            auto ref_state_it = ref_indices.find(ref_sat);
            if (ref_state_it == ref_indices.end()) {
                blocks.push_back(std::move(phase_block));
                blocks.push_back(std::move(code_block));
                return;
            }

            const double ref_wavelength = (freq == 0) ? ref_sd.l1_wavelength : ref_sd.l2_wavelength;
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
                          (freq == 0) ? ref_sd.l1_frequency_hz : ref_sd.l2_frequency_hz)
                    : 0.0;
            if (estimate_iono && filter_state_.covariance(ref_iono_idx, ref_iono_idx) <= 0.0) {
                blocks.push_back(std::move(phase_block));
                blocks.push_back(std::move(code_block));
                return;
            }
            const double ref_phase_variance = varerr(ref_sd.elevation, true);
            const double ref_code_variance = varerr(ref_sd.elevation, false);

            for (const auto& pair : system_pairs) {
                if (pair.freq != freq) continue;
                const auto sat_it = sat_data.find(pair.sat);
                if (sat_it == sat_data.end()) continue;
                const auto& sat = pair.sat;
                const auto& sd = sat_it->second;
                const auto& sat_indices = (freq == 0) ? filter_state_.n1_indices : filter_state_.n2_indices;
                auto sat_state_it = sat_indices.find(sat);
                if (sat_state_it == sat_indices.end()) continue;

                const double sat_wavelength = (freq == 0) ? sd.l1_wavelength : sd.l2_wavelength;
                if (sat_wavelength <= 0.0) continue;
                const double sat_phase_variance = varerr(sd.elevation, true);
                const double sat_code_variance = varerr(sd.elevation, false);
                const int sat_iono_idx = estimate_iono ? II(sat) : -1;
                const double sat_iono_scale =
                    estimate_iono
                        ? ionoFrequencyScale(
                              freq,
                              sd.l1_frequency_hz,
                              (freq == 0) ? sd.l1_frequency_hz : sd.l2_frequency_hz)
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
                const double ref_phase = (freq == 0) ? ref_sd.rover_l1_phase - ref_sd.base_l1_phase
                                                     : ref_sd.rover_l2_phase - ref_sd.base_l2_phase;
                const double sat_phase = (freq == 0) ? sd.rover_l1_phase - sd.base_l1_phase
                                                     : sd.rover_l2_phase - sd.base_l2_phase;
                const double ref_code = (freq == 0) ? ref_sd.rover_l1_code - ref_sd.base_l1_code
                                                    : ref_sd.rover_l2_code - ref_sd.base_l2_code;
                const double sat_code = (freq == 0) ? sd.rover_l1_code - sd.base_l1_code
                                                    : sd.rover_l2_code - sd.base_l2_code;
                const bool autocal_glonass =
                    usesGlonassAutocal(rtk_config_) &&
                    ref_sd.satellite.system == GNSSSystem::GLONASS &&
                    sd.satellite.system == GNSSSystem::GLONASS &&
                    freq < GLO_HWBIAS_STATES;
                const double df_mhz =
                    autocal_glonass
                        ? (((freq == 0) ? ref_sd.l1_frequency_hz : ref_sd.l2_frequency_hz) -
                           ((freq == 0) ? sd.l1_frequency_hz : sd.l2_frequency_hz)) / 1e6
                        : 0.0;
                const double glonass_icb =
                    glonassInterChannelBiasMeters(
                        rtk_config_,
                        ref_sd.satellite.system,
                        sd.satellite.system,
                        (freq == 0) ? ref_sd.l1_frequency_hz : ref_sd.l2_frequency_hz,
                        (freq == 0) ? sd.l1_frequency_hz : sd.l2_frequency_hz,
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
                code_block.rows.push_back(std::move(code_row));
            }
            blocks.push_back(std::move(phase_block));
            blocks.push_back(std::move(code_block));
        };

        append_frequency_blocks(0);
        append_frequency_blocks(1);
    }

    return blocks;
}

bool RTKProcessor::updateFilter(const std::map<SatelliteId, SatelliteData>& sat_data) {
    if (sat_data.size() < 4) return false;

    const auto blocks = buildMeasurementBlocks(sat_data);
    const auto measurement_diagnostics = rtk_measurement::summarizeMeasurementBlocks(blocks);
    auto measurement_system = rtk_measurement::assembleMeasurementSystem(
        blocks, filter_state_.state.size());
    const auto update_result = rtk_update::applyMeasurementUpdate(filter_state_.state,
                                                                  filter_state_.covariance,
                                                                  measurement_system,
                                                                  30.0,
                                                                  6);
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
    return update_result.ok;
}

// ============================================================
// Resolve ambiguities: SD->DD transform + LAMBDA
// ============================================================
bool RTKProcessor::resolveAmbiguities() {
    if (!filter_initialized_) return false;
    if (usesEstimatedIono(rtk_config_)) return false;

    const auto& sat_data = current_sat_data_;

    const int min_lock = std::max(1, rtk_config_.min_lock_count);
    return resolveAmbiguities(buildDoubleDifferencePairs(sat_data, min_lock));
}

bool RTKProcessor::resolveAmbiguities(std::vector<DDPair> dd_pairs) {
    if (!filter_initialized_) return false;
    if (usesEstimatedIono(rtk_config_)) return false;

    const auto& sat_data = current_sat_data_;

    const int na = usesGlonassAutocal(rtk_config_) ? REAL_STATES : BASE_STATES;

    dd_pairs.erase(
        std::remove_if(dd_pairs.begin(), dd_pairs.end(),
                       [&](const DDPair& pair) {
                           return !isAmbiguityResolutionSystem(rtk_config_, pair.ref_sat.system);
                       }),
        dd_pairs.end());

    int nb = dd_pairs.size();
    if (nb < 4) return false;

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
        }
    }

    // Try full set first
    VectorXd dd_fixed;
    double ratio = 0.0;
    bool fixed = false;

    // Use lower ratio threshold when holdamb is active (more confidence in solution)
    double effective_ratio_threshold = rtk_config_.ambiguity_ratio_threshold;
    if (rtk_config_.ar_policy != RTKConfig::ARPolicy::DEMO5_CONTINUOUS) {
        if (consecutive_fix_count_ >= rtk_config_.min_hold_count && has_last_fixed_position_) {
            effective_ratio_threshold = 2.0;
        }
    }

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

    struct WideLaneConstraint {
        int l1_index = -1;
        int l2_index = -1;
        double fixed_integer = 0.0;
    };
    std::vector<WideLaneConstraint> wide_lane_constraints;
    int wide_lane_total = 0;
    int wide_lane_fixed = 0;

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
            if (distanceToNearestInteger(wide_lane_float) >= wide_lane_threshold) {
                continue;
            }

            wide_lane_constraints.push_back({i, l2_pair, fixed_integer});
            wide_lane_fixed++;
        }
        if (wide_lane_total > 0) {
            std::clog << "[RTK-AR] WL fixed " << wide_lane_fixed
                      << "/" << wide_lane_total << "\n";
        }
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
        if (lambdaMethod(full_problem.dd_float, full_problem.Qb, dd_fixed, ratio)) {
            if (ratio >= effective_ratio_threshold) {
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

    // WL-NL fallback: when LAMBDA fails on long baseline, try WL-NL AR
    if (!fixed && rtk_config_.ionoopt == RTKConfig::IonoOpt::IFLC && max_var < 1.0) {
        // Only attempt when KF has converged (max_var < 1 = several epochs in)
        VectorXd wlnl_fixed = dd_float;
        int wl_ok = 0, wl_total = 0;

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
            if (std::abs(dd_mw - nw) > 0.25) continue;

            // IF → NL
            double if_dd = c1_if * rit->second.l1_wavelength * dd_float(i) +
                           c2_if * rit->second.l2_wavelength * dd_float(l2p);
            double n2f = (if_dd - c1_if * rit->second.l1_wavelength * nw) / lam_nl;
            double n2 = std::round(n2f);
            if (std::abs(n2f - n2) > 0.25) continue;

            wlnl_fixed(i) = nw + n2;
            wlnl_fixed(l2p) = n2;
            wl_ok++;
        }

        if (wl_ok >= 3) {
            // Use resolved pairs for position
            std::vector<int> resolved;
            for (int i = 0; i < nb; ++i) {
                if (std::abs(wlnl_fixed(i) - dd_float(i)) > 0.01) resolved.push_back(i);
            }
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
                    dd_fixed = wlnl_fixed;
                    fixed = true;
                    ratio = 999.9;
                    last_dd_pairs_ = dd_pairs;
                    last_best_subset_.clear();
                    for (int i = 0; i < nb; ++i) last_best_subset_.push_back(i);
                    last_dd_fixed_ = dd_fixed;
                }
            }
        }
    }

    // Partial AR: try removing worst satellites if full set fails
    rtk_ar_evaluation::CandidateState best_candidate;
    best_candidate.fixed = fixed;
    best_candidate.ratio = ratio;
    best_candidate.subset.resize(nb);
    for (int i = 0; i < nb; ++i) best_candidate.subset[i] = i;
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

    if (nb > 4 && (search_preferred_subsets || search_drop_subsets)) {
        auto try_subset = [&](const std::vector<int>& subset) {
            const int ns = subset.size();
            if (ns < 4) {
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
            rtk_ar_selection::buildPreferredSubsets(descriptors);
        const bool compare_preferred_subsets =
            usesGlonassAutocal(rtk_config_) ||
            std::any_of(dd_pairs.begin(), dd_pairs.end(), [](const DDPair& pair) {
                return pair.ref_sat.system == GNSSSystem::GLONASS ||
                       pair.ref_sat.system == GNSSSystem::BeiDou;
            });
        if (compare_preferred_subsets && search_preferred_subsets) {
            for (const auto& subset : preferred_subsets) {
                if (subset.size() < 4 || subset.size() >= static_cast<size_t>(nb)) {
                    continue;
                }
                try_subset(subset);
            }
        }

        if (search_drop_subsets) {
            const auto progressive_subsets =
                rtk_ar_selection::buildProgressiveVarianceDropSubsets(descriptors, 4, 6);
            for (const auto& subset : progressive_subsets) {
                try_subset(subset);
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

    if (!fixed) return false;

    // Fixed solution: xa = y[:na] - Qab * Qb^{-1} * (dd_float - dd_fixed)
    const auto fixed_problem = build_search_problem(best_candidate.subset);
    VectorXd xa;
    if (!rtk_ar_evaluation::solveFixedHeadState(fixed_problem.head_state,
                                                fixed_problem.Qab,
                                                fixed_problem.Qb,
                                                fixed_problem.dd_float,
                                                dd_fixed, xa)) {
        return false;
    }
    fixed_baseline_ = xa.head<3>();
    has_fixed_solution_ = true;
    last_ar_ratio_ = ratio;
    last_num_fixed_ambiguities_ = dd_fixed.size();

    // Store fix info for hold and validation
    last_dd_pairs_ = dd_pairs;
    last_best_subset_ = best_candidate.subset;
    last_dd_fixed_ = dd_fixed;

    return true;
}

// ============================================================
// Validate fixed solution
// ============================================================
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
    if (!has_fixed_solution_) return false;

    // Reject fixes that jump too much from the previous fix position
    // This catches wrong integers that pass the ratio test
    Vector3d new_pos = base_position_ + fixed_baseline_;
    if (!isMovingBasePositionMode(rtk_config_) &&
        rtk_validation::exceedsFixHistoryJump(
            new_pos,
            last_fixed_position_,
            has_last_fixed_position_,
            rtk_config_.position_mode == RTKConfig::PositionMode::STATIC,
            consecutive_fix_count_)) {
        return false;
    }
    if (!isMovingBasePositionMode(rtk_config_) &&
        rtk_config_.max_position_jump_m > 0.0 &&
        rtk_validation::exceedsAbsoluteJump(
            new_pos, last_fixed_position_, has_last_fixed_position_,
            rtk_config_.max_position_jump_m)) {
        return false;
    }
    if (!isMovingBasePositionMode(rtk_config_) &&
        rtk_config_.max_position_jump_rate_mps > 0.0 &&
        has_last_fixed_position_ &&
        has_last_fixed_time_ &&
        rtk_validation::exceedsAdaptiveJump(
            new_pos,
            last_fixed_position_,
            current_time - last_fixed_time_,
            rtk_config_.max_position_jump_min_m,
            rtk_config_.max_position_jump_rate_mps)) {
        return false;
    }

    // Sanity check: reject fixes where any component is unreasonably large
    if (fixed_baseline_.norm() > rtk_config_.max_baseline_length) {
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

    const double max_postfix_residual_rms = rtk_config_.max_postfix_residual_rms;
    if (filter_initialized_ &&
        filter_state_.state.size() >= 3 &&
        std::isfinite(max_postfix_residual_rms) &&
        max_postfix_residual_rms > 0.0 &&
        computePostFixResidualRms() > max_postfix_residual_rms) {
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
    if (isMovingBasePositionMode(rtk_config_)) {
        return false;
    }
    if (!rtk_validation::canAttemptHoldFix(consecutive_fix_count_,
                                           rtk_config_.min_hold_count,
                                           has_last_fixed_position_,
                                           last_dd_fixed_.size() > 0)) {
        return false;
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
    if (nb < 4) return false;

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

    if (matched < 4) return false;  // Need at least 4 matched pairs to trust held integers

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
    if (Qb_solver.info() != Eigen::Success) return false;
    VectorXd Qb_inv_db = Qb_solver.solve(db);

    VectorXd xa = head_state - Qab * Qb_inv_db;
    Vector3d test_pos = base_position_ + xa.head<3>();

    // Position validation: only accept if close to last fix
    const double max_hold_jump_m =
        (rtk_config_.position_mode == RTKConfig::PositionMode::STATIC) ? 0.1 : 1.0;
    if (rtk_validation::exceedsAbsoluteJump(
            test_pos, last_fixed_position_, has_last_fixed_position_, max_hold_jump_m)) {
        return false;
    }
    if (rtk_config_.max_hold_divergence_m > 0.0 && filter_state_.state.size() >= 3) {
        const Vector3d float_pos = base_position_ + filter_state_.state.head<3>();
        if ((test_pos - float_pos).norm() > rtk_config_.max_hold_divergence_m) {
            return false;
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
            refresh_trusted = trusted_jump <= std::max(3.0, 6.0 * dt);
        }
    }
    if (refresh_trusted) {
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
    const ObservationData& rover_obs, const ObservationData& base_obs, const NavigationData& nav) {
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

    const auto sat_data =
        const_cast<RTKProcessor*>(this)->collectSatelliteData(rover_obs, base_obs, nav);
    if (sat_data.size() < 4) {
        return {};
    }

    const_cast<RTKProcessor*>(this)->current_sat_data_ = sat_data;
    const double bias_dt =
        has_last_epoch_ ? std::max(rover_obs.time - last_epoch_time_, 1e-3) : 1.0;
    const_cast<RTKProcessor*>(this)->updateBias(sat_data, bias_dt);

    std::vector<DoubleDifference> measurements;
    const auto dd_pairs = buildDoubleDifferencePairs(sat_data, 0);
    if (dd_pairs.empty()) {
        return measurements;
    }

    const Vector3d rover_pos = base_position_ + filter_state_.state.head<3>();
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
        measurement.variance = varerr(measurement.elevation, true);
        measurement.valid = true;
        measurements.push_back(std::move(measurement));
    }

    return measurements;
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
