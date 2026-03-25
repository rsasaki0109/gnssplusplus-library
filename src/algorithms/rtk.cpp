#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/algorithms/kalman.hpp>
#include <libgnss++/algorithms/lambda.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/constants.hpp>
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

bool isBeiDouGeoSatellite(const SatelliteId& sat) {
    if (sat.system != GNSSSystem::BeiDou) {
        return false;
    }
    return (sat.prn >= 1 && sat.prn <= 5) || (sat.prn >= 59 && sat.prn <= 63);
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
    switch (system) {
        case GNSSSystem::GPS:
            return signal == SignalType::GPS_L1CA;
        case GNSSSystem::GLONASS:
            return signal == SignalType::GLO_L1CA || signal == SignalType::GLO_L1P;
        case GNSSSystem::Galileo:
            return signal == SignalType::GAL_E1;
        case GNSSSystem::BeiDou:
            return signal == SignalType::BDS_B1I || signal == SignalType::BDS_B1C;
        case GNSSSystem::QZSS:
            return signal == SignalType::QZS_L1CA;
        default:
            return false;
    }
}

bool isSecondaryRTKSignal(GNSSSystem system, SignalType signal) {
    switch (system) {
        case GNSSSystem::GPS:
            return signal == SignalType::GPS_L2C;
        case GNSSSystem::GLONASS:
            return signal == SignalType::GLO_L2CA || signal == SignalType::GLO_L2P;
        case GNSSSystem::Galileo:
            return signal == SignalType::GAL_E5A;
        case GNSSSystem::BeiDou:
            return signal == SignalType::BDS_B3I ||
                   signal == SignalType::BDS_B2I ||
                   signal == SignalType::BDS_B2A;
        case GNSSSystem::QZSS:
            return signal == SignalType::QZS_L2C;
        default:
            return false;
    }
}

double signalFrequencyHz(SignalType signal, const Ephemeris* eph = nullptr) {
    switch (signal) {
        case SignalType::GPS_L1CA:
        case SignalType::QZS_L1CA:
            return constants::GPS_L1_FREQ;
        case SignalType::GPS_L2C:
        case SignalType::QZS_L2C:
            return constants::GPS_L2_FREQ;
        case SignalType::GPS_L5:
        case SignalType::QZS_L5:
            return constants::GPS_L5_FREQ;
        case SignalType::GLO_L1CA:
        case SignalType::GLO_L1P:
            if (eph && eph->satellite.system == GNSSSystem::GLONASS) {
                return constants::GLO_L1_BASE_FREQ +
                       eph->glonass_frequency_channel * constants::GLO_L1_STEP_FREQ;
            }
            return constants::GLO_L1_BASE_FREQ;
        case SignalType::GLO_L2CA:
        case SignalType::GLO_L2P:
            if (eph && eph->satellite.system == GNSSSystem::GLONASS) {
                return constants::GLO_L2_BASE_FREQ +
                       eph->glonass_frequency_channel * constants::GLO_L2_STEP_FREQ;
            }
            return constants::GLO_L2_BASE_FREQ;
        case SignalType::GAL_E1:
            return constants::GAL_E1_FREQ;
        case SignalType::GAL_E5A:
            return constants::GAL_E5A_FREQ;
        case SignalType::GAL_E5B:
            return constants::GAL_E5B_FREQ;
        case SignalType::GAL_E6:
            return constants::GAL_E6_FREQ;
        case SignalType::BDS_B1I:
            return constants::BDS_B1I_FREQ;
        case SignalType::BDS_B2I:
            return constants::BDS_B2I_FREQ;
        case SignalType::BDS_B3I:
            return constants::BDS_B3I_FREQ;
        case SignalType::BDS_B1C:
            return constants::BDS_B1C_FREQ;
        case SignalType::BDS_B2A:
            return constants::BDS_B2A_FREQ;
        default:
            return 0.0;
    }
}

double signalWavelengthM(SignalType signal, const Ephemeris* eph = nullptr) {
    const double frequency = signalFrequencyHz(signal, eph);
    if (frequency <= 0.0) {
        return 0.0;
    }
    return constants::SPEED_OF_LIGHT / frequency;
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
    filter_state_.next_state_idx = REAL_STATES;
    ambiguity_states_.clear();
    lock_count_l1_.clear();
    lock_count_l2_.clear();
    has_fixed_solution_ = false;
    has_last_fixed_position_ = false;
    has_last_solution_position_ = false;
    has_last_trusted_position_ = false;
    has_ref_satellite_ = false;
    has_last_epoch_ = false;
    has_last_trusted_time_ = false;
    current_sat_data_.clear();
    gf_l1l2_history_.clear();
    consecutive_fix_count_ = 0;
    last_ar_ratio_ = 0.0;
    last_num_fixed_ambiguities_ = 0;
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

void RTKProcessor::removeSatelliteFromState(const SatelliteId& sat) {
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
    std::map<SatelliteId, const Observation*> rover_l1, rover_l2, base_l1, base_l2;
    for (const auto& obs : rover_obs.observations) {
        if (!isEnabledRTKSystem(rtk_config_, obs.satellite.system)) continue;
        if (!isUsableRTKSatellite(obs.satellite)) continue;
        if (isPrimaryRTKSignal(obs.satellite.system, obs.signal) &&
            obs.has_carrier_phase && obs.has_pseudorange) {
            rover_l1[obs.satellite] = &obs;
        }
        if (isSecondaryRTKSignal(obs.satellite.system, obs.signal) &&
            obs.has_carrier_phase && obs.has_pseudorange) {
            rover_l2[obs.satellite] = &obs;
        }
    }
    for (const auto& obs : base_obs.observations) {
        if (!isEnabledRTKSystem(rtk_config_, obs.satellite.system)) continue;
        if (!isUsableRTKSatellite(obs.satellite)) continue;
        if (isPrimaryRTKSignal(obs.satellite.system, obs.signal) &&
            obs.has_carrier_phase && obs.has_pseudorange) {
            base_l1[obs.satellite] = &obs;
        }
        if (isSecondaryRTKSignal(obs.satellite.system, obs.signal) &&
            obs.has_carrier_phase && obs.has_pseudorange) {
            base_l2[obs.satellite] = &obs;
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
        for (const auto& [sat, r_obs] : rover_l1) {
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
    for (const auto& [sat, r_obs] : rover_l1) {
        auto b_it = base_l1.find(sat);
        if (b_it == base_l1.end()) continue;
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
            double base_pr = b_it->second->pseudorange;
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
        sd.l1_wavelength = signalWavelengthM(sd.l1_signal, eph);
        if (sd.l1_wavelength <= 0.0) continue;
        sd.rover_l1_phase = r_obs->carrier_phase; sd.rover_l1_code = r_obs->pseudorange;
        sd.base_l1_phase = b_it->second->carrier_phase; sd.base_l1_code = b_it->second->pseudorange;
        sd.has_l1 = true; sd.l1_lli = r_obs->lli | b_it->second->lli;
        auto r_l2 = rover_l2.find(sat); auto b_l2 = base_l2.find(sat);
        if (r_l2 != rover_l2.end() && b_l2 != base_l2.end()) {
            sd.l2_signal = r_l2->second->signal;
            sd.l2_frequency_hz = signalFrequencyHz(sd.l2_signal, eph);
            sd.l2_wavelength = signalWavelengthM(sd.l2_signal, eph);
            sd.rover_l2_phase = r_l2->second->carrier_phase; sd.rover_l2_code = r_l2->second->pseudorange;
            sd.base_l2_phase = b_l2->second->carrier_phase; sd.base_l2_code = b_l2->second->pseudorange;
            sd.has_l2 = sd.l2_wavelength > 0.0;
            sd.l2_lli = r_l2->second->lli | b_l2->second->lli;
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
    SatelliteId best_dual;
    SatelliteId best_l1;
    double best_dual_el = -1.0;
    double best_l1_el = -1.0;

    auto has_required_lock = [&](const SatelliteId& sat, int freq) {
        if (min_lock_count <= 0) return true;
        const auto& lock_counts = (freq == 0) ? lock_count_l1_ : lock_count_l2_;
        auto it = lock_counts.find(sat);
        return it != lock_counts.end() && it->second >= min_lock_count;
    };

    for (const auto& [sat, sd] : sat_data) {
        if (sat.system != system || !sd.has_l1) continue;
        auto n1_it = filter_state_.n1_indices.find(sat);
        if (n1_it == filter_state_.n1_indices.end() || filter_state_.state(n1_it->second) == 0.0) continue;
        if (!has_required_lock(sat, 0)) continue;

        if (sd.elevation > best_l1_el) {
            best_l1_el = sd.elevation;
            best_l1 = sat;
        }

        auto n2_it = filter_state_.n2_indices.find(sat);
        if (!sd.has_l2 || n2_it == filter_state_.n2_indices.end()) continue;
        if (filter_state_.state(n2_it->second) == 0.0) continue;
        if (!has_required_lock(sat, 1)) continue;
        if (sd.elevation > best_dual_el) {
            best_dual_el = sd.elevation;
            best_dual = sat;
        }
    }

    if (best_dual_el >= 0.0) {
        ref_sat = best_dual;
        return true;
    }
    if (best_l1_el >= 0.0) {
        ref_sat = best_l1;
        return true;
    }
    return false;
}

std::vector<RTKProcessor::DDPair> RTKProcessor::buildDoubleDifferencePairs(
    const std::map<SatelliteId, SatelliteData>& sat_data,
    int min_lock_count) const {
    std::vector<DDPair> dd_pairs;

    for (GNSSSystem system : kRTKSupportedSystems) {
        if (!isEnabledRTKSystem(rtk_config_, system)) continue;
        SatelliteId ref_sat;
        if (!selectSystemReferenceSatellite(sat_data, system, min_lock_count, ref_sat)) continue;

        const auto ref_sd_it = sat_data.find(ref_sat);
        if (ref_sd_it == sat_data.end()) continue;
        const auto& ref_sd = ref_sd_it->second;

        auto ref_n1_it = filter_state_.n1_indices.find(ref_sat);
        if (ref_n1_it != filter_state_.n1_indices.end() && ref_sd.has_l1) {
            for (const auto& [sat, sd] : sat_data) {
                if (sat.system != system || sat == ref_sat || !sd.has_l1) continue;
                if (requiresMatchedCarrierWavelength(rtk_config_, system) &&
                    std::abs(sd.l1_wavelength - ref_sd.l1_wavelength) > 1e-6) {
                    continue;
                }
                auto n1_it = filter_state_.n1_indices.find(sat);
                if (n1_it == filter_state_.n1_indices.end() || filter_state_.state(n1_it->second) == 0.0) continue;
                if (min_lock_count > 0) {
                    auto lock_it = lock_count_l1_.find(sat);
                    if (lock_it == lock_count_l1_.end() || lock_it->second < min_lock_count) continue;
                }
                dd_pairs.push_back({ref_sat, ref_n1_it->second, n1_it->second, sat, 0});
            }
        }

        auto ref_n2_it = filter_state_.n2_indices.find(ref_sat);
        if (ref_n2_it != filter_state_.n2_indices.end() && ref_sd.has_l2) {
            for (const auto& [sat, sd] : sat_data) {
                if (sat.system != system || sat == ref_sat || !sd.has_l2) continue;
                if (requiresMatchedCarrierWavelength(rtk_config_, system) &&
                    std::abs(sd.l2_wavelength - ref_sd.l2_wavelength) > 1e-6) {
                    continue;
                }
                auto n2_it = filter_state_.n2_indices.find(sat);
                if (n2_it == filter_state_.n2_indices.end() || filter_state_.state(n2_it->second) == 0.0) continue;
                if (min_lock_count > 0) {
                    auto lock_it = lock_count_l2_.find(sat);
                    if (lock_it == lock_count_l2_.end() || lock_it->second < min_lock_count) continue;
                }
                dd_pairs.push_back({ref_sat, ref_n2_it->second, n2_it->second, sat, 1});
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
    filter_state_.n1_indices.clear();
    filter_state_.n2_indices.clear();
    filter_state_.next_state_idx = REAL_STATES;

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
void RTKProcessor::updateBias(const std::map<SatelliteId, SatelliteData>& sat_data) {
    std::vector<SatelliteId> sats_to_remove;
    for (const auto& [sat, idx] : filter_state_.n1_indices) {
        if (sat_data.find(sat) == sat_data.end()) sats_to_remove.push_back(sat);
    }
    for (const auto& sat : sats_to_remove) {
        removeSatelliteFromState(sat);
        lock_count_l1_.erase(sat);
        lock_count_l2_.erase(sat);
        gf_l1l2_history_.erase(sat);
    }

    std::set<SatelliteId> gf_slips;
    if (rtk_config_.enable_cycle_slip_detection) {
        for (const auto& [sat, sd] : sat_data) {
            if (!sd.has_l1 || !sd.has_l2 || sd.l1_wavelength <= 0.0 || sd.l2_wavelength <= 0.0) continue;
            double gf = (sd.rover_l1_phase - sd.base_l1_phase) * sd.l1_wavelength -
                        (sd.rover_l2_phase - sd.base_l2_phase) * sd.l2_wavelength;
            auto prev_it = gf_l1l2_history_.find(sat);
            if (prev_it != gf_l1l2_history_.end() &&
                std::abs(gf - prev_it->second) > rtk_config_.cycle_slip_threshold) {
                gf_slips.insert(sat);
            }
            gf_l1l2_history_[sat] = gf;
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
            bool slip = (lli & 0x01) != 0 || gf_slips.find(sat) != gf_slips.end();
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

void RTKProcessor::resetPositionToSPP(const ObservationData& rover_obs, const NavigationData& nav) {
    if (rtk_config_.position_mode == RTKConfig::PositionMode::STATIC) {
        // Static: position accumulates with process noise
        double pos_pnoise = rtk_config_.process_noise_position;  // default 1e-4 m^2/s
        for (int i = 0; i < BASE_STATES; ++i)
            filter_state_.covariance(i, i) += pos_pnoise;
        return;
    }

    // Kinematic mode: reset position with large variance
    Vector3d rover_pos;
    double var_pos = 900.0;  // RTKLIB VAR_POS = SQR(30)
    auto spp = spp_processor_.processEpoch(rover_obs, nav);
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

    try {
        if (!base_position_known_) {
            auto spp = spp_processor_.processEpoch(rover_obs, nav);
            rememberSolution(spp);
            consecutive_fix_count_ = 0;
            return spp;
        }

        auto current_spp = spp_processor_.processEpoch(rover_obs, nav);
        auto fallback_spp = [&]() {
            last_ar_ratio_ = 0.0;
            last_num_fixed_ambiguities_ = 0;
            auto spp = current_spp;
            if (spp.isValid() && has_last_trusted_position_ && has_last_trusted_time_) {
                const double trusted_jump =
                    (spp.position_ecef - last_trusted_position_).norm();
                if (spp.num_satellites <= 5 && trusted_jump > 25.0) {
                    spp = PositionSolution{};
                    spp.time = rover_obs.time;
                    spp.status = SolutionStatus::NONE;
                }
            }
            if (spp.isValid() && has_last_solution_position_ && has_last_epoch_) {
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
            consecutive_fix_count_ = 0;
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
        updateBias(sat_data);

        // Iterative KF update
        bool filter_ok = false;
        for (int iter = 0; iter < rtk_config_.kf_iterations; ++iter) {
            const Vector3d baseline_before_iter = filter_state_.state.head<3>();
            filter_ok = updateFilter(sat_data);
            if (!filter_ok) break;
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

            if (!finitePosition(solution) || deviatesTooFarFromSPP(solution, 150.0)) {
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

            if (n_sats <= 4 && has_last_trusted_position_ && has_last_trusted_time_) {
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

            if (saved_has_last_trusted && saved_has_last_trusted_time) {
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

            if (saved_has_last_solution && saved_has_last_solution_time) {
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

            if (have_fix_candidate && has_fixed_solution_) {
                if (validateFixedSolution(sat_data)) {
                    Vector3d saved_baseline = filter_state_.state.head<3>();
                    filter_state_.state.head<3>() = fixed_baseline_;
                    solution = generateSolution(rover_obs.time, SolutionStatus::FIXED, n_sats);
                    filter_state_.state.head<3>() = saved_baseline;
                    const double fixed_float_jump =
                        (solution.position_ecef - float_solution.position_ecef).norm();
                    double fixed_trusted_jump = 0.0;
                    double max_fixed_trusted_jump = 0.0;
                    bool exceeds_trusted_jump = false;
                    if (saved_has_last_trusted && saved_has_last_trusted_time) {
                        double dt = rover_obs.time - saved_last_trusted_time;
                        if (!std::isfinite(dt) || dt < 0.5) dt = 1.0;
                        fixed_trusted_jump =
                            (solution.position_ecef - saved_last_trusted_position).norm();
                        max_fixed_trusted_jump = std::max(20.0, 25.0 * dt);
                        exceeds_trusted_jump = fixed_trusted_jump > max_fixed_trusted_jump;
                    }
                    if (!finitePosition(solution) ||
                        deviatesTooFarFromSPP(solution, 150.0) ||
                        fixed_float_jump > 20.0 ||
                        exceeds_trusted_jump) {
                        has_fixed_solution_ = false;
                        last_trusted_position_ = saved_last_trusted_position;
                        has_last_trusted_position_ = saved_has_last_trusted;
                        last_trusted_time_ = saved_last_trusted_time;
                        has_last_trusted_time_ = saved_has_last_trusted_time;
                        rememberSolution(float_solution);
                        updateStatistics(SolutionStatus::FLOAT);
                        consecutive_fix_count_ = 0;
                        return float_solution;
                    }
                    updateStatistics(SolutionStatus::FIXED);
                    consecutive_fix_count_++;

                    // Save fixed position for next epoch's position reset
                    last_fixed_position_ = base_position_ + fixed_baseline_;
                    has_last_fixed_position_ = true;

                    // holdamb: constrain SD ambiguities toward validated DD integers
                    if (consecutive_fix_count_ >= rtk_config_.min_hold_count) {
                        applyHoldAmbiguity();
                    }
                } else {
                    has_fixed_solution_ = false;
                    updateStatistics(SolutionStatus::FLOAT);
                    consecutive_fix_count_ = 0;
                }
            } else {
                updateStatistics(SolutionStatus::FLOAT);
                consecutive_fix_count_ = 0;
            }
        } else {
            return fallback_spp();
        }
    } catch (const std::exception& e) {
        std::cerr << "RTK exception: " << e.what() << std::endl;
        auto spp = spp_processor_.processEpoch(rover_obs, nav);
        rememberSolution(spp);
        consecutive_fix_count_ = 0;
        return spp;
    }
    return solution;
}

// ============================================================
// KF update: DD observation model with H mapping to SD states
// ============================================================
bool RTKProcessor::updateFilter(const std::map<SatelliteId, SatelliteData>& sat_data) {

    if (sat_data.size() < 4) return false;

    int n_states = filter_state_.state.size();
    int max_obs = 4 * sat_data.size();
    MatrixXd H = MatrixXd::Zero(max_obs, n_states);
    VectorXd z(max_obs);
    std::vector<double> Ri_vec, Rj_vec;
    std::vector<int> nb_vec;

    Vector3d rover_pos = base_position_ + filter_state_.state.head<3>();

    int oi = 0;
    for (GNSSSystem system : kRTKSupportedSystems) {
        if (!isEnabledRTKSystem(rtk_config_, system)) continue;
        SatelliteId ref_sat;
        if (!selectSystemReferenceSatellite(sat_data, system, 0, ref_sat)) continue;

        auto ref_it = sat_data.find(ref_sat);
        if (ref_it == sat_data.end()) continue;
        const auto& ref_sd = ref_it->second;
        const double rr_ref = geodist_range(ref_sd.sat_pos, rover_pos) +
                              tropModel(rover_pos, ref_sd.elevation);
        const double br_ref = geodist_range(ref_sd.sat_pos_base, base_position_) +
                              tropModel(base_position_, ref_sd.base_elevation);
        const Vector3d los_ref = (ref_sd.sat_pos - rover_pos).normalized();

        auto append_block = [&](int freq, bool is_phase) {
            const bool ref_has_freq = (freq == 0) ? ref_sd.has_l1 : ref_sd.has_l2;
            if (!ref_has_freq) {
                nb_vec.push_back(0);
                return;
            }

            const auto& ref_indices = (freq == 0) ? filter_state_.n1_indices : filter_state_.n2_indices;
            auto ref_state_it = ref_indices.find(ref_sat);
            if (ref_state_it == ref_indices.end()) {
                nb_vec.push_back(0);
                return;
            }

            const double ref_wavelength = (freq == 0) ? ref_sd.l1_wavelength : ref_sd.l2_wavelength;
            if (ref_wavelength <= 0.0) {
                nb_vec.push_back(0);
                return;
            }

            int block_count = 0;
            for (const auto& [sat, sd] : sat_data) {
                if (sat.system != system || sat == ref_sat) continue;
                const bool sat_has_freq = (freq == 0) ? sd.has_l1 : sd.has_l2;
                if (!sat_has_freq) continue;
                const auto& sat_indices = (freq == 0) ? filter_state_.n1_indices : filter_state_.n2_indices;
                auto sat_state_it = sat_indices.find(sat);
                if (sat_state_it == sat_indices.end()) continue;

                const double sat_wavelength = (freq == 0) ? sd.l1_wavelength : sd.l2_wavelength;
                if (sat_wavelength <= 0.0) continue;

                const double rr = geodist_range(sd.sat_pos, rover_pos) + tropModel(rover_pos, sd.elevation);
                const double br = geodist_range(sd.sat_pos_base, base_position_) +
                                  tropModel(base_position_, sd.base_elevation);
                const double geom_dd = (rr_ref - br_ref) - (rr - br);
                const Vector3d dd_los = -los_ref + (sd.sat_pos - rover_pos).normalized();

                if (is_phase) {
                    const double ref_phase = (freq == 0) ? ref_sd.rover_l1_phase - ref_sd.base_l1_phase
                                                         : ref_sd.rover_l2_phase - ref_sd.base_l2_phase;
                    const double sat_phase = (freq == 0) ? sd.rover_l1_phase - sd.base_l1_phase
                                                         : sd.rover_l2_phase - sd.base_l2_phase;
                    const double glonass_icb =
                        glonassInterChannelBiasMeters(
                            rtk_config_,
                            ref_sd.satellite.system,
                            sd.satellite.system,
                            (freq == 0) ? ref_sd.l1_frequency_hz : ref_sd.l2_frequency_hz,
                            (freq == 0) ? sd.l1_frequency_hz : sd.l2_frequency_hz,
                            freq);
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
                    const double amb_term =
                        ref_wavelength * filter_state_.state(ref_state_it->second) -
                        sat_wavelength * filter_state_.state(sat_state_it->second);
                    z(oi) = ref_phase * ref_wavelength - sat_phase * sat_wavelength -
                            geom_dd - amb_term - glonass_icb;
                    if (autocal_glonass) {
                        z(oi) -= df_mhz * filter_state_.state(IL(freq));
                        H(oi, IL(freq)) = df_mhz;
                    }
                    H(oi, ref_state_it->second) = ref_wavelength;
                    H(oi, sat_state_it->second) = -sat_wavelength;
                } else {
                    const double ref_code = (freq == 0) ? ref_sd.rover_l1_code - ref_sd.base_l1_code
                                                        : ref_sd.rover_l2_code - ref_sd.base_l2_code;
                    const double sat_code = (freq == 0) ? sd.rover_l1_code - sd.base_l1_code
                                                        : sd.rover_l2_code - sd.base_l2_code;
                    z(oi) = (ref_code - sat_code) - geom_dd;
                }

                H(oi, 0) = dd_los(0);
                H(oi, 1) = dd_los(1);
                H(oi, 2) = dd_los(2);
                Ri_vec.push_back(varerr(ref_sd.elevation, is_phase));
                Rj_vec.push_back(varerr(sd.elevation, is_phase));
                oi++;
                block_count++;
            }
            nb_vec.push_back(block_count);
        };

        append_block(0, true);
        append_block(1, true);
        append_block(0, false);
        append_block(1, false);
    }

    if (oi < 6) return false;

    // Build DD error covariance R (RTKLIB ddcov)
    MatrixXd R = MatrixXd::Zero(oi, oi);
    {
        int k = 0;
        for (int b = 0; b < (int)nb_vec.size(); ++b) {
            for (int i = 0; i < nb_vec[b]; ++i) {
                for (int j = 0; j < nb_vec[b]; ++j) {
                    R(k + i, k + j) = Ri_vec[k + i];
                    if (i == j) R(k + i, k + j) += Rj_vec[k + i];
                }
            }
            k += nb_vec[b];
        }
    }

    // Reject outliers (RTKLIB maxinno check)
    for (int i = 0; i < oi; ++i) {
        if (std::abs(z(i)) > 30.0) {
            z(i) = 0.0;
            H.row(i).setZero();
        }
    }

    // Native Eigen Kalman filter update
    {
        MatrixXd H_active = H.topRows(oi);
        VectorXd v_active = z.head(oi);
        MatrixXd R_active = R.topLeftCorner(oi, oi);

        int info = kalmanFilter(filter_state_.state, filter_state_.covariance,
                                H_active, v_active, R_active);
        if (info) return false;
    }

    return true;
}

// ============================================================
// Resolve ambiguities: SD->DD transform + LAMBDA
// ============================================================
bool RTKProcessor::resolveAmbiguities() {
    if (!filter_initialized_) return false;

    const auto& sat_data = current_sat_data_;

    const int min_lock = std::max(1, rtk_config_.min_lock_count);
    return resolveAmbiguities(buildDoubleDifferencePairs(sat_data, min_lock));
}

bool RTKProcessor::resolveAmbiguities(std::vector<DDPair> dd_pairs) {
    if (!filter_initialized_) return false;

    const auto& sat_data = current_sat_data_;

    int nx = filter_state_.state.size();
    const int na = usesGlonassAutocal(rtk_config_) ? REAL_STATES : BASE_STATES;

    dd_pairs.erase(
        std::remove_if(dd_pairs.begin(), dd_pairs.end(),
                       [&](const DDPair& pair) {
                           return !isAmbiguityResolutionSystem(rtk_config_, pair.ref_sat.system);
                       }),
        dd_pairs.end());

    int nb = dd_pairs.size();
    if (nb < 4) return false;


    int ny = na + nb;

    // Build D matrix (SD to DD transform)
    MatrixXd D = MatrixXd::Zero(nx, ny);
    for (int i = 0; i < na; ++i) D(i, i) = 1.0;
    for (int i = 0; i < nb; ++i) {
        D(dd_pairs[i].ref_idx, na + i) = 1.0;
        D(dd_pairs[i].sat_idx, na + i) = -1.0;
    }

    // Transform: y = D' * x, Qy = D' * P * D
    VectorXd y = D.transpose() * filter_state_.state;
    MatrixXd DP = D.transpose() * filter_state_.covariance;
    MatrixXd Qy = DP * D;

    // Extract Qb (DD amb cov) and Qab (cross-cov)
    MatrixXd Qb(nb, nb);
    MatrixXd Qab(na, nb);
    for (int i = 0; i < nb; ++i)
        for (int j = 0; j < nb; ++j)
            Qb(i, j) = Qy(na + i, na + j);
    for (int i = 0; i < na; ++i)
        for (int j = 0; j < nb; ++j)
            Qab(i, j) = Qy(i, na + j);

    Qb = (Qb + Qb.transpose()) / 2.0;
    for (int i = 0; i < nb; ++i)
        if (Qb(i, i) < 1e-6) Qb(i, i) = 1e-6;

    // Variance check
    double max_var = 0;
    for (int i = 0; i < nb; ++i) max_var = std::max(max_var, Qb(i, i));

    VectorXd dd_float = y.tail(nb);

    // Exclude DD pairs with outlier variance (relative to median)
    // This removes newly-appearing satellites that haven't converged
    {
        std::vector<double> vars(nb);
        for (int i = 0; i < nb; ++i) vars[i] = Qb(i, i);
        std::sort(vars.begin(), vars.end());
        double median_var = vars[nb / 2];
        double var_threshold = std::max(median_var * 10.0, 1e-4);  // at least 10x median

        std::vector<int> good_pairs;
        for (int i = 0; i < nb; ++i) {
            if (Qb(i, i) <= var_threshold) good_pairs.push_back(i);
        }
        if ((int)good_pairs.size() < 4) good_pairs.clear();  // keep all if too few good

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
    if (consecutive_fix_count_ >= rtk_config_.min_hold_count && has_last_fixed_position_) {
        effective_ratio_threshold = 2.0;
    }

    // Standard LAMBDA path
    if (!fixed) {
        if (lambdaMethod(dd_float, Qb, dd_fixed, ratio)) {
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
                    VectorXd xa = y.head(na) - sQab * slv.solve(sf - sx);
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
    std::vector<int> best_subset;
    for (int i = 0; i < nb; ++i) best_subset.push_back(i);

    const bool search_preferred_subsets =
        !fixed || ratio < effective_ratio_threshold + 0.5;
    const bool search_drop_subsets =
        !fixed || ratio < effective_ratio_threshold + 0.8 || max_var > 0.20;

    if (nb > 4 && (search_preferred_subsets || search_drop_subsets)) {
        const VectorXd full_dd_float = dd_float;
        const MatrixXd full_Qb = Qb;
        const MatrixXd full_Qab = Qab;
        double best_ratio = fixed ? ratio : 0.0;
        std::vector<int> current_subset = best_subset;
        VectorXd best_dd_float = full_dd_float;
        MatrixXd best_Qb = full_Qb;
        MatrixXd best_Qab = full_Qab;
        VectorXd best_dd_fixed = dd_fixed;
        bool best_fixed = fixed;

        auto adopt_subset = [&](const std::vector<int>& subset,
                                const VectorXd& sub_float,
                                const MatrixXd& sub_Qb,
                                const MatrixXd& sub_Qab,
                                const VectorXd& sub_fixed,
                                double sub_ratio) {
            best_subset = subset;
            best_dd_float = sub_float;
            best_Qb = sub_Qb;
            best_Qab = sub_Qab;
            best_dd_fixed = sub_fixed;
            best_ratio = sub_ratio;
            best_fixed = true;
        };

        auto try_subset = [&](const std::vector<int>& subset) {
            const int ns = subset.size();
            if (ns < 4) {
                return false;
            }

            VectorXd sub_float(ns);
            MatrixXd sub_Qb(ns, ns);
            MatrixXd sub_Qab(na, ns);
            for (int i = 0; i < ns; ++i) {
                sub_float(i) = full_dd_float(subset[i]);
                for (int j = 0; j < ns; ++j) {
                    sub_Qb(i, j) = full_Qb(subset[i], subset[j]);
                }
                for (int j = 0; j < na; ++j) {
                    sub_Qab(j, i) = full_Qab(j, subset[i]);
                }
            }
            sub_Qb = (sub_Qb + sub_Qb.transpose()) / 2.0;

            VectorXd sub_fixed;
            double sub_ratio = 0.0;
            if (!lambdaMethod(sub_float, sub_Qb, sub_fixed, sub_ratio)) {
                return false;
            }
            if (sub_ratio < effective_ratio_threshold) {
                return false;
            }

            if (!best_fixed || sub_ratio > best_ratio + 1e-6) {
                adopt_subset(subset, sub_float, sub_Qb, sub_Qab, sub_fixed, sub_ratio);
                return true;
            }
            return false;
        };

        auto build_subset_excluding = [&](std::initializer_list<GNSSSystem> excluded) {
            std::vector<int> subset;
            for (int i = 0; i < nb; ++i) {
                bool keep = true;
                for (GNSSSystem system : excluded) {
                    if (dd_pairs[i].ref_sat.system == system) {
                        keep = false;
                        break;
                    }
                }
                if (keep) {
                    subset.push_back(i);
                }
            }
            return subset;
        };

        const std::vector<std::vector<int>> preferred_subsets = {
            build_subset_excluding({GNSSSystem::GLONASS, GNSSSystem::BeiDou}),
            build_subset_excluding({GNSSSystem::GLONASS}),
            build_subset_excluding({GNSSSystem::BeiDou}),
        };
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
            for (int drop = 0; drop < std::min(nb - 4, 6); ++drop) {
                int worst = -1;
                double worst_var = -1;
                for (int idx : current_subset) {
                    if (full_Qb(idx, idx) > worst_var) { worst_var = full_Qb(idx, idx); worst = idx; }
                }
                if (worst < 0) break;

                std::vector<int> subset;
                for (int idx : current_subset) {
                    if (idx != worst) subset.push_back(idx);
                }
                if (subset.size() < 4) break;

                try_subset(subset);
                current_subset = subset;
            }
        }

        if (best_fixed) {
            fixed = true;
            dd_fixed = best_dd_fixed;
            ratio = best_ratio;
            Qab = best_Qab;
            Qb = best_Qb;
            dd_float = best_dd_float;
        } else if (!fixed && best_subset.size() < static_cast<size_t>(nb)) {
            Qab = best_Qab;
            Qb = best_Qb;
            dd_float = best_dd_float;
        }
    }

    if (!fixed) return false;

    // Fixed solution: xa = y[:na] - Qab * Qb^{-1} * (dd_float - dd_fixed)
    VectorXd db = dd_float - dd_fixed;
    Eigen::LDLT<MatrixXd> Qb_solver(Qb);
    if (Qb_solver.info() != Eigen::Success) return false;
    VectorXd Qb_inv_db = Qb_solver.solve(db);

    VectorXd xa = y.head(na) - Qab * Qb_inv_db;
    fixed_baseline_ = xa.head<3>();
    has_fixed_solution_ = true;
    last_ar_ratio_ = ratio;
    last_num_fixed_ambiguities_ = dd_fixed.size();

    // Store fix info for hold and validation
    last_dd_pairs_ = dd_pairs;
    last_best_subset_ = best_subset;
    last_dd_fixed_ = dd_fixed;

    return true;
}

// ============================================================
// Validate fixed solution
// ============================================================
bool RTKProcessor::validateFixedSolution(const std::map<SatelliteId, SatelliteData>& sat_data) {
    if (!has_fixed_solution_) return false;

    // Reject fixes that jump too much from the previous fix position
    // This catches wrong integers that pass the ratio test
    if (has_last_fixed_position_) {
        Vector3d new_pos = base_position_ + fixed_baseline_;
        double jump = (new_pos - last_fixed_position_).norm();
        // Use strict threshold for static, relaxed for kinematic
        double max_jump = (rtk_config_.position_mode == RTKConfig::PositionMode::STATIC) ? 0.1 : 0.2;
        if (jump > max_jump) {
            // Don't reject if this is a fresh start (no consecutive fixes yet)
            // because the initial fix might be far from SPP position
            if (consecutive_fix_count_ >= 3) {
                return false;
            }
        }
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
    if (last_dd_fixed_.size() == 0 || !has_last_fixed_position_) return false;

    int nx = filter_state_.state.size();
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

    if (matched < 4) return false;  // Need at least 4 matched pairs

    // Compute fixed baseline using xa = y[:na] - Qab * Qb^-1 * (dd_float - dd_fixed)
    // Build D matrix
    MatrixXd D = MatrixXd::Zero(nx, na + nb);
    for (int i = 0; i < na; ++i) D(i, i) = 1.0;
    for (int i = 0; i < nb; ++i) {
        D(dd_pairs[i].ref_idx, na + i) = 1.0;
        D(dd_pairs[i].sat_idx, na + i) = -1.0;
    }

    VectorXd y = D.transpose() * filter_state_.state;
    MatrixXd Qy = D.transpose() * filter_state_.covariance * D;

    MatrixXd Qb(nb, nb);
    MatrixXd Qab(na, nb);
    for (int i = 0; i < nb; ++i)
        for (int j = 0; j < nb; ++j)
            Qb(i, j) = Qy(na + i, na + j);
    for (int i = 0; i < na; ++i)
        for (int j = 0; j < nb; ++j)
            Qab(i, j) = Qy(i, na + j);
    Qb = (Qb + Qb.transpose()) / 2.0;
    for (int i = 0; i < nb; ++i)
        if (Qb(i, i) < 1e-6) Qb(i, i) = 1e-6;

    VectorXd dd_float_v = y.tail(nb);
    VectorXd db = dd_float_v - dd_fixed;
    Eigen::LDLT<MatrixXd> Qb_solver(Qb);
    if (Qb_solver.info() != Eigen::Success) return false;
    VectorXd Qb_inv_db = Qb_solver.solve(db);

    VectorXd xa = y.head(na) - Qab * Qb_inv_db;
    Vector3d test_pos = base_position_ + xa.head<3>();

    // Position validation: only accept if close to last fix
    double pos_diff = (test_pos - last_fixed_position_).norm();
    if (pos_diff > 0.3) return false;

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
        if (!has_last_trusted_position_ || !has_last_trusted_time_) {
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

    // Regularize covariance to ensure positive-definiteness for LAMBDA
    MatrixXd Q_reg = covariance;
    Q_reg = (Q_reg + Q_reg.transpose()) * 0.5;

    // Ensure minimum diagonal values
    constexpr double MIN_VAR = 1e-6;
    for (int i = 0; i < n; ++i) {
        if (Q_reg(i, i) < MIN_VAR) Q_reg(i, i) = MIN_VAR;
    }

    // Check positive-definiteness via Cholesky; if it fails, add ridge
    Eigen::LLT<MatrixXd> llt(Q_reg);
    if (llt.info() != Eigen::Success) {
        Eigen::SelfAdjointEigenSolver<MatrixXd> eig(Q_reg);
        double min_eig = eig.eigenvalues().minCoeff();
        if (min_eig < MIN_VAR) {
            Q_reg += MatrixXd::Identity(n, n) * (MIN_VAR - min_eig);
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
    const_cast<RTKProcessor*>(this)->updateBias(sat_data);

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
    return has_fixed_solution_ && validateFixedSolution(sat_data);
}

} // namespace libgnss
