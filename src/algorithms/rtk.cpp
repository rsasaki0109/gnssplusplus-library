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

// Convenience aliases for wavelengths and IFLC coefficients from constants.hpp
static constexpr double LAMBDA_L1 = constants::GPS_L1_WAVELENGTH;
static constexpr double LAMBDA_L2 = constants::GPS_L2_WAVELENGTH;
static constexpr double IFLC_C1   = constants::IFLC_C1;
static constexpr double IFLC_C2   = constants::IFLC_C2;

// Delegate to extracted modules
static inline double tropModel(const Vector3d& pos_ecef, double elevation) {
    return models::tropDelaySaastamoinen(pos_ecef, elevation);
}

// Geometric range with Sagnac correction (delegated to coordinates.hpp)
static inline double geodist_range(const Vector3d& rs, const Vector3d& rr) {
    return geodist(rs, rr);
}

RTKProcessor::RTKProcessor() : spp_processor_() { filter_initialized_ = false; }
RTKProcessor::RTKProcessor(const RTKConfig& rtk_config) : rtk_config_(rtk_config), spp_processor_() { filter_initialized_ = false; }

bool RTKProcessor::initialize(const ProcessorConfig& config) {
    config_ = config;
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
    filter_state_.next_state_idx = BASE_STATES;
    ambiguity_states_.clear();
    lock_count_l1_.clear();
    lock_count_l2_.clear();
    has_fixed_solution_ = false;
    has_last_fixed_position_ = false;
    has_last_solution_position_ = false;
    has_ref_satellite_ = false;
    has_last_epoch_ = false;
    current_sat_data_.clear();
    gf_l1l2_history_.clear();
    consecutive_fix_count_ = 0;
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
    int idx = IB(sat.prn, 0);
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
    int idx = IB(sat.prn, 1);
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
        if (obs.signal == SignalType::GPS_L1CA && obs.has_carrier_phase && obs.has_pseudorange) rover_l1[obs.satellite] = &obs;
        if (obs.signal == SignalType::GPS_L2C && obs.has_carrier_phase && obs.has_pseudorange) rover_l2[obs.satellite] = &obs;
    }
    for (const auto& obs : base_obs.observations) {
        if (obs.signal == SignalType::GPS_L1CA && obs.has_carrier_phase && obs.has_pseudorange) base_l1[obs.satellite] = &obs;
        if (obs.signal == SignalType::GPS_L2C && obs.has_carrier_phase && obs.has_pseudorange) base_l2[obs.satellite] = &obs;
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

        // Compute base satellite position from base pseudorange travel time
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
        sd.rover_l1_phase = r_obs->carrier_phase; sd.rover_l1_code = r_obs->pseudorange;
        sd.base_l1_phase = b_it->second->carrier_phase; sd.base_l1_code = b_it->second->pseudorange;
        sd.has_l1 = true; sd.l1_lli = r_obs->lli | b_it->second->lli;
        auto r_l2 = rover_l2.find(sat); auto b_l2 = base_l2.find(sat);
        if (r_l2 != rover_l2.end() && b_l2 != base_l2.end()) {
            sd.rover_l2_phase = r_l2->second->carrier_phase; sd.rover_l2_code = r_l2->second->pseudorange;
            sd.base_l2_phase = b_l2->second->carrier_phase; sd.base_l2_code = b_l2->second->pseudorange;
            sd.has_l2 = true; sd.l2_lli = r_l2->second->lli | b_l2->second->lli;
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
    filter_state_.next_state_idx = BASE_STATES;

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
            if (!sd.has_l1 || !sd.has_l2) continue;
            double gf = (sd.rover_l1_phase - sd.base_l1_phase) * LAMBDA_L1 -
                        (sd.rover_l2_phase - sd.base_l2_phase) * LAMBDA_L2;
            auto prev_it = gf_l1l2_history_.find(sat);
            if (prev_it != gf_l1l2_history_.end() &&
                std::abs(gf - prev_it->second) > rtk_config_.cycle_slip_threshold) {
                gf_slips.insert(sat);
            }
            gf_l1l2_history_[sat] = gf;
        }
    }

    for (int freq = 0; freq < 2; ++freq) {
        double wl = (freq == 0) ? LAMBDA_L1 : LAMBDA_L2;
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

        // Compute SD biases and offset correction (RTKLIB udbias)
        std::map<SatelliteId, double> bias;
        double offset = 0.0;
        int offset_count = 0;

        for (const auto& [sat, sd] : sat_data) {
            bool has_freq = (freq == 0) ? sd.has_l1 : sd.has_l2;
            if (!has_freq) continue;
            double cp, pr;
            if (freq == 0) {
                cp = sd.rover_l1_phase - sd.base_l1_phase;
                pr = sd.rover_l1_code - sd.base_l1_code;
            } else {
                cp = sd.rover_l2_phase - sd.base_l2_phase;
                pr = sd.rover_l2_code - sd.base_l2_code;
            }
            double b = cp - pr / wl;
            bias[sat] = b;
            auto idx_it = indices.find(sat);
            if (idx_it != indices.end() && filter_state_.state(idx_it->second) != 0.0) {
                offset += b - filter_state_.state(idx_it->second);
                offset_count++;
            }
        }

        // Apply offset correction (preserves DD values)
        if (offset_count > 0) {
            double avg_offset = offset / offset_count;
            for (auto& [sat, idx] : indices) {
                if (filter_state_.state(idx) != 0.0) {
                    filter_state_.state(idx) += avg_offset;
                }
            }
        }

        // Initialize new satellites
        for (const auto& [sat, b] : bias) {
            auto idx_it = indices.find(sat);
            if (idx_it != indices.end() && filter_state_.state(idx_it->second) != 0.0) continue;
            if (freq == 0) getOrCreateN1Index(sat, b);
            else getOrCreateN2Index(sat, b);
            lock_counts[sat] = 0;
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
    if (has_last_solution_position_) {
        rover_pos = last_solution_position_;
    } else {
        auto spp = spp_processor_.processEpoch(rover_obs, nav);
        if (spp.isValid()) {
            rover_pos = spp.position_ecef;
        } else if (rover_obs.receiver_position.norm() > 1e6) {
            rover_pos = rover_obs.receiver_position;
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

    try {
        if (!base_position_known_) {
            auto spp = spp_processor_.processEpoch(rover_obs, nav);
            rememberSolution(spp);
            consecutive_fix_count_ = 0;
            return spp;
        }

        auto fallback_spp = [&]() {
            auto spp = spp_processor_.processEpoch(rover_obs, nav);
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

        SatelliteId new_ref = selectReferenceSatellite(sat_data);
        handleReferenceSatelliteChange(new_ref, sat_data);
        current_sat_data_ = sat_data;
        updateBias(sat_data);

        // Iterative KF update
        bool filter_ok = false;
        for (int iter = 0; iter < rtk_config_.kf_iterations; ++iter) {
            filter_ok = updateFilter(sat_data);
            if (!filter_ok) break;
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
            solution = generateSolution(rover_obs.time, SolutionStatus::FLOAT, n_sats);

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
            if (resolveAmbiguities() && has_fixed_solution_) {
                if (validateFixedSolution(sat_data)) {
                    Vector3d saved_baseline = filter_state_.state.head<3>();
                    filter_state_.state.head<3>() = fixed_baseline_;
                    solution = generateSolution(rover_obs.time, SolutionStatus::FIXED, n_sats);
                    filter_state_.state.head<3>() = saved_baseline;
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

    SatelliteId ref_sat = current_ref_satellite_;
    auto ref_it = sat_data.find(ref_sat);
    if (ref_it == sat_data.end()) return false;
    const auto& ref_sd = ref_it->second;

    auto ref_n1_it = filter_state_.n1_indices.find(ref_sat);
    if (ref_n1_it == filter_state_.n1_indices.end()) return false;

    int n_states = filter_state_.state.size();
    int max_obs = 4 * sat_data.size();
    MatrixXd H = MatrixXd::Zero(max_obs, n_states);
    VectorXd z(max_obs);
    std::vector<double> Ri_vec, Rj_vec;
    std::vector<int> nb_vec;

    Vector3d rover_pos = base_position_ + filter_state_.state.head<3>();

    double rr_ref = geodist_range(ref_sd.sat_pos, rover_pos) + tropModel(rover_pos, ref_sd.elevation);
    double br_ref = geodist_range(ref_sd.sat_pos_base, base_position_) + tropModel(base_position_, ref_sd.base_elevation);
    Vector3d los_ref = (ref_sd.sat_pos - rover_pos).normalized();

    int oi = 0;

    // L1 phase DDs
    int block_count = 0;
        for (const auto& [sat, sd] : sat_data) {
            if (sat == ref_sat || !sd.has_l1) continue;
            auto n1_it = filter_state_.n1_indices.find(sat);
            if (n1_it == filter_state_.n1_indices.end()) continue;
            double rr = geodist_range(sd.sat_pos, rover_pos) + tropModel(rover_pos, sd.elevation);
            double br = geodist_range(sd.sat_pos_base, base_position_) + tropModel(base_position_, sd.base_elevation);
            double geom_dd = (rr_ref - br_ref) - (rr - br);
            Vector3d dd_los = -los_ref + (sd.sat_pos - rover_pos).normalized();
            double phase_dd = (ref_sd.rover_l1_phase - ref_sd.base_l1_phase) - (sd.rover_l1_phase - sd.base_l1_phase);
            double amb_term = LAMBDA_L1 * filter_state_.state(ref_n1_it->second)
                            - LAMBDA_L1 * filter_state_.state(n1_it->second);
            z(oi) = phase_dd * LAMBDA_L1 - geom_dd - amb_term;
            H(oi, 0) = dd_los(0); H(oi, 1) = dd_los(1); H(oi, 2) = dd_los(2);
            H(oi, ref_n1_it->second) = LAMBDA_L1;
            H(oi, n1_it->second) = -LAMBDA_L1;
            Ri_vec.push_back(varerr(ref_sd.elevation, true));
            Rj_vec.push_back(varerr(sd.elevation, true));
            oi++; block_count++;
        }
        nb_vec.push_back(block_count);

        // L2 phase DDs
        auto ref_n2_it = filter_state_.n2_indices.find(ref_sat);
        block_count = 0;
        if (ref_sd.has_l2 && ref_n2_it != filter_state_.n2_indices.end()) {
            for (const auto& [sat, sd] : sat_data) {
                if (sat == ref_sat || !sd.has_l2) continue;
                auto n2_it = filter_state_.n2_indices.find(sat);
                if (n2_it == filter_state_.n2_indices.end()) continue;
                double rr = geodist_range(sd.sat_pos, rover_pos) + tropModel(rover_pos, sd.elevation);
                double br = geodist_range(sd.sat_pos_base, base_position_) + tropModel(base_position_, sd.base_elevation);
                double geom_dd = (rr_ref - br_ref) - (rr - br);
                Vector3d dd_los = -los_ref + (sd.sat_pos - rover_pos).normalized();
                double phase_dd = (ref_sd.rover_l2_phase - ref_sd.base_l2_phase) - (sd.rover_l2_phase - sd.base_l2_phase);
                z(oi) = phase_dd * LAMBDA_L2 - geom_dd
                        - LAMBDA_L2 * filter_state_.state(ref_n2_it->second)
                        + LAMBDA_L2 * filter_state_.state(n2_it->second);
                H(oi, 0) = dd_los(0); H(oi, 1) = dd_los(1); H(oi, 2) = dd_los(2);
                H(oi, ref_n2_it->second) = LAMBDA_L2;
                H(oi, n2_it->second) = -LAMBDA_L2;
                Ri_vec.push_back(varerr(ref_sd.elevation, true));
                Rj_vec.push_back(varerr(sd.elevation, true));
                oi++; block_count++;
            }
        }
        nb_vec.push_back(block_count);

        // L1 code DDs
        block_count = 0;
        for (const auto& [sat, sd] : sat_data) {
            if (sat == ref_sat || !sd.has_l1) continue;
            double rr = geodist_range(sd.sat_pos, rover_pos) + tropModel(rover_pos, sd.elevation);
            double br = geodist_range(sd.sat_pos_base, base_position_) + tropModel(base_position_, sd.base_elevation);
            double geom_dd = (rr_ref - br_ref) - (rr - br);
            Vector3d dd_los = -los_ref + (sd.sat_pos - rover_pos).normalized();
            double code_dd = (ref_sd.rover_l1_code - ref_sd.base_l1_code) - (sd.rover_l1_code - sd.base_l1_code);
            z(oi) = code_dd - geom_dd;
            H(oi, 0) = dd_los(0); H(oi, 1) = dd_los(1); H(oi, 2) = dd_los(2);
            Ri_vec.push_back(varerr(ref_sd.elevation, false));
            Rj_vec.push_back(varerr(sd.elevation, false));
            oi++; block_count++;
        }
        nb_vec.push_back(block_count);

        // L2 code DDs
        block_count = 0;
        if (ref_sd.has_l2) {
            for (const auto& [sat, sd] : sat_data) {
                if (sat == ref_sat || !sd.has_l2) continue;
                double rr = geodist_range(sd.sat_pos, rover_pos) + tropModel(rover_pos, sd.elevation);
                double br = geodist_range(sd.sat_pos_base, base_position_) + tropModel(base_position_, sd.base_elevation);
                double geom_dd = (rr_ref - br_ref) - (rr - br);
                Vector3d dd_los = -los_ref + (sd.sat_pos - rover_pos).normalized();
                double code_dd = (ref_sd.rover_l2_code - ref_sd.base_l2_code) - (sd.rover_l2_code - sd.base_l2_code);
                z(oi) = code_dd - geom_dd;
                H(oi, 0) = dd_los(0); H(oi, 1) = dd_los(1); H(oi, 2) = dd_los(2);
                Ri_vec.push_back(varerr(ref_sd.elevation, false));
                Rj_vec.push_back(varerr(sd.elevation, false));
                oi++; block_count++;
            }
        }
        nb_vec.push_back(block_count);

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
    if (!filter_initialized_ || !has_ref_satellite_) return false;

    SatelliteId ref_sat = current_ref_satellite_;
    const auto& sat_data = current_sat_data_;
    if (sat_data.find(ref_sat) == sat_data.end()) return false;

    int nx = filter_state_.state.size();
    int na = BASE_STATES;

    bool use_iflc = (rtk_config_.ionoopt == RTKConfig::IonoOpt::IFLC);

    // Build DD pairs (L1 + L2)
    std::vector<DDPair> dd_pairs;

    int nfreq_ar = 2;

    for (int freq = 0; freq < nfreq_ar; ++freq) {
        const auto& indices = (freq == 0) ? filter_state_.n1_indices : filter_state_.n2_indices;
        const auto& lock_counts = (freq == 0) ? lock_count_l1_ : lock_count_l2_;
        const int min_lock = std::max(1, rtk_config_.min_lock_count);

        int ref_idx = -1;
        SatelliteId lambda_ref = current_ref_satellite_;

        auto choose_reference = [&](const SatelliteId& sat) -> bool {
            auto idx_it = indices.find(sat);
            if (idx_it == indices.end()) return false;
            if (filter_state_.state(idx_it->second) == 0.0) return false;
            auto lock_it = lock_counts.find(sat);
            if (lock_it == lock_counts.end() || lock_it->second < min_lock) return false;
            if (sat_data.find(sat) == sat_data.end()) return false;
            ref_idx = idx_it->second;
            lambda_ref = sat;
            return true;
        };

        if (!choose_reference(current_ref_satellite_)) {
            double best_el = -1.0;
            for (const auto& [sat, state_idx] : indices) {
                if (filter_state_.state(state_idx) == 0.0) continue;
                auto lock_it = lock_counts.find(sat);
                if (lock_it == lock_counts.end() || lock_it->second < min_lock) continue;
                auto sat_it = sat_data.find(sat);
                if (sat_it == sat_data.end()) continue;
                if (sat_it->second.elevation <= best_el) continue;
                best_el = sat_it->second.elevation;
                ref_idx = state_idx;
                lambda_ref = sat;
            }
        }
        if (ref_idx < 0) continue;

        for (const auto& [sat, state_idx] : indices) {
            if (sat == lambda_ref) continue;
            if (filter_state_.state(state_idx) == 0.0) continue;
            auto lock_it = lock_counts.find(sat);
            if (lock_it == lock_counts.end() || lock_it->second < min_lock) continue;
            dd_pairs.push_back({ref_idx, state_idx, sat, freq});
        }
    }

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
    double ratio;
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
                    static constexpr double F1v = constants::GPS_L1_FREQ;
                    static constexpr double F2v = constants::GPS_L2_FREQ;
                    static constexpr double Cv = constants::SPEED_OF_LIGHT;
                    static constexpr double LAM_WLv = Cv / (F1v - F2v);
                    int mismatch = 0, checked = 0;
                    for (int i = 0; i < nb; ++i) {
                        if (dd_pairs[i].freq != 0) continue;
                        int l2p = -1;
                        for (int j = 0; j < nb; ++j)
                            if (dd_pairs[j].freq == 1 && dd_pairs[j].sat == dd_pairs[i].sat) { l2p = j; break; }
                        if (l2p < 0) continue;
                        // LAMBDA N_wl vs MW N_wl
                        double lambda_nwl = dd_fixed(i) - dd_fixed(l2p);

                        SatelliteId ref_id;
                        for (const auto& [s, idx] : filter_state_.n1_indices)
                            if (idx == dd_pairs[i].ref_idx) { ref_id = s; break; }
                        auto rit = sat_data.find(ref_id), sit = sat_data.find(dd_pairs[i].sat);
                        if (rit == sat_data.end() || sit == sat_data.end()) continue;
                        if (!rit->second.has_l2 || !sit->second.has_l2) continue;
                        auto mw_sd = [&](const SatelliteData& d) {
                            return (d.rover_l1_phase - d.base_l1_phase) - (d.rover_l2_phase - d.base_l2_phase)
                                 - (F1v*(d.rover_l1_code - d.base_l1_code) + F2v*(d.rover_l2_code - d.base_l2_code))
                                   / ((F1v+F2v) * LAM_WLv);
                        };
                        double dd_mw = mw_sd(rit->second) - mw_sd(sit->second);
                        double mw_nwl = std::round(dd_mw);
                        if (std::abs(dd_mw - mw_nwl) < 0.25) {
                            checked++;
                            // Check WL consistency
                            if (std::abs(lambda_nwl - mw_nwl) > 0.5) mismatch++;
                            // Also check NL: use IF to derive expected N2
                            static constexpr double C1v = F1v*F1v/(F1v*F1v-F2v*F2v);
                            static constexpr double C2v = -(F2v*F2v)/(F1v*F1v-F2v*F2v);
                            static constexpr double LNL = C1v*(Cv/F1v) + C2v*(Cv/F2v);
                            double if_val = C1v * LAMBDA_L1 * dd_float(i) + C2v * LAMBDA_L2 * dd_float(l2p);
                            double n2_if = (if_val - C1v * LAMBDA_L1 * mw_nwl) / LNL;
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
        static constexpr double F1 = constants::GPS_L1_FREQ;
        static constexpr double F2 = constants::GPS_L2_FREQ;
        static constexpr double CC = constants::SPEED_OF_LIGHT;
        static constexpr double LAM_WL = CC / (F1 - F2);
        static constexpr double C1_IF = F1*F1 / (F1*F1 - F2*F2);
        static constexpr double C2_IF = -(F2*F2) / (F1*F1 - F2*F2);
        static constexpr double LAM_NL = C1_IF * (CC/F1) + C2_IF * (CC/F2);

        VectorXd wlnl_fixed = dd_float;
        int wl_ok = 0, wl_total = 0;

        for (int i = 0; i < nb; ++i) {
            if (dd_pairs[i].freq != 0) continue;
            wl_total++;
            int l2p = -1;
            for (int j = 0; j < nb; ++j)
                if (dd_pairs[j].freq == 1 && dd_pairs[j].sat == dd_pairs[i].sat) { l2p = j; break; }
            if (l2p < 0) continue;

            // Find ref satellite
            SatelliteId ref_id;
            for (const auto& [s, idx] : filter_state_.n1_indices)
                if (idx == dd_pairs[i].ref_idx) { ref_id = s; break; }
            auto rit = sat_data.find(ref_id), sit = sat_data.find(dd_pairs[i].sat);
            if (rit == sat_data.end() || sit == sat_data.end()) continue;
            if (!rit->second.has_l2 || !sit->second.has_l2) continue;

            // MW wide-lane
            auto mw_sd = [&](const SatelliteData& d) {
                return (d.rover_l1_phase - d.base_l1_phase) - (d.rover_l2_phase - d.base_l2_phase)
                     - (F1*(d.rover_l1_code - d.base_l1_code) + F2*(d.rover_l2_code - d.base_l2_code))
                       / ((F1+F2) * LAM_WL);
            };
            double dd_mw = mw_sd(rit->second) - mw_sd(sit->second);
            double nw = std::round(dd_mw);
            if (std::abs(dd_mw - nw) > 0.25) continue;

            // IF → NL
            double if_dd = C1_IF * LAMBDA_L1 * dd_float(i) + C2_IF * LAMBDA_L2 * dd_float(l2p);
            double n2f = (if_dd - C1_IF * LAMBDA_L1 * nw) / LAM_NL;
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

    if (!fixed && nb > 4) {
        double best_ratio = ratio;
        VectorXd best_fixed = dd_fixed;
        std::vector<int> current_subset = best_subset;

        for (int drop = 0; drop < std::min(nb - 4, 6); ++drop) {
            int worst = -1;
            double worst_var = -1;
            for (int idx : current_subset) {
                if (Qb(idx, idx) > worst_var) { worst_var = Qb(idx, idx); worst = idx; }
            }
            if (worst < 0) break;

            std::vector<int> subset;
            for (int idx : current_subset) {
                if (idx != worst) subset.push_back(idx);
            }
            int ns = subset.size();
            if (ns < 4) break;

            VectorXd sub_float(ns);
            MatrixXd sub_Qb(ns, ns);
            MatrixXd sub_Qab(na, ns);
            for (int i = 0; i < ns; ++i) {
                sub_float(i) = dd_float(subset[i]);
                for (int j = 0; j < ns; ++j)
                    sub_Qb(i, j) = Qb(subset[i], subset[j]);
                for (int j = 0; j < na; ++j)
                    sub_Qab(j, i) = Qab(j, subset[i]);
            }
            sub_Qb = (sub_Qb + sub_Qb.transpose()) / 2.0;

            VectorXd sub_fixed;
            double sub_ratio;
            if (lambdaMethod(sub_float, sub_Qb, sub_fixed, sub_ratio)) {
                if (sub_ratio > best_ratio) {
                    best_ratio = sub_ratio;
                    best_fixed = sub_fixed;
                    best_subset = subset;
                    Qab = sub_Qab;
                    Qb = sub_Qb;
                    dd_float = sub_float;
                }
                if (sub_ratio >= effective_ratio_threshold) {
                    fixed = true;
                    dd_fixed = sub_fixed;
                    ratio = sub_ratio;
                    break;
                }
            }
            current_subset = subset;
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

        static constexpr double LAMBDA_WL_M = constants::SPEED_OF_LIGHT /
            (constants::GPS_L1_FREQ - constants::GPS_L2_FREQ);  // ~0.862m
        double f1 = constants::GPS_L1_FREQ;
        double f2 = constants::GPS_L2_FREQ;

        // Build map: satellite -> (fixed_DD_N1, fixed_DD_N2)
        // from last_dd_pairs_ and last_dd_fixed_
        std::map<SatelliteId, double> fixed_dd_n1, fixed_dd_n2;
        SatelliteId lambda_ref_l1, lambda_ref_l2;
        for (int i = 0; i < (int)last_best_subset_.size(); ++i) {
            int dd_idx = last_best_subset_[i];
            if (dd_idx >= (int)last_dd_pairs_.size()) continue;
            const auto& pair = last_dd_pairs_[dd_idx];
            if (pair.freq == 0) {
                fixed_dd_n1[pair.sat] = last_dd_fixed_(i);
                // Identify L1 ref from the ref_idx
                for (const auto& [sat, idx] : filter_state_.n1_indices) {
                    if (idx == pair.ref_idx) lambda_ref_l1 = sat;
                }
            } else if (pair.freq == 1) {
                fixed_dd_n2[pair.sat] = last_dd_fixed_(i);
                for (const auto& [sat, idx] : filter_state_.n2_indices) {
                    if (idx == pair.ref_idx) lambda_ref_l2 = sat;
                }
            }
        }

        // For each satellite with both L1 and L2 fixed DD:
        // Compute fixed wide-lane DD_NW = DD_N1 - DD_N2
        // Compare with Melbourne-Wubbena DD_MW from observations
        int bad_wl_count = 0;
        int checked_count = 0;
        for (const auto& [sat, dd_n1] : fixed_dd_n1) {
            auto n2_it = fixed_dd_n2.find(sat);
            if (n2_it == fixed_dd_n2.end()) continue;
            double dd_n2 = n2_it->second;
            double fixed_wl = dd_n1 - dd_n2;  // should be integer

            // Compute DD Melbourne-Wubbena from observations
            auto sat_it = sat_data.find(sat);
            auto ref_it = sat_data.find(lambda_ref_l1);
            if (sat_it == sat_data.end() || ref_it == sat_data.end()) continue;
            if (!sat_it->second.has_l2 || !ref_it->second.has_l2) continue;

            const auto& ref_sd = ref_it->second;
            const auto& sd = sat_it->second;

            double ref_L1_sd = ref_sd.rover_l1_phase - ref_sd.base_l1_phase;
            double ref_L2_sd = ref_sd.rover_l2_phase - ref_sd.base_l2_phase;
            double ref_P1_sd = ref_sd.rover_l1_code - ref_sd.base_l1_code;
            double ref_P2_sd = ref_sd.rover_l2_code - ref_sd.base_l2_code;
            double ref_MW = (ref_L1_sd - ref_L2_sd)
                          - (f1 * ref_P1_sd + f2 * ref_P2_sd) / (LAMBDA_WL_M * (f1 + f2));

            double sat_L1_sd = sd.rover_l1_phase - sd.base_l1_phase;
            double sat_L2_sd = sd.rover_l2_phase - sd.base_l2_phase;
            double sat_P1_sd = sd.rover_l1_code - sd.base_l1_code;
            double sat_P2_sd = sd.rover_l2_code - sd.base_l2_code;
            double sat_MW = (sat_L1_sd - sat_L2_sd)
                          - (f1 * sat_P1_sd + f2 * sat_P2_sd) / (LAMBDA_WL_M * (f1 + f2));

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
    int na = BASE_STATES;

    // Rebuild DD pairs using same logic as resolveAmbiguities
    SatelliteId ref_sat = current_ref_satellite_;
    if (sat_data.find(ref_sat) == sat_data.end()) return false;

    std::vector<DDPair> dd_pairs;
    for (int freq = 0; freq < 2; ++freq) {
        const auto& indices = (freq == 0) ? filter_state_.n1_indices : filter_state_.n2_indices;
        const auto& lock_counts = (freq == 0) ? lock_count_l1_ : lock_count_l2_;

        int ref_idx = -1;
        SatelliteId lambda_ref;
        for (const auto& [sat, state_idx] : indices) {
            if (filter_state_.state(state_idx) == 0.0) continue;
            auto lock_it = lock_counts.find(sat);
            if (lock_it == lock_counts.end() || lock_it->second <= 0) continue;
            if (sat_data.find(sat) == sat_data.end()) continue;
            lambda_ref = sat;
            ref_idx = state_idx;
            break;
        }
        if (ref_idx < 0) continue;

        for (const auto& [sat, state_idx] : indices) {
            if (sat == lambda_ref) continue;
            if (filter_state_.state(state_idx) == 0.0) continue;
            auto lock_it = lock_counts.find(sat);
            if (lock_it == lock_counts.end() || lock_it->second <= 0) continue;
            dd_pairs.push_back({ref_idx, state_idx, sat, freq});
        }
    }

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
            if (dd_pairs[i].ref_idx == last_dd_pairs_[dd_idx].ref_idx &&
                dd_pairs[i].sat_idx == last_dd_pairs_[dd_idx].sat_idx) {
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
    rememberSolution(solution);
    return solution;
}

void RTKProcessor::rememberSolution(const PositionSolution& solution) {
    if (!solution.isValid()) return;
    last_solution_position_ = solution.position_ecef;
    has_last_solution_position_ = true;
    last_epoch_time_ = solution.time;
    has_last_epoch_ = true;
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
    if (n == 0 || n > 40) return false;

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
    const ObservationData&, const ObservationData&, const NavigationData&) { return {}; }
void RTKProcessor::detectCycleSlips(const ObservationData&, const ObservationData&) {}
bool RTKProcessor::resolveAmbiguities(int) { return false; }
RTKProcessor::LAMBDAResult RTKProcessor::solveLAMBDA(const VectorXd&, const MatrixXd&) { return {}; }
bool RTKProcessor::validateAmbiguityResolution(const VectorXd&, const VectorXd&, const MatrixXd&, double) { return false; }
void RTKProcessor::updateAmbiguityStates(const std::vector<DoubleDifference>&) {}
Vector3d RTKProcessor::calculateBaseline() const { return Vector3d::Zero(); }
VectorXd RTKProcessor::calculateResiduals(const std::vector<DoubleDifference>&, const Vector3d&) const { return VectorXd::Zero(1); }
MatrixXd RTKProcessor::formMeasurementMatrix(const std::vector<DoubleDifference>&, const NavigationData&, const GNSSTime&) const { return MatrixXd::Zero(1,1); }
MatrixXd RTKProcessor::calculateMeasurementWeights(const std::vector<DoubleDifference>&) const { return MatrixXd::Identity(1,1); }
bool RTKProcessor::hasSufficientSatellites(const std::vector<DoubleDifference>&) const { return false; }
void RTKProcessor::resetAmbiguity(const SatelliteId&, SignalType) {}
bool RTKProcessor::applyFixedAmbiguities(const VectorXd&, const VectorXd&, const std::map<SatelliteId, SatelliteData>&) { return false; }
void RTKProcessor::solvePositionWithAmbiguities(const std::map<SatelliteId, SatelliteData>&) {}
bool RTKProcessor::trySingleEpochAR(const std::map<SatelliteId, SatelliteData>&) { return false; }

} // namespace libgnss
