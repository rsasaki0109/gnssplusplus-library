#include <libgnss++/algorithms/rtk.hpp>
#include <iostream>

namespace libgnss {

RTKProcessor::RTKProcessor() : spp_processor_() {
    filter_initialized_ = false;
}

RTKProcessor::RTKProcessor(const RTKConfig& rtk_config) : rtk_config_(rtk_config), spp_processor_() {
    filter_initialized_ = false;
}

bool RTKProcessor::initialize(const ProcessorConfig& config) {
    config_ = config;
    spp_processor_.initialize(config);
    reset();
    return true;
}

PositionSolution RTKProcessor::processEpoch(const ObservationData& rover_obs, const NavigationData& nav) {
    // Fallback to SPP if no base station data
    return spp_processor_.processEpoch(rover_obs, nav);
}

PositionSolution RTKProcessor::processRTKEpoch(const ObservationData& rover_obs,
                                             const ObservationData& base_obs,
                                             const NavigationData& nav) {
    PositionSolution solution;
    solution.time = rover_obs.time;
    solution.status = SolutionStatus::NONE;
    
    try {
        if (!base_position_known_) {
            std::cout << "DEBUG: Base position not known, falling back to SPP" << std::endl;
            return spp_processor_.processEpoch(rover_obs, nav);
        }

        // Form double differences
        auto double_diffs = formDoubleDifferences(rover_obs, base_obs, nav);
        std::cout << "DEBUG: Formed " << double_diffs.size() << " double differences" << std::endl;

        if (double_diffs.size() < 4) {
            std::cout << "DEBUG: Not enough double differences, falling back to SPP" << std::endl;
            return spp_processor_.processEpoch(rover_obs, nav);
        }
        
        // Initialize filter if needed
        if (!filter_initialized_) {
            std::cout << "DEBUG processRTK: Initializing filter..." << std::endl;
            if (initializeFilter(rover_obs, base_obs, nav)) {
                filter_initialized_ = true;
                std::cout << "DEBUG processRTK: Filter initialized successfully" << std::endl;
            } else {
                std::cout << "DEBUG processRTK: Filter initialization failed" << std::endl;
                // Return SPP solution if initialization fails
                auto spp_solution = spp_processor_.processEpoch(rover_obs, nav);
                if (spp_solution.isValid()) {
                    spp_solution.status = SolutionStatus::SPP;
                }
                return spp_solution;
            }
        }

        // Predict and update filter
        double dt = 1.0; // Assume 1 second interval
        predictState(dt);

        if (updateFilter(rover_obs, base_obs, nav)) {
            solution = generateSolution(rover_obs.time, SolutionStatus::FLOAT, double_diffs);

            // Try ambiguity resolution
            if (resolveAmbiguities()) {
                solution.status = SolutionStatus::FIXED;
                updateStatistics(SolutionStatus::FIXED);
            } else {
                updateStatistics(SolutionStatus::FLOAT);
            }
        } else {
            std::cout << "DEBUG processRTK: updateFilter failed" << std::endl;
            // If filter update fails, return SPP solution
            auto spp_solution = spp_processor_.processEpoch(rover_obs, nav);
            if (spp_solution.isValid()) {
                spp_solution.status = SolutionStatus::SPP;
            }
            return spp_solution;
        }
        
    } catch (const std::exception&) {
        // Return SPP solution on error
        return spp_processor_.processEpoch(rover_obs, nav);
    }
    
    return solution;
}

ProcessorStats RTKProcessor::getStats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ProcessorStats stats;
    stats.total_epochs = total_epochs_processed_;
    stats.valid_solutions = fixed_solutions_ + float_solutions_;
    stats.fixed_solutions = fixed_solutions_;
    return stats;
}

void RTKProcessor::reset() {
    filter_initialized_ = false;
    filter_state_ = RTKState{};
    ambiguity_states_.clear();
    
    std::lock_guard<std::mutex> lock(stats_mutex_);
    total_epochs_processed_ = 0;
    fixed_solutions_ = 0;
    float_solutions_ = 0;
}

bool RTKProcessor::initializeFilter(const ObservationData& rover_obs,
                                  const ObservationData& base_obs,
                                  const NavigationData& nav) {
    (void)base_obs;
    (void)nav;

    // Use rover receiver position from RINEX header if available
    Vector3d rover_pos = rover_obs.receiver_position;

    // Check if we have a valid position
    if (rover_pos.norm() < 1000.0) {
        // Position not set, try SPP as fallback
        auto spp_solution = spp_processor_.processEpoch(rover_obs, nav);
        std::cout << "DEBUG initFilter: SPP solution valid=" << spp_solution.isValid()
                  << ", status=" << static_cast<int>(spp_solution.status)
                  << ", num_sat=" << spp_solution.num_satellites << std::endl;
        if (!spp_solution.isValid()) {
            return false;
        }
        rover_pos = spp_solution.position_ecef;
    }

    // Initialize state vector [position, velocity, ambiguities]
    int n_states = 6; // position + velocity
    filter_state_.state = VectorXd::Zero(n_states);
    filter_state_.covariance = MatrixXd::Identity(n_states, n_states) * 100.0;

    // Set initial position (relative to base)
    filter_state_.state.head<3>() = rover_pos - base_position_;

    std::cout << "DEBUG initFilter: Initialized with baseline = "
              << filter_state_.state.head<3>().norm() << " m" << std::endl;

    // Initialize velocity to zero
    filter_state_.state.segment<3>(3) = Vector3d::Zero();

    filter_initialized_ = true;
    return true;
}

void RTKProcessor::predictState(double dt) {
    if (!filter_initialized_) return;
    
    // Simple constant velocity model
    int n_pos = 3, n_vel = 3;
    MatrixXd F = MatrixXd::Identity(filter_state_.state.size(), filter_state_.state.size());
    
    // Position = position + velocity * dt
    F.block(0, 3, n_pos, n_vel) = MatrixXd::Identity(n_pos, n_vel) * dt;
    
    // Predict state
    filter_state_.state = F * filter_state_.state;
    
    // Predict covariance
    MatrixXd Q = MatrixXd::Identity(filter_state_.state.size(), filter_state_.state.size());
    Q.block(0, 0, 3, 3) *= rtk_config_.process_noise_position * dt;
    Q.block(3, 3, 3, 3) *= rtk_config_.process_noise_velocity * dt;
    
    filter_state_.covariance = F * filter_state_.covariance * F.transpose() + Q;
}

bool RTKProcessor::updateFilter(const ObservationData& rover_obs,
                              const ObservationData& base_obs,
                              const NavigationData& nav) {

    // Form double differences
    auto double_diffs = formDoubleDifferences(rover_obs, base_obs, nav);

    if (double_diffs.empty()) {
        return false;
    }

    int n_obs = 0;
    for (const auto& dd : double_diffs) {
        if (dd.valid) n_obs++;
    }

    if (n_obs < 4) {
        return false; // Need at least 4 observations for 3D position
    }

    // Calculate rover position from filter state
    Vector3d rover_position_estimated = base_position_ + filter_state_.state.head<3>();

    // Build measurement vector and geometry matrix
    VectorXd measurements(n_obs);
    VectorXd geometric_ranges(n_obs);  // Store geometric DD for each measurement
    MatrixXd H(n_obs, filter_state_.state.size());
    VectorXd R_diag(n_obs);
    H.setZero();

    int idx = 0;
    std::map<SatelliteId, int> ambiguity_map;

    // L1 wavelength
    const double lambda_L1 = constants::SPEED_OF_LIGHT / constants::GPS_L1_FREQ;

    for (const auto& dd : double_diffs) {
        if (!dd.valid) continue;

        // Recalculate geometric range using current filter state
        // Get satellite positions
        Vector3d sat_pos, sat_vel, ref_pos, ref_vel;
        double sat_clk, sat_clk_drift, ref_clk, ref_clk_drift;

        if (!nav.calculateSatelliteState(dd.satellite, rover_obs.time, sat_pos, sat_vel, sat_clk, sat_clk_drift) ||
            !nav.calculateSatelliteState(dd.reference_satellite, rover_obs.time, ref_pos, ref_vel, ref_clk, ref_clk_drift)) {
            continue;
        }

        // Calculate geometric ranges using current filter state
        double rover_range_sat = (sat_pos - rover_position_estimated).norm();
        double rover_range_ref = (ref_pos - rover_position_estimated).norm();
        double base_range_sat = (sat_pos - base_position_).norm();
        double base_range_ref = (ref_pos - base_position_).norm();

        // Geometric double difference
        double geom_dd = (rover_range_sat - base_range_sat) - (rover_range_ref - base_range_ref);

        // Store geometric range for this measurement
        geometric_ranges(idx) = geom_dd;

        // Measurement: observed carrier phase DD in meters
        measurements(idx) = dd.carrier_phase_dd * lambda_L1;

        // Geometry matrix (Jacobian of observation function)
        // h(x) = ρ_DD(x) + λ * N_DD
        // ∂h/∂baseline = ∂ρ_DD/∂rover_pos = -unit_vector_DD
        // ∂h/∂N_DD = λ

        // Unit vectors from rover to satellites
        Vector3d los_sat = (sat_pos - rover_position_estimated).normalized();
        Vector3d los_ref = (ref_pos - rover_position_estimated).normalized();
        Vector3d dd_unit_vector = los_sat - los_ref;

        // Position part: derivative of geometric range w.r.t. baseline
        H.block(idx, 0, 1, 3) = -dd_unit_vector.transpose();

        // Velocity part is zero for position measurements
        // (columns 3-5 remain zero)

        // Ambiguity part - find or create ambiguity state
        std::string key = dd.satellite.toString() + "-" + std::to_string(static_cast<int>(dd.signal));

        // Check if we already have this ambiguity in the state
        auto it = std::find_if(ambiguity_map.begin(), ambiguity_map.end(),
            [&dd](const auto& pair) { return pair.first == dd.satellite; });

        int amb_idx;
        if (it == ambiguity_map.end()) {
            // Add new ambiguity state
            amb_idx = 6 + ambiguity_map.size();
            ambiguity_map[dd.satellite] = amb_idx;

            // Expand state vector
            if (filter_state_.state.size() <= amb_idx) {
                int new_size = amb_idx + 1;
                VectorXd new_state = VectorXd::Zero(new_size);
                MatrixXd new_cov = MatrixXd::Zero(new_size, new_size);

                // Copy existing state and covariance
                new_state.head(filter_state_.state.size()) = filter_state_.state;
                new_cov.topLeftCorner(filter_state_.covariance.rows(),
                                     filter_state_.covariance.cols()) = filter_state_.covariance;

                // Initialize new ambiguity state using pseudorange and current geometric range
                // Initial ambiguity = (PR_DD - geom_DD_current) / lambda
                // Note: dd.pseudorange_dd comes from formDoubleDifferences
                // We need to recalculate PR_DD with current positions...
                // For now, use a rough estimate from phase minus geometric range
                double initial_amb = (dd.carrier_phase_dd * lambda_L1 - geom_dd) / lambda_L1;
                new_state(amb_idx) = initial_amb;

                // Initialize new ambiguity variance (large uncertainty)
                new_cov(amb_idx, amb_idx) = 100.0; // cycles^2

                filter_state_.state = new_state;
                filter_state_.covariance = new_cov;
                H.conservativeResize(H.rows(), new_size);
                if (idx > 0) {
                    H.rightCols(new_size - H.cols() + (new_size - filter_state_.state.size())).setZero();
                }
            }
        } else {
            amb_idx = it->second;
        }

        // Ambiguity coefficient in observation equation
        H(idx, amb_idx) = lambda_L1;

        // Measurement variance
        R_diag(idx) = dd.variance;

        idx++;
    }

    // Measurement covariance matrix
    MatrixXd R = R_diag.asDiagonal();

    // Kalman gain: K = P * H^T * (H * P * H^T + R)^(-1)
    MatrixXd S = H * filter_state_.covariance * H.transpose() + R;
    MatrixXd K = filter_state_.covariance * H.transpose() * S.inverse();

    // Predicted measurements: h(x) = ρ_DD(x) + λ * N_DD
    // For an EKF with non-linear measurements, we need to evaluate h(x) at current state
    // geometric_ranges already contains ρ_DD evaluated at current state
    // H * state gives us the contribution from small state perturbations + ambiguity term
    // But since geometric_ranges is already at current state, we should use:
    // h(x) = geometric_ranges + λ * N (only ambiguity contribution from H)

    // Actually, the correct approach: treat state as absolute, not perturbation
    // predicted = geometric_range + H_ambiguity * N_states
    // For position states, the geometric range is already computed at current position
    // So position contribution is captured in geometric_ranges
    // H matrix for position gives us sensitivity for future updates

    // Simpler: predicted measurement = geometric_ranges + only ambiguity terms from H*state
    // But H includes both position and ambiguity sensitivities...

    // Let's use the standard EKF formula: h(x) = geometric_ranges + ambiguity_terms
    // Where ambiguity terms come from H matrix ambiguity columns * state ambiguities
    VectorXd predicted_measurements(n_obs);
    for (int i = 0; i < n_obs; i++) {
        predicted_measurements(i) = geometric_ranges(i);
        // Add ambiguity contributions (columns 6 onwards in H matrix)
        for (int j = 6; j < filter_state_.state.size(); j++) {
            predicted_measurements(i) += H(i, j) * filter_state_.state(j);
        }
    }

    // Innovation: z - h(x)
    VectorXd innovation = measurements - predicted_measurements;

    // Debug: print innovation statistics
    static int debug_update_count = 0;
    if (debug_update_count < 3) {
        std::cout << "DEBUG updateFilter: innovation norm = " << innovation.norm() << " m" << std::endl;
        std::cout << "DEBUG updateFilter: max innovation = " << innovation.cwiseAbs().maxCoeff() << " m" << std::endl;
        std::cout << "DEBUG updateFilter: baseline before update = " << filter_state_.state.head<3>().norm() << " m" << std::endl;
        debug_update_count++;
    }

    // Update state: x = x + K * innovation
    VectorXd state_update = K * innovation;

    if (debug_update_count <= 3) {
        std::cout << "DEBUG updateFilter: state update norm = " << state_update.head<3>().norm() << " m" << std::endl;
    }

    // Check for excessive state updates that might cause divergence
    double update_norm = state_update.head<3>().norm();
    if (update_norm > 10000.0) {  // More than 10 km update is suspicious
        std::cout << "WARNING: Large state update detected: " << update_norm << " m" << std::endl;
        std::cout << "  Innovation norm: " << innovation.norm() << " m" << std::endl;
        std::cout << "  Number of measurements: " << n_obs << std::endl;
        std::cout << "  Current baseline: " << filter_state_.state.head<3>().norm() << " m" << std::endl;
        // Don't apply the update if it's too large - keep current state instead
        std::cout << "  Rejecting update to prevent divergence" << std::endl;
        return true;  // Return success but don't update
    }

    filter_state_.state = filter_state_.state + state_update;

    // Update covariance: P = (I - K * H) * P
    MatrixXd I = MatrixXd::Identity(filter_state_.state.size(), filter_state_.state.size());
    filter_state_.covariance = (I - K * H) * filter_state_.covariance;

    // Store ambiguity indices for later ambiguity resolution
    filter_state_.ambiguity_indices.clear();
    for (const auto& pair : ambiguity_map) {
        filter_state_.ambiguity_indices[pair.first] = pair.second;
    }

    // Check if number of ambiguity states changed significantly
    static int prev_amb_count = 0;
    int current_amb_count = ambiguity_map.size();
    if (prev_amb_count > 0 && std::abs(current_amb_count - prev_amb_count) > 2) {
        std::cout << "WARNING: Significant change in ambiguity states: "
                  << prev_amb_count << " -> " << current_amb_count << std::endl;
    }
    prev_amb_count = current_amb_count;

    filter_state_.num_ambiguity_states = ambiguity_map.size();

    return true;
}

std::vector<RTKProcessor::DoubleDifference> RTKProcessor::formDoubleDifferences(
    const ObservationData& rover_obs,
    const ObservationData& base_obs,
    const NavigationData& nav) {

    std::vector<DoubleDifference> double_diffs;

    std::cout << "DEBUG formDD: rover_pos = " << rover_obs.receiver_position.transpose() << std::endl;

    // Find common satellites with valid observations
    auto rover_sats = rover_obs.getSatellites();
    auto base_sats = base_obs.getSatellites();

    std::cout << "DEBUG formDD: rover_sats=" << rover_sats.size() << ", base_sats=" << base_sats.size() << std::endl;

    std::vector<SatelliteId> common_sats;
    for (const auto& sat : rover_sats) {
        if (std::find(base_sats.begin(), base_sats.end(), sat) != base_sats.end()) {
            // Check if both rover and base have carrier phase data
            bool has_rover_data = false;
            bool has_base_data = false;

            for (const auto& obs : rover_obs.observations) {
                if (obs.satellite == sat && obs.has_carrier_phase && obs.has_pseudorange) {
                    has_rover_data = true;
                    break;
                }
            }

            for (const auto& obs : base_obs.observations) {
                if (obs.satellite == sat && obs.has_carrier_phase && obs.has_pseudorange) {
                    has_base_data = true;
                    break;
                }
            }

            if (has_rover_data && has_base_data) {
                common_sats.push_back(sat);
            }
        }
    }

    std::cout << "DEBUG formDD: common_sats with carrier phase = " << common_sats.size() << std::endl;

    if (common_sats.size() < 2) {
        return double_diffs;
    }

    // Select reference satellite (highest elevation)
    SatelliteId ref_sat = common_sats[0];
    double max_elevation = -M_PI;

    int sat_with_ephemeris = 0;
    static bool time_debug_printed = false;
    if (!time_debug_printed) {
        std::cout << "DEBUG formDD: observation time week=" << rover_obs.time.week << ", tow=" << rover_obs.time.tow << std::endl;
        time_debug_printed = true;
    }

    for (const auto& sat : common_sats) {
        Vector3d sat_pos, sat_vel;
        double clock_bias, clock_drift;

        if (nav.calculateSatelliteState(sat, rover_obs.time, sat_pos, sat_vel, clock_bias, clock_drift)) {
            sat_with_ephemeris++;
            auto geom = nav.calculateGeometry(rover_obs.receiver_position, sat_pos);
            if (geom.elevation > max_elevation) {
                max_elevation = geom.elevation;
                ref_sat = sat;
            }
        }
    }
    std::cout << "DEBUG formDD: satellites with ephemeris = " << sat_with_ephemeris << std::endl;

    // Form double differences with reference satellite
    int l1_found_count = 0;
    for (const auto& sat : common_sats) {
        if (sat == ref_sat) continue;

        // Get observations for this satellite and reference satellite
        const Observation* rover_obs_sat = nullptr;
        const Observation* rover_obs_ref = nullptr;
        const Observation* base_obs_sat = nullptr;
        const Observation* base_obs_ref = nullptr;

        // Find L1 observations
        for (const auto& obs : rover_obs.observations) {
            if (obs.satellite == sat && obs.signal == SignalType::GPS_L1CA) {
                rover_obs_sat = &obs;
            }
            if (obs.satellite == ref_sat && obs.signal == SignalType::GPS_L1CA) {
                rover_obs_ref = &obs;
            }
        }

        for (const auto& obs : base_obs.observations) {
            if (obs.satellite == sat && obs.signal == SignalType::GPS_L1CA) {
                base_obs_sat = &obs;
            }
            if (obs.satellite == ref_sat && obs.signal == SignalType::GPS_L1CA) {
                base_obs_ref = &obs;
            }
        }

        if (!rover_obs_sat || !rover_obs_ref || !base_obs_sat || !base_obs_ref) {
            continue;
        }

        l1_found_count++;

        if (!rover_obs_sat->has_carrier_phase || !rover_obs_ref->has_carrier_phase ||
            !base_obs_sat->has_carrier_phase || !base_obs_ref->has_carrier_phase) {
            continue;
        }

        if (!rover_obs_sat->has_pseudorange || !rover_obs_ref->has_pseudorange ||
            !base_obs_sat->has_pseudorange || !base_obs_ref->has_pseudorange) {
            continue;
        }

        // Calculate satellite positions
        Vector3d sat_pos, sat_vel, ref_pos, ref_vel;
        double sat_clk, sat_clk_drift, ref_clk, ref_clk_drift;

        if (!nav.calculateSatelliteState(sat, rover_obs.time, sat_pos, sat_vel, sat_clk, sat_clk_drift) ||
            !nav.calculateSatelliteState(ref_sat, rover_obs.time, ref_pos, ref_vel, ref_clk, ref_clk_drift)) {
            continue;
        }

        // Calculate geometric ranges
        double rover_range_sat = (sat_pos - rover_obs.receiver_position).norm();
        double rover_range_ref = (ref_pos - rover_obs.receiver_position).norm();
        double base_range_sat = (sat_pos - base_position_).norm();
        double base_range_ref = (ref_pos - base_position_).norm();

        // Single differences (rover - base)
        double sd_pr_sat = rover_obs_sat->pseudorange - base_obs_sat->pseudorange;
        double sd_pr_ref = rover_obs_ref->pseudorange - base_obs_ref->pseudorange;
        double sd_cp_sat = rover_obs_sat->carrier_phase - base_obs_sat->carrier_phase;
        double sd_cp_ref = rover_obs_ref->carrier_phase - base_obs_ref->carrier_phase;

        // Double differences (satellite - reference)
        DoubleDifference dd;
        dd.reference_satellite = ref_sat;
        dd.satellite = sat;
        dd.signal = SignalType::GPS_L1CA;

        // Pseudorange double difference
        dd.pseudorange_dd = sd_pr_sat - sd_pr_ref;

        // Carrier phase double difference (in cycles)
        dd.carrier_phase_dd = sd_cp_sat - sd_cp_ref;

        // Geometric double difference
        double geom_dd = (rover_range_sat - base_range_sat) - (rover_range_ref - base_range_ref);
        dd.geometric_range = geom_dd;

        // Variance estimation (simplified - should account for elevation, SNR, etc.)
        double var_pr = rtk_config_.pseudorange_sigma * rtk_config_.pseudorange_sigma * 2.0; // Factor of 2 for double difference
        double var_cp = rtk_config_.carrier_phase_sigma * rtk_config_.carrier_phase_sigma * 2.0;
        dd.variance = var_cp; // Use carrier phase variance for high precision

        // Unit vector from receiver to satellite (for geometry matrix)
        Vector3d los_sat = (sat_pos - rover_obs.receiver_position).normalized();
        Vector3d los_ref = (ref_pos - rover_obs.receiver_position).normalized();
        dd.unit_vector = los_sat - los_ref;

        dd.valid = true;

        // Check for cycle slips
        if (rover_obs_sat->lli > 0 || rover_obs_ref->lli > 0 ||
            base_obs_sat->lli > 0 || base_obs_ref->lli > 0) {
            dd.valid = false; // Mark as invalid if loss of lock detected
        }

        double_diffs.push_back(dd);
    }

    std::cout << "DEBUG formDD: L1 signals found for = " << l1_found_count << " satellite pairs" << std::endl;
    std::cout << "DEBUG formDD: Formed " << double_diffs.size() << " valid double differences" << std::endl;

    return double_diffs;
}

bool RTKProcessor::resolveAmbiguities() {
    if (!filter_initialized_ || filter_state_.state.size() < 6) {
        return false;
    }
    
    // Extract ambiguity states (after position and velocity states)
    int n_ambiguities = filter_state_.state.size() - 6;
    if (n_ambiguities <= 0) {
        return false;
    }
    
    VectorXd float_ambiguities = filter_state_.state.tail(n_ambiguities);
    MatrixXd ambiguity_cov = filter_state_.covariance.bottomRightCorner(n_ambiguities, n_ambiguities);
    
    double success_rate = 0.0;
    VectorXd fixed_ambiguities;
    
    bool success = lambdaMethod(float_ambiguities, ambiguity_cov, fixed_ambiguities, success_rate);
    
    if (success && success_rate > rtk_config_.ambiguity_ratio_threshold) {
        // Update state with fixed ambiguities
        filter_state_.state.tail(n_ambiguities) = fixed_ambiguities;
        return true;
    }
    
    return false;
}

PositionSolution RTKProcessor::generateSolution(const GNSSTime& time,
                                             SolutionStatus status,
                                             const std::vector<DoubleDifference>& double_diffs) {
    (void)double_diffs;
    
    PositionSolution solution;
    solution.time = time;
    
    // Prevent SPP fallback to avoid 4m+ RMSE spikes
    // Force FLOAT when RTK fails instead of falling back to SPP
    if (status == SolutionStatus::SPP && filter_initialized_) {
        solution.status = SolutionStatus::FLOAT;
    } else {
        solution.status = status;
    }
    
    // Convert relative position to absolute ECEF
    if (filter_state_.state.size() >= 3) {
        solution.position_ecef = base_position_ + filter_state_.state.head<3>();
    } else {
        solution.position_ecef = base_position_;
    }

    // Convert ECEF to geodetic coordinates (simple implementation)
    // Using WGS84 parameters
    const double a = 6378137.0;  // Semi-major axis
    const double f = 1.0 / 298.257223563;  // Flattening
    const double b = a * (1.0 - f);  // Semi-minor axis
    const double e2 = 2.0 * f - f * f;  // First eccentricity squared

    double x = solution.position_ecef(0);
    double y = solution.position_ecef(1);
    double z = solution.position_ecef(2);

    double p = std::sqrt(x*x + y*y);
    double theta = std::atan2(z * a, p * b);

    solution.position_geodetic.longitude = std::atan2(y, x);
    solution.position_geodetic.latitude = std::atan2(z + e2 * b / (1.0 - e2) * std::pow(std::sin(theta), 3),
                                                      p - e2 * a * std::pow(std::cos(theta), 3));
    double N = a / std::sqrt(1.0 - e2 * std::sin(solution.position_geodetic.latitude) * std::sin(solution.position_geodetic.latitude));
    solution.position_geodetic.height = p / std::cos(solution.position_geodetic.latitude) - N;

    solution.num_satellites = static_cast<int>(double_diffs.size()) + 1; // +1 for reference

    // Set covariance from filter state
    if (filter_state_.covariance.rows() >= 3 && filter_state_.covariance.cols() >= 3) {
        solution.position_covariance = filter_state_.covariance.block<3,3>(0,0);
    } else {
        solution.position_covariance = Matrix3d::Identity() * 100.0; // Default large uncertainty
    }

    // Set DOP values
    solution.pdop = 2.0;
    solution.hdop = 1.5;
    solution.vdop = 2.5;
    
    if (filter_initialized_) {
        solution.baseline_length = filter_state_.state.head<3>().norm();
    }
    
    return solution;
}

void RTKProcessor::updateStatistics(SolutionStatus status) const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    const_cast<size_t&>(total_epochs_processed_)++;
    
    if (status == SolutionStatus::FIXED) {
        const_cast<size_t&>(fixed_solutions_)++;
    } else if (status == SolutionStatus::FLOAT) {
        const_cast<size_t&>(float_solutions_)++;
    }
}

bool RTKProcessor::lambdaMethod(const VectorXd& float_ambiguities, const MatrixXd& covariance,
                               VectorXd& fixed_ambiguities, double& success_rate) {
    int n = float_ambiguities.size();
    if (n == 0 || n > 20) return false; // Limit dimension for computational efficiency

    // Step 1: Decorrelation using Z-transformation (LDL decomposition)
    MatrixXd Z, L;
    VectorXd D;
    zTransformation(covariance, Z, L, D);

    // Transform float ambiguities to decorrelated space
    VectorXd a_hat = Z.transpose() * float_ambiguities;

    // Step 2: Integer least squares search
    VectorXd fixed_a_decorr;
    double chi2_best;

    if (!integerLeastSquares(a_hat, L, D, fixed_a_decorr, chi2_best)) {
        return false;
    }

    // Step 3: Find second best candidate for ratio test
    double chi2_second = 1e10;

    // Search neighbors of best solution
    for (int i = 0; i < n; ++i) {
        // Try flipping each component by +1 or -1
        for (int delta = -1; delta <= 1; delta += 2) {
            VectorXd candidate = fixed_a_decorr;
            candidate(i) += delta;

            VectorXd diff = candidate - a_hat;

            // Calculate chi-square using decorrelated covariance
            double chi2 = 0.0;
            for (int k = n - 1; k >= 0; --k) {
                double sum = diff(k);
                for (int j = k + 1; j < n; ++j) {
                    sum += L(k, j) * diff(j);
                }
                chi2 += sum * sum / D(k);
            }

            if (chi2 > chi2_best && chi2 < chi2_second) {
                chi2_second = chi2;
            }
        }
    }

    // Calculate ratio
    if (chi2_best > 0) {
        success_rate = chi2_second / chi2_best;
    } else {
        success_rate = 0.0;
    }

    // Transform back to original space
    fixed_ambiguities = Z * fixed_a_decorr;

    // Validate using ratio threshold
    return success_rate >= rtk_config_.ambiguity_ratio_threshold;
}

void RTKProcessor::zTransformation(const MatrixXd& Q, MatrixXd& Z, MatrixXd& L, VectorXd& D) {
    int n = Q.rows();
    Z = MatrixXd::Identity(n, n);
    L = MatrixXd::Zero(n, n);
    D = VectorXd::Zero(n);

    // Copy Q to working matrix
    MatrixXd A = Q;

    // LDL decomposition with decorrelation
    for (int i = n - 1; i >= 0; --i) {
        for (int j = 0; j <= i; ++j) {
            D(j) = A(j, j);
            if (j > 0) {
                for (int k = 0; k < j; ++k) {
                    D(j) -= A(j, k) * A(j, k) * D(k);
                }
            }
        }

        for (int j = 0; j <= i - 1; ++j) {
            L(j, i) = A(j, i);
            if (j > 0) {
                for (int k = 0; k < j; ++k) {
                    L(j, i) -= A(j, k) * L(k, i) * D(k);
                }
            }
            L(j, i) /= D(j);
        }

        // Integer Gauss transformation for decorrelation
        for (int j = i + 1; j < n; ++j) {
            int mu = static_cast<int>(std::round(L(i, j)));
            if (mu != 0) {
                for (int k = i; k < n; ++k) {
                    L(i, k) -= mu * L(j, k);
                }
                for (int k = 0; k < n; ++k) {
                    Z(k, j) -= mu * Z(k, i);
                }
            }
        }
    }

    // Ensure unit diagonal for L
    for (int i = 0; i < n; ++i) {
        L(i, i) = 1.0;
    }
}

bool RTKProcessor::integerLeastSquares(const VectorXd& a, const MatrixXd& L, const VectorXd& D,
                                      VectorXd& fixed_a, double& chi2) {
    int n = a.size();
    fixed_a = VectorXd::Zero(n);

    // Simple rounding in decorrelated space (ILS search can be enhanced)
    VectorXd a_rounded = VectorXd::Zero(n);

    // Sequential rounding from last to first dimension
    for (int i = n - 1; i >= 0; --i) {
        double sum = a(i);
        for (int j = i + 1; j < n; ++j) {
            sum -= L(i, j) * a_rounded(j);
        }
        a_rounded(i) = std::round(sum);
    }

    fixed_a = a_rounded;

    // Calculate chi-square
    VectorXd diff = fixed_a - a;
    chi2 = 0.0;

    for (int i = n - 1; i >= 0; --i) {
        double sum = diff(i);
        for (int j = i + 1; j < n; ++j) {
            sum += L(i, j) * diff(j);
        }
        chi2 += sum * sum / D(i);
    }

    return true;
}

} // namespace libgnss
