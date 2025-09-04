#include <libgnss++/algorithms/rtk.hpp>

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
            // Fallback to SPP
            return spp_processor_.processEpoch(rover_obs, nav);
        }
        
        // Form double differences
        auto double_diffs = formDoubleDifferences(rover_obs, base_obs, nav);
        
        if (double_diffs.size() < 4) {
            return spp_processor_.processEpoch(rover_obs, nav);
        }
        
        // Initialize filter if needed
        if (!filter_initialized_) {
            if (initializeFilter(rover_obs, base_obs, nav)) {
                filter_initialized_ = true;
            } else {
                return spp_processor_.processEpoch(rover_obs, nav);
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
    // Simple initialization - use SPP solution as initial position
    auto spp_solution = spp_processor_.processEpoch(rover_obs, nav);
    if (!spp_solution.isValid()) {
        return false;
    }
    
    // Initialize state vector [position, velocity, ambiguities]
    int n_states = 6; // position + velocity
    filter_state_.state = VectorXd::Zero(n_states);
    filter_state_.covariance = MatrixXd::Identity(n_states, n_states) * 100.0;
    
    // Set initial position
    filter_state_.state.head<3>() = spp_solution.position_ecef - base_position_;
    
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
    (void)rover_obs;
    (void)base_obs;
    (void)nav;
    // Simplified update - would implement full Kalman filter update here
    return true;
}

std::vector<RTKProcessor::DoubleDifference> RTKProcessor::formDoubleDifferences(
    const ObservationData& rover_obs,
    const ObservationData& base_obs,
    const NavigationData& nav) {
    (void)nav;
    
    std::vector<DoubleDifference> double_diffs;
    
    // Find common satellites
    auto rover_sats = rover_obs.getSatellites();
    auto base_sats = base_obs.getSatellites();
    
    std::vector<SatelliteId> common_sats;
    for (const auto& sat : rover_sats) {
        if (std::find(base_sats.begin(), base_sats.end(), sat) != base_sats.end()) {
            common_sats.push_back(sat);
        }
    }
    
    if (common_sats.size() < 2) {
        return double_diffs;
    }
    
    // Use first satellite as reference
    SatelliteId ref_sat = common_sats[0];
    
    for (size_t i = 1; i < common_sats.size(); ++i) {
        DoubleDifference dd;
        dd.reference_satellite = ref_sat;
        dd.satellite = common_sats[i];
        dd.signal = SignalType::GPS_L1CA; // Simplified
        dd.valid = true;
        
        // Would calculate actual double differences here
        dd.pseudorange_dd = 0.0;
        dd.carrier_phase_dd = 0.0;
        dd.variance = rtk_config_.carrier_phase_sigma * rtk_config_.carrier_phase_sigma;
        
        double_diffs.push_back(dd);
    }
    
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
                                              const std::vector<DoubleDifference>& measurements) {
    PositionSolution solution;
    solution.time = time;
    solution.status = status;
    solution.num_satellites = measurements.size() + 1; // +1 for reference satellite
    
    if (filter_initialized_) {
        solution.position_ecef = base_position_ + filter_state_.state.head<3>();
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

bool RTKProcessor::lambdaMethod(const VectorXd& float_ambiguities,
                               const MatrixXd& covariance_matrix,
                               VectorXd& fixed_ambiguities,
                               double& success_rate) {
    int n = float_ambiguities.size();
    if (n == 0 || covariance_matrix.rows() != n || covariance_matrix.cols() != n) {
        return false;
    }
    
    // Decorrelation using Z-transformation
    MatrixXd Z, L;
    VectorXd D;
    zTransformation(covariance_matrix, Z, L, D);
    
    // Decorrelated ambiguities
    VectorXd decorrelated_ambiguities = Z.transpose() * float_ambiguities;
    
    // Integer least squares search
    VectorXd fixed_decorrelated;
    double chi2_best, chi2_second;
    
    if (!integerLeastSquares(decorrelated_ambiguities, L, D, fixed_decorrelated, chi2_best)) {
        return false;
    }
    
    // Search for second best candidate for ratio test
    VectorXd temp_fixed;
    double temp_chi2;
    (void)temp_fixed;
    (void)temp_chi2;
    chi2_second = chi2_best + 1000.0; // Default value
    
    // Simplified search for second best candidate
    for (int i = 0; i < n; ++i) {
        VectorXd candidate = fixed_decorrelated;
        candidate(i) += (candidate(i) > decorrelated_ambiguities(i)) ? -1.0 : 1.0;
        
        VectorXd diff = candidate - decorrelated_ambiguities;
        double chi2 = diff.transpose() * L.inverse().transpose() * D.asDiagonal().inverse() * L.inverse() * diff;
        
        if (chi2 < chi2_second && chi2 > chi2_best) {
            chi2_second = chi2;
        }
    }
    
    // Ratio test validation
    if (!validateAmbiguityResolution(chi2_best, chi2_second)) {
        return false;
    }
    
    // Transform back to original space
    fixed_ambiguities = Z * fixed_decorrelated;
    success_rate = chi2_second / chi2_best;
    
    return true;
}

void RTKProcessor::zTransformation(const MatrixXd& Q, MatrixXd& Z, MatrixXd& L, VectorXd& D) {
    int n = Q.rows();
    Z = MatrixXd::Identity(n, n);
    L = MatrixXd::Zero(n, n);
    D = VectorXd::Zero(n);
    
    MatrixXd Q_work = Q;
    
    // LDL decomposition
    for (int i = 0; i < n; ++i) {
        D(i) = Q_work(i, i);
        L(i, i) = 1.0;
        
        for (int j = i + 1; j < n; ++j) {
            L(j, i) = Q_work(j, i) / D(i);
        }
        
        for (int j = i + 1; j < n; ++j) {
            for (int k = j; k < n; ++k) {
                Q_work(k, j) -= L(k, i) * L(j, i) * D(i);
            }
        }
    }
    
    // Decorrelation using Z-transformation (simplified version)
    for (int i = n - 1; i >= 1; --i) {
        for (int j = i - 1; j >= 0; --j) {
            double mu = std::round(L(i, j));
            if (std::abs(mu) > 0.5) {
                // Apply Z-transformation
                for (int k = 0; k < n; ++k) {
                    Z(k, j) -= mu * Z(k, i);
                }
                
                // Update L matrix
                L(i, j) -= mu;
                for (int k = 0; k <= j; ++k) {
                    L(j, k) -= mu * L(i, k);
                }
            }
        }
    }
}

bool RTKProcessor::integerLeastSquares(const VectorXd& a, const MatrixXd& L, const VectorXd& D,
                                     VectorXd& fixed_a, double& chi2) {
    int n = a.size();
    fixed_a = VectorXd::Zero(n);
    
    // Simplified integer search (nearest neighbor rounding)
    VectorXd rounded = a;
    for (int i = 0; i < n; ++i) {
        rounded(i) = std::round(a(i));
    }
    
    // Adjustment using backward substitution
    for (int i = n - 1; i >= 0; --i) {
        double sum = 0.0;
        for (int j = i + 1; j < n; ++j) {
            sum += L(j, i) * fixed_a(j);
        }
        fixed_a(i) = std::round(a(i) - sum);
    }
    
    // Calculate chi-square statistic
    VectorXd diff = fixed_a - a;
    chi2 = 0.0;
    
    for (int i = 0; i < n; ++i) {
        double temp = diff(i);
        for (int j = 0; j < i; ++j) {
            temp -= L(i, j) * diff(j);
        }
        chi2 += temp * temp / D(i);
    }
    
    return true;
}

bool RTKProcessor::validateAmbiguityResolution(double chi2_1, double chi2_2) {
    // Ratio test
    double ratio = chi2_2 / chi2_1;
    return ratio > rtk_config_.ambiguity_ratio_threshold;
}

} // namespace libgnss
