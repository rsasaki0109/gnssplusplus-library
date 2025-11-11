#include <libgnss++/algorithms/ppp.hpp>

namespace libgnss {

PPPProcessor::PPPProcessor() : spp_processor_() {
    filter_initialized_ = false;
    converged_ = false;
    convergence_time_ = 0.0;
}

PPPProcessor::PPPProcessor(const PPPConfig& ppp_config) : ppp_config_(ppp_config), spp_processor_() {
    filter_initialized_ = false;
    converged_ = false;
    convergence_time_ = 0.0;
}

bool PPPProcessor::initialize(const ProcessorConfig& config) {
    config_ = config;
    spp_processor_.initialize(config);
    reset();
    return true;
}

PositionSolution PPPProcessor::processEpoch(const ObservationData& obs, const NavigationData& nav) {
    PositionSolution solution;
    solution.time = obs.time;
    solution.status = SolutionStatus::NONE;
    
    try {
        // Initialize filter if needed
        if (!filter_initialized_) {
            if (initializeFilter(obs, nav)) {
                filter_initialized_ = true;
                convergence_start_time_ = obs.time;
            } else {
                return spp_processor_.processEpoch(obs, nav);
            }
        }
        
        // Predict and update filter
        double dt = 30.0; // Assume 30 second interval
        predictState(dt);
        
        if (updateFilter(obs, nav)) {
            auto if_obs = formIonosphereFree(obs);
            solution = generateSolution(obs.time, if_obs);
            
            // Check convergence
            checkConvergence(obs.time);
            
            if (converged_) {
                solution.status = SolutionStatus::PPP_FLOAT;
                if (ppp_config_.enable_ambiguity_resolution && resolveAmbiguities()) {
                    solution.status = SolutionStatus::PPP_FIXED;
                }
            }
            
            updateStatistics(converged_);
        }
        
    } catch (const std::exception&) {
        // Return SPP solution on error
        return spp_processor_.processEpoch(obs, nav);
    }
    
    return solution;
}

ProcessorStats PPPProcessor::getStats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ProcessorStats stats;
    stats.total_epochs = total_epochs_processed_;
    stats.valid_solutions = converged_solutions_;
    if (total_epochs_processed_ > 0) {
        stats.average_processing_time_ms = total_convergence_time_ / total_epochs_processed_;
    }
    return stats;
}

void PPPProcessor::reset() {
    filter_initialized_ = false;
    converged_ = false;
    convergence_time_ = 0.0;
    filter_state_ = PPPState{};
    ambiguity_states_.clear();
    recent_positions_.clear();
    
    std::lock_guard<std::mutex> lock(stats_mutex_);
    total_epochs_processed_ = 0;
    converged_solutions_ = 0;
    total_convergence_time_ = 0.0;
}

bool PPPProcessor::loadPreciseProducts(const std::string& orbit_file, const std::string& clock_file) {
    return precise_products_.loadSP3File(orbit_file) && 
           precise_products_.loadClockFile(clock_file);
}

bool PPPProcessor::initializeFilter(const ObservationData& obs, const NavigationData& nav) {
    // Use SPP solution as initial position
    auto spp_solution = spp_processor_.processEpoch(obs, nav);
    if (!spp_solution.isValid()) {
        return false;
    }
    
    // Initialize state vector [position, velocity, clock, troposphere, ambiguities]
    int n_states = 8; // Basic states without ambiguities
    filter_state_.state = VectorXd::Zero(n_states);
    filter_state_.covariance = MatrixXd::Identity(n_states, n_states);
    
    // Set initial position
    filter_state_.state.segment(filter_state_.pos_index, 3) = spp_solution.position_ecef;
    
    // Set initial covariances
    filter_state_.covariance.block(0, 0, 3, 3) *= 100.0; // Position
    filter_state_.covariance.block(3, 3, 3, 3) *= 10.0;  // Velocity
    filter_state_.covariance(6, 6) = 1e6;  // Clock
    filter_state_.covariance(7, 7) = 0.01; // Troposphere
    
    filter_state_.total_states = n_states;
    
    return true;
}

void PPPProcessor::predictState(double dt) {
    if (!filter_initialized_) return;
    
    // State transition matrix
    MatrixXd F = MatrixXd::Identity(filter_state_.total_states, filter_state_.total_states);
    
    // Position = position + velocity * dt
    F.block(0, 3, 3, 3) = MatrixXd::Identity(3, 3) * dt;
    
    // Predict state
    filter_state_.state = F * filter_state_.state;
    
    // Process noise
    MatrixXd Q = MatrixXd::Zero(filter_state_.total_states, filter_state_.total_states);
    Q.block(0, 0, 3, 3) = MatrixXd::Identity(3, 3) * ppp_config_.process_noise_position * dt;
    Q.block(3, 3, 3, 3) = MatrixXd::Identity(3, 3) * ppp_config_.process_noise_velocity * dt;
    Q(6, 6) = ppp_config_.process_noise_clock * dt;
    Q(7, 7) = ppp_config_.process_noise_troposphere * dt;
    
    // Predict covariance
    filter_state_.covariance = F * filter_state_.covariance * F.transpose() + Q;
}

bool PPPProcessor::updateFilter(const ObservationData& obs, const NavigationData& nav) {
    // Form ionosphere-free combinations
    auto if_obs = formIonosphereFree(obs);
    
    if (if_obs.size() < 4) {
        return false;
    }
    
    // Apply precise corrections
    applyPreciseCorrections(if_obs, nav, obs.time);
    
    // Form measurement equations
    auto meas_eq = formMeasurementEquations(if_obs, nav, obs.time);
    
    // Kalman filter update
    MatrixXd K = filter_state_.covariance * meas_eq.design_matrix.transpose() * 
                 (meas_eq.design_matrix * filter_state_.covariance * meas_eq.design_matrix.transpose() + 
                  meas_eq.weight_matrix.inverse()).inverse();
    
    filter_state_.state += K * meas_eq.residuals;
    filter_state_.covariance = (MatrixXd::Identity(filter_state_.total_states, filter_state_.total_states) - 
                               K * meas_eq.design_matrix) * filter_state_.covariance;
    
    return true;
}

std::vector<PPPProcessor::IonosphereFreeObs> PPPProcessor::formIonosphereFree(const ObservationData& obs) {
    std::vector<IonosphereFreeObs> if_obs;
    
    // Simplified - would form actual ionosphere-free combinations
    for (const auto& observation : obs.observations) {
        if (observation.valid && observation.satellite.system == GNSSSystem::GPS) {
            IonosphereFreeObs if_ob;
            if_ob.satellite = observation.satellite;
            if_ob.pseudorange_if = observation.pseudorange;
            if_ob.carrier_phase_if = observation.carrier_phase;
            if_ob.variance_pr = ppp_config_.pseudorange_sigma * ppp_config_.pseudorange_sigma;
            if_ob.variance_cp = ppp_config_.carrier_phase_sigma * ppp_config_.carrier_phase_sigma;
            if_ob.valid = true;
            
            if_obs.push_back(if_ob);
        }
    }
    
    return if_obs;
}

void PPPProcessor::applyPreciseCorrections(std::vector<IonosphereFreeObs>& observations,
                                         const NavigationData& nav,
                                         const GNSSTime& time) {
    // Would apply precise orbit and clock corrections here
    (void)observations;
    (void)nav;
    (void)time;
}

void PPPProcessor::detectCycleSlips(const ObservationData& obs) {
    // Would implement cycle slip detection here
    (void)obs;
}

bool PPPProcessor::resolveAmbiguities() {
    // Would implement PPP ambiguity resolution here
    return false;
}

PPPProcessor::MeasurementEquation PPPProcessor::formMeasurementEquations(
    const std::vector<IonosphereFreeObs>& observations,
    const NavigationData& nav,
    const GNSSTime& time) {
    
    MeasurementEquation meas_eq;
    int n_obs = observations.size() * 2; // Pseudorange + carrier phase
    
    meas_eq.design_matrix = MatrixXd::Zero(n_obs, filter_state_.total_states);
    meas_eq.observations = VectorXd::Zero(n_obs);
    meas_eq.predicted = VectorXd::Zero(n_obs);
    meas_eq.weight_matrix = MatrixXd::Identity(n_obs, n_obs);
    
    // Would form actual measurement equations here
    (void)nav;
    (void)time;
    
    meas_eq.residuals = meas_eq.observations - meas_eq.predicted;
    
    return meas_eq;
}

void PPPProcessor::checkConvergence(const GNSSTime& current_time) {
    if (converged_) return;
    
    Vector3d current_pos = filter_state_.state.segment(filter_state_.pos_index, 3);
    recent_positions_.push_back(current_pos);
    
    // Keep only recent positions
    if (recent_positions_.size() > ppp_config_.convergence_min_epochs) {
        recent_positions_.erase(recent_positions_.begin());
    }
    
    if (recent_positions_.size() >= ppp_config_.convergence_min_epochs) {
        // Check position stability
        Vector3d mean_pos = Vector3d::Zero();
        for (const auto& pos : recent_positions_) {
            mean_pos += pos;
        }
        mean_pos /= recent_positions_.size();
        
        double max_deviation = 0.0;
        for (const auto& pos : recent_positions_) {
            double deviation = (pos - mean_pos).norm();
            max_deviation = std::max(max_deviation, deviation);
        }
        
        if (max_deviation < ppp_config_.convergence_threshold_horizontal) {
            converged_ = true;
            convergence_time_ = current_time - convergence_start_time_;
        }
    }
}

PositionSolution PPPProcessor::generateSolution(const GNSSTime& time,
                                              const std::vector<IonosphereFreeObs>& observations) {
    PositionSolution solution;
    solution.time = time;
    solution.status = converged_ ? SolutionStatus::PPP_FLOAT : SolutionStatus::SPP;
    solution.num_satellites = observations.size();
    
    if (filter_initialized_) {
        solution.position_ecef = filter_state_.state.segment(filter_state_.pos_index, 3);
        solution.receiver_clock_bias = filter_state_.state(filter_state_.clock_index);
        
        // Extract position covariance
        solution.position_covariance = filter_state_.covariance.block(0, 0, 3, 3);
    }
    
    return solution;
}

void PPPProcessor::updateStatistics(bool converged) const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    const_cast<size_t&>(total_epochs_processed_)++;
    if (converged) {
        const_cast<size_t&>(converged_solutions_)++;
    }
}

} // namespace libgnss
