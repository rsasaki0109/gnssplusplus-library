#include <libgnss++/algorithms/spp.hpp>
#include <stdexcept>

namespace libgnss {

SPPProcessor::SPPProcessor() {
    // Initialize with default configuration
    estimated_position_.setZero();
    receiver_clock_bias_ = 0.0;
}

SPPProcessor::SPPProcessor(const SPPConfig& spp_config) : spp_config_(spp_config) {
    estimated_position_.setZero();
    receiver_clock_bias_ = 0.0;
}

bool SPPProcessor::initialize(const ProcessorConfig& config) {
    config_ = config;
    reset();
    return true;
}

PositionSolution SPPProcessor::processEpoch(const ObservationData& obs, const NavigationData& nav) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    PositionSolution solution;
    solution.time = obs.time;
    solution.status = SolutionStatus::NONE;
    
    try {
        // Validate and filter observations
        auto valid_obs = validateObservations(obs, nav, obs.time);
        
        if (valid_obs.size() < 4) {
            updateStatistics(0.0, false);
            return solution;
        }
        
        // Solve position
        solution = solvePosition(valid_obs, nav, obs.time);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        solution.processing_time_ms = duration.count();
        
        updateStatistics(duration.count(), solution.isValid());
        
    } catch (const std::exception&) {
        updateStatistics(0.0, false);
    }
    
    return solution;
}

ProcessorStats SPPProcessor::getStats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ProcessorStats stats;
    stats.total_epochs = total_epochs_processed_;
    stats.valid_solutions = successful_solutions_;
    if (total_epochs_processed_ > 0) {
        stats.average_processing_time_ms = total_processing_time_ms_ / total_epochs_processed_;
    }
    return stats;
}

void SPPProcessor::reset() {
    estimated_position_.setZero();
    receiver_clock_bias_ = 0.0;
    system_biases_.clear();
    
    std::lock_guard<std::mutex> lock(stats_mutex_);
    total_epochs_processed_ = 0;
    successful_solutions_ = 0;
    total_processing_time_ms_ = 0.0;
}

PositionSolution SPPProcessor::solvePosition(const std::vector<Observation>& valid_obs,
                                           const NavigationData& nav,
                                           const GNSSTime& time) {
    PositionSolution solution;
    solution.time = time;
    solution.status = SolutionStatus::SPP;
    solution.num_satellites = valid_obs.size();
    
    // Initialize position if needed
    if (estimated_position_.norm() < 1000.0) {
        initializePosition(valid_obs, nav, time);
    }
    
    // Iterative least squares solution
    Vector3d position = estimated_position_;
    double clock_bias = receiver_clock_bias_;
    
    for (int iter = 0; iter < spp_config_.max_iterations; ++iter) {
        // Calculate satellite states
        auto sat_states = calculateSatelliteStates(valid_obs, nav, time);
        
        // Form measurement equations
        std::vector<Vector3d> sat_positions;
        VectorXd observations(valid_obs.size());
        VectorXd predicted(valid_obs.size());
        
        for (size_t i = 0; i < valid_obs.size(); ++i) {
            const auto& obs = valid_obs[i];
            const auto& sat_state = sat_states.at(obs.satellite);
            
            if (!sat_state.valid) continue;
            
            sat_positions.push_back(sat_state.position);
            observations(i) = obs.pseudorange;
            predicted(i) = calculatePredictedRange(position, sat_state.position, 
                                                 clock_bias, sat_state.clock_bias);
        }
        
        // Calculate geometry matrix
        MatrixXd H = calculateGeometryMatrix(position, sat_positions);
        
        // Calculate residuals
        VectorXd residuals = observations - predicted;
        
        // Check convergence
        if (residuals.norm() < spp_config_.position_convergence_threshold) {
            break;
        }
        
        // Least squares update
        MatrixXd HTH = H.transpose() * H;
        if (HTH.determinant() < 1e-12) {
            solution.status = SolutionStatus::NONE;
            return solution;
        }
        
        VectorXd dx = HTH.inverse() * H.transpose() * residuals;
        
        position += dx.head<3>();
        clock_bias += dx(3);
        
        solution.iterations = iter + 1;
    }
    
    // Update solution
    solution.position_ecef = position;
    solution.receiver_clock_bias = clock_bias;
    solution.residual_rms = 0.0; // Would calculate actual RMS
    
    // Calculate DOP values
    std::vector<Vector3d> sat_positions;
    for (size_t i = 0; i < valid_obs.size(); ++i) {
        auto sat_states = calculateSatelliteStates(valid_obs, nav, time);
        const auto& sat_state = sat_states.at(valid_obs[i].satellite);
        if (sat_state.valid) {
            sat_positions.push_back(sat_state.position);
        }
    }
    
    if (sat_positions.size() >= 4) {
        MatrixXd H = calculateGeometryMatrix(position, sat_positions);
        calculateDOP(H, solution);
    }
    
    // Store satellites used
    for (const auto& obs : valid_obs) {
        solution.satellites_used.push_back(obs.satellite);
    }
    
    estimated_position_ = position;
    receiver_clock_bias_ = clock_bias;
    
    return solution;
}

std::vector<Observation> SPPProcessor::validateObservations(const ObservationData& obs,
                                                          const NavigationData& nav,
                                                          const GNSSTime& time) const {
    std::vector<Observation> valid_obs;
    
    for (const auto& observation : obs.observations) {
        // Check if ephemeris is available
        if (!nav.hasEphemeris(observation.satellite, time)) {
            continue;
        }
        
        // Check observation validity
        if (!observation.valid || observation.pseudorange <= 0.0) {
            continue;
        }
        
        // Check SNR threshold
        if (observation.snr < config_.snr_mask) {
            continue;
        }
        
        valid_obs.push_back(observation);
    }
    
    return valid_obs;
}

bool SPPProcessor::initializePosition(const std::vector<Observation>& observations,
                                    const NavigationData& nav,
                                    const GNSSTime& time) {
    // Simple initialization - use first satellite position as rough estimate
    if (!observations.empty()) {
        auto sat_states = calculateSatelliteStates(observations, nav, time);
        if (!sat_states.empty()) {
            estimated_position_ = sat_states.begin()->second.position;
            estimated_position_ *= 0.9; // Move closer to Earth center
            return true;
        }
    }
    return false;
}

std::map<SatelliteId, SPPProcessor::SatelliteState> SPPProcessor::calculateSatelliteStates(
    const std::vector<Observation>& observations,
    const NavigationData& nav,
    const GNSSTime& time) const {
    
    std::map<SatelliteId, SatelliteState> states;
    
    for (const auto& obs : observations) {
        SatelliteState state;
        state.valid = nav.calculateSatelliteState(obs.satellite, time,
                                                state.position, state.velocity,
                                                state.clock_bias, state.clock_drift);
        states[obs.satellite] = state;
    }
    
    return states;
}

double SPPProcessor::calculatePredictedRange(const Vector3d& receiver_pos,
                                           const Vector3d& satellite_pos,
                                           double receiver_clock_bias,
                                           double satellite_clock_bias) const {
    double geometric_range = (satellite_pos - receiver_pos).norm();
    double clock_correction = (receiver_clock_bias - satellite_clock_bias) * constants::SPEED_OF_LIGHT;
    return geometric_range + clock_correction;
}

MatrixXd SPPProcessor::calculateGeometryMatrix(const Vector3d& receiver_pos,
                                             const std::vector<Vector3d>& satellite_positions) const {
    int n_sats = satellite_positions.size();
    MatrixXd H(n_sats, 4);
    
    for (int i = 0; i < n_sats; ++i) {
        Vector3d los = (satellite_positions[i] - receiver_pos).normalized();
        H(i, 0) = -los(0);
        H(i, 1) = -los(1);
        H(i, 2) = -los(2);
        H(i, 3) = 1.0;
    }
    
    return H;
}

void SPPProcessor::calculateDOP(const MatrixXd& geometry_matrix, PositionSolution& solution) const {
    MatrixXd Q = (geometry_matrix.transpose() * geometry_matrix).inverse();
    
    solution.gdop = std::sqrt(Q.trace());
    solution.pdop = std::sqrt(Q(0,0) + Q(1,1) + Q(2,2));
    solution.hdop = std::sqrt(Q(0,0) + Q(1,1));
    solution.vdop = std::sqrt(Q(2,2));
}

void SPPProcessor::updateStatistics(double processing_time_ms, bool success) const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    const_cast<size_t&>(total_epochs_processed_)++;
    const_cast<double&>(total_processing_time_ms_) += processing_time_ms;
    if (success) {
        const_cast<size_t&>(successful_solutions_)++;
    }
}

// Utility functions
namespace spp_utils {

double calculateElevation(const Vector3d& receiver_pos, const Vector3d& satellite_pos) {
    Vector3d los = satellite_pos - receiver_pos;
    Vector3d up = receiver_pos.normalized();
    return std::asin(los.dot(up) / los.norm());
}

double calculateAzimuth(const Vector3d& receiver_pos, const Vector3d& satellite_pos) {
    // Simplified azimuth calculation
    Vector3d los = satellite_pos - receiver_pos;
    return std::atan2(los(1), los(0));
}

GeodeticCoord ecefToGeodetic(const Vector3d& ecef_pos) {
    // WGS84 ellipsoid parameters
    const double a = constants::WGS84_A;
    const double e2 = constants::WGS84_E2;
    
    double x = ecef_pos(0);
    double y = ecef_pos(1);
    double z = ecef_pos(2);
    
    double lon = std::atan2(y, x);
    double p = std::sqrt(x*x + y*y);
    double lat = std::atan2(z, p * (1.0 - e2));
    
    // Iterative solution for latitude
    for (int i = 0; i < 5; ++i) {
        double N = a / std::sqrt(1.0 - e2 * std::sin(lat) * std::sin(lat));
        double h = p / std::cos(lat) - N;
        lat = std::atan2(z, p * (1.0 - e2 * N / (N + h)));
    }
    
    double N = a / std::sqrt(1.0 - e2 * std::sin(lat) * std::sin(lat));
    double h = p / std::cos(lat) - N;
    
    return GeodeticCoord(lat, lon, h);
}

Vector3d geodeticToEcef(const GeodeticCoord& geodetic_pos) {
    const double a = constants::WGS84_A;
    const double e2 = constants::WGS84_E2;
    
    double lat = geodetic_pos.latitude;
    double lon = geodetic_pos.longitude;
    double h = geodetic_pos.height;
    
    double N = a / std::sqrt(1.0 - e2 * std::sin(lat) * std::sin(lat));
    
    double x = (N + h) * std::cos(lat) * std::cos(lon);
    double y = (N + h) * std::cos(lat) * std::sin(lon);
    double z = (N * (1.0 - e2) + h) * std::sin(lat);
    
    return Vector3d(x, y, z);
}

} // namespace spp_utils

} // namespace libgnss
