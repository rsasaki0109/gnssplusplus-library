#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/models/troposphere.hpp>
#include <libgnss++/models/ionosphere.hpp>
#include <stdexcept>
#include <chrono>
#include <cmath>

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
        // Use RINEX header position for initialization if available
        if (estimated_position_.norm() < 1000.0 && obs.receiver_position.norm() > 1e6) {
            estimated_position_ = obs.receiver_position;
        }

        // Validate and filter observations
        auto valid_obs = validateObservations(obs, nav, obs.time);

        if (valid_obs.size() < 4) {
            updateStatistics(0.0, false);
            return solution;
        }

        // Solve position using native least-squares solver
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
    // Direct call to native least-squares solver
    return solvePositionLS(valid_obs, nav, time);
}

PositionSolution SPPProcessor::solvePositionLS(const std::vector<Observation>& valid_obs,
                                               const NavigationData& nav,
                                               const GNSSTime& time) {
    PositionSolution solution;
    solution.time = time;
    solution.status = SolutionStatus::SPP;
    solution.num_satellites = valid_obs.size();

    if (estimated_position_.norm() < 1000.0) {
        initializePosition(valid_obs, nav, time);
    }

    Vector3d position = estimated_position_;
    double clock_bias = receiver_clock_bias_;

    for (int iter = 0; iter < spp_config_.max_iterations; ++iter) {
        std::vector<Vector3d> sat_positions;
        std::vector<double> obs_values;
        std::vector<double> pred_values;
        std::vector<double> weights;

        // Compute geodetic position for atmospheric corrections
        double rcv_lat, rcv_lon, rcv_h;
        ecef2geodetic(position, rcv_lat, rcv_lon, rcv_h);

        for (const auto& obs : valid_obs) {
            Vector3d sat_pos, sat_vel;
            double sat_clk, sat_clk_drift;

            // First pass: approximate transmission time
            double travel_time = obs.pseudorange / constants::SPEED_OF_LIGHT;
            GNSSTime tx_time = time - travel_time;
            if (!nav.calculateSatelliteState(obs.satellite, tx_time,
                                             sat_pos, sat_vel, sat_clk, sat_clk_drift))
                continue;

            // Second pass: correct for satellite clock
            tx_time = tx_time - sat_clk;
            if (!nav.calculateSatelliteState(obs.satellite, tx_time,
                                             sat_pos, sat_vel, sat_clk, sat_clk_drift))
                continue;

            // Sagnac correction via Earth rotation matrix
            double signal_travel = (sat_pos - position).norm() / constants::SPEED_OF_LIGHT;
            double angle = constants::OMEGA_E * signal_travel;
            Eigen::Matrix3d R;
            R << std::cos(angle),  std::sin(angle), 0,
                -std::sin(angle),  std::cos(angle), 0,
                0, 0, 1;
            Vector3d corrected_sat_pos = R * sat_pos;

            // Geometric range
            double geometric_range = (corrected_sat_pos - position).norm();

            // Elevation angle for atmospheric corrections and weighting
            auto geom = nav.calculateGeometry(position, corrected_sat_pos);
            double elevation = geom.elevation;

            // Skip low-elevation satellites
            if (elevation < 0.087) // ~5 degrees
                continue;

            // Tropospheric correction using Saastamoinen model
            double trop_delay = models::tropDelaySaastamoinen(position, elevation);

            // Ionospheric correction using Klobuchar model
            double iono_delay = 0.0;
            if (nav.ionosphere_model.valid) {
                double azimuth = geom.azimuth;
                iono_delay = models::ionoDelayKlobuchar(
                    rcv_lat, rcv_lon, azimuth, elevation,
                    time.tow,
                    nav.ionosphere_model.alpha,
                    nav.ionosphere_model.beta);
            }

            // TGD correction
            double tgd = 0.0;
            for (const auto& [sat_id, eph_vec] : nav.ephemeris_data) {
                if (sat_id == obs.satellite) {
                    for (const auto& eph : eph_vec) {
                        if (eph.valid) {
                            tgd = eph.tgd * constants::SPEED_OF_LIGHT;
                            break;
                        }
                    }
                    break;
                }
            }

            // Corrected pseudorange:
            // PR = rho + c*dtr - c*dts + T + I + TGD_effect
            // corrected_pr = PR + c*dts - T - I - TGD = rho + c*dtr
            double corrected_pr = obs.pseudorange
                                  + sat_clk * constants::SPEED_OF_LIGHT
                                  - trop_delay
                                  - iono_delay
                                  - tgd;

            // Predicted range (geometric + receiver clock bias)
            double predicted = geometric_range + clock_bias;

            sat_positions.push_back(corrected_sat_pos);
            obs_values.push_back(corrected_pr);
            pred_values.push_back(predicted);

            // Elevation-dependent weight
            double sin_el = std::sin(elevation);
            if (sin_el < 0.1) sin_el = 0.1;
            weights.push_back(sin_el * sin_el);
        }

        if (sat_positions.size() < 4) {
            solution.status = SolutionStatus::NONE;
            return solution;
        }

        int n = sat_positions.size();
        VectorXd observations = Eigen::Map<VectorXd>(obs_values.data(), n);
        VectorXd predicted = Eigen::Map<VectorXd>(pred_values.data(), n);
        MatrixXd H = calculateGeometryMatrix(position, sat_positions);
        VectorXd residuals = observations - predicted;

        // Weighted least squares
        MatrixXd W = MatrixXd::Zero(n, n);
        for (int i = 0; i < n; ++i)
            W(i, i) = weights[i];

        MatrixXd HTWH = H.transpose() * W * H;
        VectorXd dx = HTWH.ldlt().solve(H.transpose() * W * residuals);

        position += dx.head<3>();
        clock_bias += dx(3);

        if (dx.head<3>().norm() < 1e-4) {
            solution.iterations = iter + 1;
            break;
        }
        if (iter == spp_config_.max_iterations - 1)
            solution.iterations = spp_config_.max_iterations;
    }

    solution.position_ecef = position;
    solution.receiver_clock_bias = clock_bias;

    // Calculate DOP from satellite geometry using final position
    auto sat_states = calculateSatelliteStates(valid_obs, nav, time);
    std::vector<Vector3d> final_sat_positions;
    for (const auto& obs : valid_obs) {
        auto it = sat_states.find(obs.satellite);
        if (it != sat_states.end() && it->second.valid)
            final_sat_positions.push_back(it->second.position);
    }
    if (final_sat_positions.size() >= 4) {
        MatrixXd H = calculateGeometryMatrix(position, final_sat_positions);
        calculateDOP(H, solution);
    }
    for (const auto& obs : valid_obs)
        solution.satellites_used.push_back(obs.satellite);

    estimated_position_ = position;
    receiver_clock_bias_ = clock_bias;
    return solution;
}

std::vector<Observation> SPPProcessor::validateObservations(const ObservationData& obs,
                                                          const NavigationData& nav,
                                                          const GNSSTime& time) const {
    std::vector<Observation> valid_obs;

    for (const auto& observation : obs.observations) {
        // SPP uses L1 pseudorange only (L2 has different ionosphere delay)
        if (observation.signal != SignalType::GPS_L1CA) continue;

        // Check observation validity first
        if (!observation.valid || observation.pseudorange <= 0.0) {
            continue;
        }

        // Estimate transmission time to check for ephemeris validity
        double travel_time = observation.pseudorange / constants::SPEED_OF_LIGHT;
        GNSSTime tx_time = time - travel_time;

        // Check if ephemeris is available at the estimated transmission time
        if (!nav.hasEphemeris(observation.satellite, tx_time)) {
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

        // Estimate signal travel time and calculate transmission time
        double travel_time = obs.pseudorange / constants::SPEED_OF_LIGHT;
        GNSSTime tx_time = time - travel_time;

        // First: get satellite clock at approximate tx time
        state.valid = nav.calculateSatelliteState(obs.satellite, tx_time,
                                                state.position, state.velocity,
                                                state.clock_bias, state.clock_drift);
        if (state.valid) {
            // Correct tx time for satellite clock bias
            tx_time = tx_time - state.clock_bias;
            // Recompute at corrected tx time
            state.valid = nav.calculateSatelliteState(obs.satellite, tx_time,
                                                    state.position, state.velocity,
                                                    state.clock_bias, state.clock_drift);
        }
        states[obs.satellite] = state;
    }

    return states;
}

double SPPProcessor::calculatePredictedRange(const Vector3d& receiver_pos,
                                           const Vector3d& satellite_pos,
                                           double receiver_clock_bias_m,
                                           double satellite_clock_bias_s) const {
    double geometric_range = (satellite_pos - receiver_pos).norm();
    // receiver_clock_bias_m is in meters, satellite_clock_bias is in seconds
    double clock_correction = receiver_clock_bias_m - satellite_clock_bias_s * constants::SPEED_OF_LIGHT;
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
    total_epochs_processed_++;
    total_processing_time_ms_ += processing_time_ms;
    if (success) {
        successful_solutions_++;
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
    double lat, lon, h;
    ecef2geodetic(ecef_pos, lat, lon, h);
    return GeodeticCoord(lat, lon, h);
}

Vector3d geodeticToEcef(const GeodeticCoord& geodetic_pos) {
    return geodetic2ecef(geodetic_pos.latitude, geodetic_pos.longitude, geodetic_pos.height);
}

} // namespace spp_utils

} // namespace libgnss
