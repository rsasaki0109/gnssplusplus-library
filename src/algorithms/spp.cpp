#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signal_policy.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/models/troposphere.hpp>
#include <libgnss++/models/ionosphere.hpp>
#include <stdexcept>
#include <chrono>
#include <cmath>

namespace libgnss {

namespace {

bool isPrimarySPPSignal(SignalType signal, const SPPProcessor::SPPConfig& config) {
    switch (signal) {
        case SignalType::GPS_L1CA:
        case SignalType::GAL_E1:
        case SignalType::QZS_L1CA:
            return true;
        case SignalType::GLO_L1CA:
            return config.enable_glonass;
        case SignalType::BDS_B1I:
        case SignalType::BDS_B1C:
            return config.enable_beidou;
        default:
            return false;
    }
}

GNSSSystem clockBiasGroup(GNSSSystem system) {
    switch (system) {
        case GNSSSystem::GPS:
        case GNSSSystem::QZSS:
            return GNSSSystem::GPS;
        case GNSSSystem::Galileo:
        case GNSSSystem::BeiDou:
        case GNSSSystem::GLONASS:
        case GNSSSystem::NavIC:
            return system;
        default:
            return GNSSSystem::UNKNOWN;
    }
}

bool usesSeparateClockBias(GNSSSystem group) {
    return group != GNSSSystem::UNKNOWN && group != GNSSSystem::GPS;
}

GNSSSystem selectReferenceClockGroup(const std::map<GNSSSystem, int>& group_counts) {
    const auto gps_it = group_counts.find(GNSSSystem::GPS);
    if (gps_it != group_counts.end() && gps_it->second > 0) {
        return GNSSSystem::GPS;
    }

    GNSSSystem best_group = GNSSSystem::UNKNOWN;
    int best_count = -1;
    for (const auto& [group, count] : group_counts) {
        if (count > best_count) {
            best_group = group;
            best_count = count;
        }
    }
    return best_group;
}

double groupDelayCorrectionMeters(const Observation& observation, const Ephemeris& eph) {
    switch (observation.satellite.system) {
        case GNSSSystem::GPS:
        case GNSSSystem::QZSS:
        case GNSSSystem::Galileo:
            return eph.tgd * constants::SPEED_OF_LIGHT;
        case GNSSSystem::BeiDou:
            switch (observation.signal) {
                case SignalType::BDS_B1I:
                case SignalType::BDS_B1C:
                    return eph.tgd * constants::SPEED_OF_LIGHT;
                case SignalType::BDS_B2I:
                case SignalType::BDS_B2A:
                    return eph.tgd_secondary * constants::SPEED_OF_LIGHT;
                case SignalType::BDS_B3I:
                default:
                    return 0.0;
            }
        default:
            return 0.0;
    }
}

}  // namespace

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

    if (estimated_position_.norm() < 1000.0) {
        initializePosition(valid_obs, nav, time);
    }

    struct MeasurementModel {
        Observation observation;
        Vector3d satellite_position;
        GNSSSystem clock_group = GNSSSystem::UNKNOWN;
        double corrected_pseudorange = 0.0;
        double elevation = 0.0;
        double weight = 1.0;
    };

    auto buildMeasurements = [&](const Vector3d& current_position) {
        std::vector<MeasurementModel> measurements;
        measurements.reserve(valid_obs.size());

        double rcv_lat = 0.0;
        double rcv_lon = 0.0;
        double rcv_h = 0.0;
        ecef2geodetic(current_position, rcv_lat, rcv_lon, rcv_h);

        for (const auto& obs : valid_obs) {
            Vector3d sat_pos;
            Vector3d sat_vel;
            double sat_clk = 0.0;
            double sat_clk_drift = 0.0;

            double travel_time = obs.pseudorange / constants::SPEED_OF_LIGHT;
            GNSSTime tx_time = time - travel_time;
            if (!nav.calculateSatelliteState(obs.satellite, tx_time,
                                             sat_pos, sat_vel, sat_clk, sat_clk_drift)) {
                continue;
            }

            tx_time = tx_time - sat_clk;
            if (!nav.calculateSatelliteState(obs.satellite, tx_time,
                                             sat_pos, sat_vel, sat_clk, sat_clk_drift)) {
                continue;
            }

            double signal_travel = (sat_pos - current_position).norm() / constants::SPEED_OF_LIGHT;
            double angle = constants::OMEGA_E * signal_travel;
            Eigen::Matrix3d earth_rotation;
            earth_rotation << std::cos(angle),  std::sin(angle), 0.0,
                             -std::sin(angle),  std::cos(angle), 0.0,
                              0.0,              0.0,             1.0;
            Vector3d corrected_sat_pos = earth_rotation * sat_pos;

            auto geom = nav.calculateGeometry(current_position, corrected_sat_pos);
            if (geom.elevation < 0.087) {
                continue;
            }

            double trop_delay = models::tropDelaySaastamoinen(current_position, geom.elevation);

            const Ephemeris* eph = nav.getEphemeris(obs.satellite, tx_time);
            if (!eph) {
                continue;
            }

            double iono_delay = 0.0;
            if (nav.ionosphere_model.valid) {
                iono_delay = models::ionoDelayKlobuchar(
                    rcv_lat, rcv_lon, geom.azimuth, geom.elevation,
                    time.tow,
                    nav.ionosphere_model.alpha,
                    nav.ionosphere_model.beta);

                const double signal_frequency = signalFrequencyHz(obs.signal, eph);
                if (signal_frequency > 0.0) {
                    const double freq_scale = constants::GPS_L1_FREQ / signal_frequency;
                    iono_delay *= freq_scale * freq_scale;
                }
            }

            double corrected_pr = obs.pseudorange
                                  + sat_clk * constants::SPEED_OF_LIGHT
                                  - trop_delay
                                  - iono_delay
                                  - groupDelayCorrectionMeters(obs, *eph);

            double sin_el = std::sin(geom.elevation);
            if (sin_el < 0.1) {
                sin_el = 0.1;
            }

            MeasurementModel measurement;
            measurement.observation = obs;
            measurement.satellite_position = corrected_sat_pos;
            measurement.clock_group = clockBiasGroup(obs.satellite.system);
            measurement.corrected_pseudorange = corrected_pr;
            measurement.elevation = geom.elevation;
            measurement.weight = sin_el * sin_el;
            if (measurement.clock_group == GNSSSystem::UNKNOWN) {
                continue;
            }
            measurements.push_back(measurement);
        }

        return measurements;
    };

    Vector3d position = estimated_position_;
    double clock_bias = receiver_clock_bias_;
    std::map<GNSSSystem, double> bias_estimates;
    std::vector<MeasurementModel> final_measurements;
    GNSSSystem final_reference_group = GNSSSystem::UNKNOWN;
    std::vector<GNSSSystem> final_bias_groups;

    for (int iter = 0; iter < spp_config_.max_iterations; ++iter) {
        auto measurements = buildMeasurements(position);
        if (measurements.size() < 4) {
            solution.status = SolutionStatus::NONE;
            return solution;
        }

        std::map<GNSSSystem, int> group_counts;
        for (const auto& measurement : measurements) {
            group_counts[measurement.clock_group]++;
        }

        const GNSSSystem reference_group = selectReferenceClockGroup(group_counts);
        std::vector<GNSSSystem> bias_groups;
        if (spp_config_.model_intersystem_bias) {
            for (const auto& [group, count] : group_counts) {
                if (count <= 0 || group == reference_group || !usesSeparateClockBias(group)) {
                    continue;
                }
                bias_groups.push_back(group);
            }
        }

        const int n = static_cast<int>(measurements.size());
        const int num_unknowns = 4 + static_cast<int>(bias_groups.size());
        if (n < num_unknowns) {
            solution.status = SolutionStatus::NONE;
            return solution;
        }

        std::map<GNSSSystem, int> bias_columns;
        for (int i = 0; i < static_cast<int>(bias_groups.size()); ++i) {
            bias_columns[bias_groups[i]] = 4 + i;
        }

        MatrixXd H = MatrixXd::Zero(n, num_unknowns);
        VectorXd residuals = VectorXd::Zero(n);
        MatrixXd weighted_H = MatrixXd::Zero(n, num_unknowns);
        VectorXd weighted_residuals = VectorXd::Zero(n);

        for (int i = 0; i < n; ++i) {
            const auto& measurement = measurements[i];
            const Vector3d los = (measurement.satellite_position - position).normalized();
            const double geometric_range = (measurement.satellite_position - position).norm();

            H(i, 0) = -los(0);
            H(i, 1) = -los(1);
            H(i, 2) = -los(2);
            H(i, 3) = 1.0;

            double predicted = geometric_range + clock_bias;
            const auto bias_col_it = bias_columns.find(measurement.clock_group);
            if (bias_col_it != bias_columns.end()) {
                predicted += bias_estimates[measurement.clock_group];
                H(i, bias_col_it->second) = 1.0;
            }

            residuals(i) = measurement.corrected_pseudorange - predicted;

            const double sigma = std::sqrt(measurement.weight);
            weighted_H.row(i) = H.row(i) * sigma;
            weighted_residuals(i) = residuals(i) * sigma;
        }

        Eigen::ColPivHouseholderQR<MatrixXd> qr(weighted_H);
        if (qr.rank() < num_unknowns) {
            solution.status = SolutionStatus::NONE;
            return solution;
        }

        VectorXd dx = qr.solve(weighted_residuals);
        if (!dx.allFinite()) {
            solution.status = SolutionStatus::NONE;
            return solution;
        }

        position += dx.head<3>();
        clock_bias += dx(3);
        for (int i = 0; i < static_cast<int>(bias_groups.size()); ++i) {
            bias_estimates[bias_groups[i]] += dx(4 + i);
        }

        final_measurements = std::move(measurements);
        final_reference_group = reference_group;
        final_bias_groups = bias_groups;

        if (dx.head<3>().norm() < spp_config_.position_convergence_threshold) {
            solution.iterations = iter + 1;
            break;
        }
        if (iter == spp_config_.max_iterations - 1) {
            solution.iterations = spp_config_.max_iterations;
        }
    }

    final_measurements = buildMeasurements(position);
    if (final_measurements.size() < 4) {
        solution.status = SolutionStatus::NONE;
        return solution;
    }

    std::map<GNSSSystem, int> final_group_counts;
    for (const auto& measurement : final_measurements) {
        final_group_counts[measurement.clock_group]++;
    }
    final_reference_group = selectReferenceClockGroup(final_group_counts);
    final_bias_groups.clear();
    if (spp_config_.model_intersystem_bias) {
        for (const auto& [group, count] : final_group_counts) {
            if (count <= 0 || group == final_reference_group || !usesSeparateClockBias(group)) {
                continue;
            }
            final_bias_groups.push_back(group);
        }
    }

    std::map<GNSSSystem, int> final_bias_columns;
    for (int i = 0; i < static_cast<int>(final_bias_groups.size()); ++i) {
        final_bias_columns[final_bias_groups[i]] = 4 + i;
    }

    VectorXd final_residuals = VectorXd::Zero(static_cast<int>(final_measurements.size()));
    std::vector<Vector3d> final_sat_positions;
    final_sat_positions.reserve(final_measurements.size());
    for (int i = 0; i < static_cast<int>(final_measurements.size()); ++i) {
        const auto& measurement = final_measurements[i];
        const double geometric_range = (measurement.satellite_position - position).norm();
        double predicted = geometric_range + clock_bias;
        const auto bias_col_it = final_bias_columns.find(measurement.clock_group);
        if (bias_col_it != final_bias_columns.end()) {
            predicted += bias_estimates[measurement.clock_group];
        }
        final_residuals(i) = measurement.corrected_pseudorange - predicted;
        final_sat_positions.push_back(measurement.satellite_position);
    }

    solution.position_ecef = position;
    solution.receiver_clock_bias = clock_bias;
    solution.num_satellites = static_cast<int>(final_measurements.size());
    solution.num_frequencies = 1;
    solution.residual_rms = std::sqrt(final_residuals.squaredNorm() /
                                      static_cast<double>(final_measurements.size()));

    double lat = 0.0;
    double lon = 0.0;
    double height = 0.0;
    ecef2geodetic(position, lat, lon, height);
    solution.position_geodetic = GeodeticCoord(lat, lon, height);

    // Calculate DOP from satellite geometry using final position
    if (final_sat_positions.size() >= 4) {
        MatrixXd H = calculateGeometryMatrix(position, final_sat_positions);
        calculateDOP(H, solution);
    }

    for (int i = 0; i < static_cast<int>(final_measurements.size()); ++i) {
        solution.satellites_used.push_back(final_measurements[i].observation.satellite);
        solution.satellite_elevations.push_back(final_measurements[i].elevation);
        solution.satellite_residuals.push_back(final_residuals(i));
    }

    estimated_position_ = position;
    receiver_clock_bias_ = clock_bias;
    system_biases_.clear();
    return solution;
}

std::vector<Observation> SPPProcessor::validateObservations(const ObservationData& obs,
                                                          const NavigationData& nav,
                                                          const GNSSTime& time) const {
    std::vector<Observation> valid_obs;

    for (const auto& observation : obs.observations) {
        if (spp_config_.use_multi_constellation) {
            if (!isPrimarySPPSignal(observation.signal, spp_config_)) continue;
        } else if (observation.signal != SignalType::GPS_L1CA) {
            continue;
        }

        // Check observation validity first
        if (!observation.valid || !observation.has_pseudorange || observation.pseudorange <= 0.0) {
            continue;
        }

        // BeiDou GEO handling still needs tighter validation than the current
        // broadcast model provides. Keep MEO/IGSO enabled and gate GEO for now.
        if (signal_policy::isBeiDouGeoSatellite(observation.satellite)) {
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
